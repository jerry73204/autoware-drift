"""L2, the live planner's tracker: the car's inputs every control tick (50 Hz) that follow L1's
plan, predicted a second ahead on the same car model (the live planner's design, section 4.5).

What it follows. The plan's line (its offset n and heading error xi), its speed vx, and its
sideslip vy and yaw rate r as references, not as errors to kill: a slide the plan asked for is
held, and steering out of a planned rotation costs (Kapania and Gerdes). The references are
taken at the arclength s the prediction reaches (contouring), not at a clock: the car may be
behind or ahead of the plan's timing, and a new plan then moves the reference, not the steering
(the hand-over of 4.4). The plan's start time is not needed for that; it only tells a new plan.

What it predicts with. axle.py's car (steering and one acceleration request; the ECU's split;
PhysX's lateral tyre with the friction ellipse), in time, not in s: the spatial form divides
by the car's progress and breaks in a spin (4.5's revision), the temporal one keeps working.
States s, n, xi, vx, vy, r, delta; inputs delta's rate and the request a. The tyre loads are
taken at the accelerations the last solution predicted for each stage (shifted one tick): the
fixed point L1 solves for exactly, reached here across ticks.

The car's limits (SR3), as hard constraints: steering lock and rate, the motor's torque and
power, each wheel's brake (the rear's with the regen); each wheel's longitudinal force within
its grip (|use| <= 1) softly, at the full grip (L1 plans at 0.92, the difference is this
layer's authority, 4.4); the road's edges softly too, with a slack whose weight is far above
any tracking error's: a prediction that cannot stay inside must still give an input.

How it solves (real-time iteration, Diehl). One Gauss-Newton SQP step per tick from the last
solution shifted by a tick: the problem's residuals and constraints are linearised (one CasADi
Function, built once, C-compiled when a compiler is there) and the QP solved by HPIPM's
Riccati-structured interior point (the decision vector in its stage order, so CasADi hands it
the OCP structure). About 1.3 ms a tick in CasADi's virtual machine (25 stages of 0.04 s);
C-generating the linearisation saved 0.3 ms for a 35 s compile, so it is not done; acados would
do the same in generated C, and is not needed yet.

When it fails (section 5): `LqrTracker`, a time-varying LQR on the same model linearised along
the plan, unconstrained and clamped to the limits, is this tick's input when the QP fails or the
tick overruns its budget.
"""
import math
import time

import casadi as ca
import numpy as np

from race_planner import axle
from race_planner.ocp import STEER_RATE
from race_planner.window import node_functions

G = 9.81
NX, NU, NS = 8, 2, 2            # states s n xi vx vy r delta a_last; inputs rate a; slacks
NP = 6                          # stage parameters: kappa mu grade sliding ax_load ay_load
NR = 6                          # references per node: n xi vx vy r delta
# Tracking scales: the error that costs as much as one unit (the cost is the error over its
# scale, squared). The line's are what L1's envelope allows (0.5 m, 5 deg, 2 m/s: 4.4) halved;
# sideslip and yaw rate are tight enough to hold a planned slide, loose enough that the line
# and the speed can buy stability with them.
SCALE_X = np.array([0.2, 0.1, 0.7, 0.8, 0.3, 0.5])        # n xi vx vy r delta
SCALE_RATE, SCALE_A, SCALE_JERK = 1.5, 3.0, 25.0          # rad/s; m/s^2 off the plan's; m/s^3
TERMINAL = 10.0                 # the last node's weight: where the car is in a second counts
EDGE_QUAD, EDGE_LIN = 1e4, 1e3  # the edge slack's cost per m^2, per m: far above tracking
USE_QUAD, USE_LIN = 1e3, 1e2    # the grip slack's (in shares of the grip)
EDGE_MARGIN = 0.1               # m inside the edge (L1 keeps 0.3: the tracker's room is the rest)
A_BOX = (-20.0, 15.0)           # m/s^2: a bound for the QP only; the car's limits are the rows
BUDGET = 0.010                  # s per tick (the design's 10 ms at 50 Hz)
LQR_DRIVE_MARGIN = 0.5          # m/s^2: the fallback's drive beyond the plan's request
# The fallback's scales: with no grip limit and no constraint in its model, it damps the slide
# harder than the NMPC (the S-bend's test alone: 0.12 m less over the line at the edge).
LQR_SCALE_X = np.array([0.25, 0.1, 0.7, 0.4, 0.15, 0.3])


def _dynamics(car, grip=1.0):
    """f(x, u, p) -> xdot in time; acc(x, u, p) -> the body's (ax, ay); g(x, u, p) -> the car's
    limit rows (use x4, drive, power, front brake, rear brake) with their bounds."""
    x, u, p = ca.SX.sym("x", 7), ca.SX.sym("u", NU), ca.SX.sym("p", NP)
    s, n, xi, vx, vy, r, delta = (x[i] for i in range(7))
    kappa, mu, grade, sliding = p[0], grip * p[1], p[2], p[3]
    o = axle.dynamics(car, [vx, vy, r], [delta, u[1]], mu, [p[4], p[5]], sliding)
    sin_t = grade / ca.sqrt(1 + grade * grade)
    sdot = (vx * ca.cos(xi) - vy * ca.sin(xi)) / (1 - n * kappa)
    xdot = ca.vertcat(sdot, vx * ca.sin(xi) + vy * ca.cos(xi), r - kappa * sdot,
                      o["dx"][0] - G * sin_t * ca.cos(xi), o["dx"][1] + G * sin_t * ca.sin(xi),
                      o["dx"][2], u[0])
    f = ca.Function("f", [x, u, p], [xdot])
    acc = ca.Function("acc", [x, u, p], [ca.vertcat(*o["acc"])])
    # the limit rows are L1's (window.node_functions), at this layer's grip
    _, g1, lbg, ubg, _ = node_functions(car)
    x6 = ca.vertcat(vx, vy, r, n, xi, delta)
    rows = g1(x6, ca.vertcat(u[0], u[1], p[4], p[5]), ca.vertcat(kappa, mu, grade, sliding))
    sel = list(range(6, 14))
    g = ca.Function("g", [x, u, p], [rows[sel]])
    return f, acc, g, np.array(lbg, float)[sel], np.array(ubg, float)[sel]


def _rk4(f, x, u, p, h):
    k1 = f(x, u, p)
    k2 = f(x + 0.5 * h * k1, u, p)
    k3 = f(x + 0.5 * h * k2, u, p)
    k4 = f(x + h * k3, u, p)
    return x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def accel_limits(car, vx):
    """The request's range at speed vx the car can carry out (SR3): the motor's torque and power
    at the rear axle, the brakes' torque per wheel (the rear's plus the regen), less the road's
    load (the ECU's feedforward adds it back)."""
    R, v = car.r_wheel, max(abs(vx), 1.0)
    road = car.drag * vx * vx + car.roll * car.m * G
    drive = min(car.axle_torque_max / R, car.power_max / v)
    share = float(axle.brake_share(car, 8.0))
    regen = min(car.regen_torque_max, car.regen_power_max / max(v / R, 1.0))
    brake = min(2 * car.brake_torque_max / (share * R),
                (2 * car.brake_torque_max + regen) / ((1 - share) * R))
    return (-brake - road) / car.m, (drive - road) / car.m


class _Plan:
    """A plan's references along s, prepared once per plan: the states (vx vy r n xi delta) at
    its nodes, its request and load accelerations on its intervals."""

    def __init__(self, plan):
        self.src = plan
        self.s = np.asarray(plan["s"], float)
        self.X = np.asarray(plan["X"], float)
        self.U = np.asarray(plan["U"], float)
        self.su = self.s[:-1]

    def at(self, s):
        """References at the arclengths s (array): (n xi vx vy r delta) 6 x len(s), the request
        a, the plan's (ax, ay) and delta's rate."""
        X = self.X
        ref = np.vstack([np.interp(s, self.s, X[i]) for i in (3, 4, 0, 1, 2, 5)])
        U = np.vstack([np.interp(s, self.su, self.U[i]) for i in range(4)])
        return ref, U[1], U[2:4], U[0]


def _track_at(track, s):
    tr = track
    return (np.interp(s, tr.s, tr.kappa), np.interp(s, tr.s, tr.mu), np.interp(s, tr.s, tr.grade),
            np.interp(s, tr.s, tr.sliding), np.interp(s, tr.s, tr.w_left),
            np.interp(s, tr.s, tr.w_right))


def _plan_states(plan, s0, x0, N, dt):
    """A trajectory along the plan from s0: s advanced at the plan's speed, the plan's states
    there (the first node the car's), its inputs."""
    s = np.empty(N + 1)
    s[0] = s0
    for k in range(N):
        ref, _, _, _ = plan.at(np.array([s[k]]))
        s[k + 1] = s[k] + max(ref[2, 0], 0.5) * dt
    ref, a, acc, rate = plan.at(s)
    X = np.vstack([s, ref])
    X[:, 0] = x0[:7]
    return X, np.vstack([rate[:N], a[:N]]), acc[:, :N]


class LqrTracker:
    """The fallback (section 5): a time-varying LQR on the plan, axle.py's car linearised at the
    plan's states a horizon ahead (in time, at the plan's speed), the Riccati recursion solved
    backward every tick. Linear and unconstrained, so it cannot fail; its inputs are clamped to
    the car's limits. Valid near the plan (Peterson), which is where it is used."""

    def __init__(self, car, track, horizon=1.0, dt=0.04, grip=1.0, tick=0.02):
        self.car, self.track, self.dt, self.tick = car, track, dt, tick
        self.N = int(round(horizon / dt))
        self.rate = car.steer_rate_max or STEER_RATE
        f, _, _, _, _ = _dynamics(car, grip)
        x, u, p = ca.SX.sym("x", 7), ca.SX.sym("u", NU), ca.SX.sym("p", NP)
        y = f(x, u, p)[1:]                        # s is not tracked: references are taken at it
        jac = ca.Function("lin", [x, u, p], [ca.jacobian(y, x)[:, 1:], ca.jacobian(y, u)])
        self.jac = jac.map(self.N, "serial")
        # on (n xi vx vy r delta) and (rate, a): the NMPC's, sideslip and yaw rate tighter
        self.Q = np.diag(1.0 / LQR_SCALE_X ** 2)
        self.R = np.diag([1.0 / SCALE_RATE ** 2, 1.0 / SCALE_A ** 2 + 1.0 / (SCALE_JERK * dt) ** 2])
        self.delta = None
        self._plan = None

    def reset(self, delta=None):
        self.delta = delta

    def gains(self, state, plan):
        """The plan's references from the car's s and the first stage's gain K (u = u_ref - K e)."""
        if self._plan is None or self._plan.src is not plan:
            self._plan = _Plan(plan)
        pl, N, dt = self._plan, self.N, self.dt
        x0 = np.array([state["s"], state["n"], state["xi"], state["vx"], state["vy"],
                       state["r"], state["delta"] if self.delta is None else self.delta])
        X, U, acc = _plan_states(pl, state["s"], x0, N, dt)
        Xr = X.copy()
        Xr[1:, 0] = pl.at(np.array([state["s"]]))[0][:, 0]     # the reference at the car's s
        kappa, mu, grade, sl, _, _ = _track_at(self.track, Xr[0, :N])
        P = np.vstack([kappa, mu, grade, sl, acc])
        A, B = (np.asarray(m) for m in self.jac(Xr[:, :N], U, P))
        A = A.reshape(6, 6, N, order="F").transpose(2, 0, 1)     # the map's blocks side by side
        B = B.reshape(6, 2, N, order="F").transpose(2, 0, 1)
        Pm = TERMINAL * self.Q
        K = None
        for k in range(N - 1, -1, -1):
            Ac = A[k]
            Ad = np.eye(6) + dt * Ac + 0.5 * dt * dt * Ac @ Ac
            Bd = dt * B[k] + 0.5 * dt * dt * Ac @ B[k]
            BtP = Bd.T @ Pm
            K = np.linalg.solve(self.R + BtP @ Bd, BtP @ Ad)
            Pm = self.Q + Ad.T @ Pm @ (Ad - Bd @ K)
            Pm = 0.5 * (Pm + Pm.T)
        return Xr, U, K, x0

    def step(self, state, plan, plan_t0=None):
        c = time.perf_counter()
        Xr, U, K, x0 = self.gains(state, plan)
        e = x0[1:] - Xr[1:, 0]
        e[1] = (e[1] + math.pi) % (2 * math.pi) - math.pi
        u = U[:, 0] - K @ e
        # no more drive than the plan's and a little: the linear model knows no grip, and drive
        # past the plan's in a slide spins the rear (the S-bend's test: lateral rms 0.58 -> 0.35 m)
        u[1] = min(u[1], max(U[1, 0], 0.0) + LQR_DRIVE_MARGIN)
        out = self.apply(state, x0[6], u[0], u[1])
        out.update(ok=True, solve_s=time.perf_counter() - c, fallback=True)
        return out

    def apply(self, state, delta, rate, a):
        """The command from the inputs: delta's rate over a tick within the rate and the lock,
        the request within what the car can carry out."""
        car = self.car
        rate = float(np.clip(rate, -self.rate, self.rate))
        delta = float(np.clip(delta + rate * self.tick, -car.steer_max, car.steer_max))
        lo, hi = accel_limits(car, state["vx"])
        self.delta = delta
        return {"accel": float(np.clip(a, lo, hi)), "tire_angle": delta}


class Tracker:
    """The NMPC (real-time iteration) over `horizon` seconds on a grid of `dt`, built once.

    step(state, plan, plan_t0) -> {accel, tire_angle, ok, solve_s, fallback}: state is the car's
    (sim.Plant's dict: s n xi vx vy r delta), plan L1's (window.Window.solve's dict), plan_t0
    its start time on the caller's clock. tire_angle is the commanded angle, delta integrated
    over the tick at the first stage's rate; accel the first stage's request."""

    def __init__(self, car, track, horizon=1.0, dt=0.04, tick=0.02, grip=1.0, budget=BUDGET,
                 qpsol="hpipm"):
        self.car, self.track, self.dt, self.tick, self.budget = car, track, dt, tick, budget
        self.N = N = int(round(horizon / dt))
        self.rate = car.steer_rate_max or STEER_RATE
        self.half = 0.5 * car.width
        f, acc, g, lbg, ubg = _dynamics(car, grip)
        self.acc = acc.map(N, "serial")
        # the decision vector, stage by stage: x_k u_k sl_k, then x_N (HPIPM's own order)
        xs = [ca.SX.sym(f"x{k}", NX) for k in range(N + 1)]
        us = [ca.SX.sym(f"u{k}", NU) for k in range(N)]
        sls = [ca.SX.sym(f"sl{k}", NS) for k in range(N)]
        w = ca.vertcat(*[ca.vertcat(xs[k], us[k], sls[k]) for k in range(N)], xs[N])
        P = ca.SX.sym("P", NP, N)           # stage parameters
        REF = ca.SX.sym("REF", NR, N)       # references at x_1 .. x_N
        AREF = ca.SX.sym("AREF", N)         # the plan's request on each stage
        WL, WR = ca.SX.sym("WL", N), ca.SX.sym("WR", N)   # the free width at x_0 .. x_N-1
        par = ca.vertcat(ca.vec(P), ca.vec(REF), AREF, WL, WR)
        res, lin, cons, lbc, ubc = [], 0, [], [], []
        isx = 1.0 / SCALE_X
        for k in range(N):
            xk, uk, sk, xn = xs[k], us[k], sls[k], xs[k + 1]
            # the request's change is against the last stage's, carried as a state (a cost
            # that coupled two stages' inputs would break the QP's stage structure)
            res += [uk[0] / SCALE_RATE, (uk[1] - AREF[k]) / SCALE_A,
                    (uk[1] - xk[7]) / (SCALE_JERK * dt)]
            e = xn[1:7] - REF[:, k]
            wk = TERMINAL if k == N - 1 else 1.0
            res += [math.sqrt(wk) * e[i] * isx[i] for i in range(NR)]
            res += [math.sqrt(EDGE_QUAD) * sk[0], math.sqrt(USE_QUAD) * sk[1]]
            lin += EDGE_LIN * sk[0] + USE_LIN * sk[1]
            # the stage's rows in HPIPM's order: its dynamics, then its own constraints
            cons.append(ca.vertcat(_rk4(f, xk[:7], uk, P[:, k], dt), uk[1]) - xn)
            lbc.append(np.zeros(NX)); ubc.append(np.zeros(NX))
            gk = g(xk[:7], uk, P[:, k])
            cons += [gk[:4] - sk[1], gk[:4] + sk[1], gk[4:]]
            lbc += [-np.inf * np.ones(4), lbg[:4], lbg[4:]]
            ubc += [ubg[:4], np.inf * np.ones(4), ubg[4:]]
            cons += [xk[1] - sk[0], xk[1] + sk[0]]     # the edges at x_k: the stage's own
            lbc += [[-np.inf], [0.0]]
            ubc += [[0.0], [np.inf]]
        R = ca.vertcat(*res)
        C = ca.vertcat(*cons)
        # the edges' bounds move with the free width (parameters): those rows less their bound
        shift = []
        for k in range(N):
            shift += [ca.SX.zeros(NX), ca.SX.zeros(12), WL[k], -WR[k]]
        C = C - ca.vertcat(*shift)
        self.lbc = np.concatenate([np.atleast_1d(np.asarray(v, float)) for v in lbc])
        self.ubc = np.concatenate([np.atleast_1d(np.asarray(v, float)) for v in ubc])
        J, Jc = ca.jacobian(R, w), ca.jacobian(C, w)
        nw = w.numel()
        H = ca.mtimes(J.T, J) + 1e-6 * ca.SX.eye(nw)
        grad = ca.mtimes(J.T, R) + ca.gradient(lin, w)
        self.qp = ca.Function("rti_qp", [w, par], [H, grad, C, Jc])
        opts = {"error_on_fail": False, "print_problem": False}
        if qpsol == "hpipm":
            opts.update({"hpipm": {"iter_max": 50, "mode": "balance", "res_g_max": 1e-5,
                                   "res_b_max": 1e-6, "res_d_max": 1e-6, "res_m_max": 1e-6}})
        elif qpsol == "qrqp":
            opts.update({"print_iter": False, "print_header": False, "print_info": False})
        self.solver = ca.conic("rti", qpsol, {"h": H.sparsity(), "a": Jc.sparsity()}, opts)
        self.lqr = LqrTracker(car, track, horizon=horizon, dt=dt, grip=grip, tick=tick)
        # the variables' box: the lock and the rate, a QP-only request bound, slacks >= 0
        lb, ub = [], []
        for k in range(N):
            lb += [-np.inf] * 6 + [-car.steer_max, -np.inf, -self.rate, A_BOX[0], 0.0, 0.0]
            ub += [np.inf] * 6 + [car.steer_max, np.inf, self.rate, A_BOX[1], np.inf, np.inf]
        lb += [-np.inf] * 6 + [-car.steer_max, -np.inf]
        ub += [np.inf] * 6 + [car.steer_max, np.inf]
        self.lbw, self.ubw = np.array(lb), np.array(ub)
        self.reset()

    def reset(self):
        """Forget the last solution (a new run, or the car leaving the plan for good)."""
        self.W = None           # the last solution: X (NX x N+1), U (NU x N), its loads (2 x N)
        self.delta = None       # the commanded tire angle
        self.a_prev = 0.0
        self.plan = None
        self.fails = 0
        self.lqr.reset()

    # -- the warm start ---------------------------------------------------------------------
    def _guess(self, x0, plan):
        """The last solution shifted a tick (interpolated on the grid), or the plan from the car."""
        N, dt = self.N, self.dt
        if self.W is None:
            X, U, acc = _plan_states(plan, x0[0], x0, N, dt)
            X = np.vstack([X, np.concatenate([[x0[7]], U[1]])])
            return X, U, acc
        X, U, acc = self.W
        tk = np.arange(N + 1) * dt
        tq = np.minimum(tk + self.tick, tk[-1])
        Xs = np.vstack([np.interp(tq, tk, X[i]) for i in range(NX)])
        tu = tk[:-1]
        Us = np.vstack([np.interp(tq[:-1], tu, U[i]) for i in range(NU)])
        accs = np.vstack([np.interp(tq[:-1], tu, acc[i]) for i in range(2)])
        # s past the last node: extrapolated at its speed, so the tail's references move on
        Xs[0, -1] = X[0, -1] + max(X[3, -1], 0.0) * self.tick
        Xs[:, 0] = x0
        return Xs, Us, accs

    def _params(self, X, acc, plan):
        N = self.N
        ref, a, _, _ = plan.at(X[0, 1:])
        _, aref, _, _ = plan.at(X[0, :N])
        kappa, mu, grade, sl, _, _ = _track_at(self.track, X[0, :N])
        _, _, _, _, wl, wr = _track_at(self.track, X[0, :N])
        m = self.half + EDGE_MARGIN
        P = np.vstack([kappa, mu, grade, sl, acc])
        return np.concatenate([P.ravel(order="F"), ref.ravel(order="F"), aref, wl - m, wr - m])

    def _pack(self, X, U):
        N = self.N
        Z = np.vstack([X[:, :N], U, np.zeros((NS, N))])
        return np.concatenate([Z.ravel(order="F"), X[:, N]])

    def _unpack(self, w):
        N, st = self.N, NX + NU + NS
        Z = w[:N * st].reshape(st, N, order="F")
        X = np.hstack([Z[:NX], w[N * st:].reshape(NX, 1)])
        return X, Z[NX:NX + NU], Z[NX + NU:]

    # -- one tick ---------------------------------------------------------------------------
    def step(self, state, plan, plan_t0=None):
        c = time.perf_counter()
        if self.plan is None or self.plan.src is not plan:
            self.plan = _Plan(plan)
        if self.delta is None:
            self.delta = float(state["delta"])
        x0 = np.array([state["s"], state["n"], state["xi"], state["vx"], state["vy"],
                       state["r"], self.delta, self.a_prev])
        X, U, acc = self._guess(x0, self.plan)
        p = self._params(X, acc, self.plan)
        w = self._pack(X, U)
        H, grad, C, Jc = self.qp(w, p)
        lbw, ubw = self.lbw - w, self.ubw - w
        lbw[:NX] = ubw[:NX] = 0.0                     # x_0 is the car's: no step there
        sol = self.solver(h=H, g=grad, a=Jc, lba=self.lbc - C, uba=self.ubc - C,
                          lbx=lbw, ubx=ubw)
        dw = np.asarray(sol["x"]).ravel()
        ok = bool(self.solver.stats()["success"]) and np.all(np.isfinite(dw))
        solve_s = time.perf_counter() - c
        if ok:
            Xn, Un, Sn = self._unpack(w + dw)
            accn = np.asarray(self.acc(Xn[:7, :self.N], Un, p[:NP * self.N].reshape(
                NP, self.N, order="F"))).reshape(2, self.N)
            self.W = (Xn, Un, accn)
            self.fails = 0
        else:
            self.W = None
            self.fails += 1
        if ok and solve_s <= self.budget:
            self.lqr.delta = self.delta
            out = self.lqr.apply(state, self.delta, Un[0, 0], Un[1, 0])
            fallback = False
        else:
            self.lqr.delta = self.delta
            out = self.lqr.step(state, plan, plan_t0)
            fallback = True
        self.delta = out["tire_angle"]
        self.a_prev = out["accel"]
        out.update(ok=ok, solve_s=solve_s, fallback=fallback,
                   edge_slack=float(np.max(Sn[0])) if ok else None)
        return out
