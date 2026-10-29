"""L1, the live planner's medium-range layer: the fastest way through the next W metres of track
from the car's (predicted) state, re-solved about once a second (the live planner's design,
section 4.4).

The problem is ocp.py's spatial minimum-time problem on axle.py's car (steering rate and one
acceleration request; the ECU's split of it into wheel forces), over a window of the track:

- states per node: vx, vy, r, n, xi, delta (as ocp.py); controls per interval: delta's rate,
  the acceleration request a, and the accelerations the loads are taken at (equal to the
  model's);
- the window starts at the given state (all six) and ends no faster than v_end (L0's speed
  there, or the quasi-steady profile's): a window must never end at a speed the next corner
  cannot take;
- the car's limits (SR3): steering lock and rate, the motor's torque and power at the rear
  axle, each wheel's brake (the rear's with what the motor regenerates), the rear wheels' top
  speed (the motor's rpm), and every wheel's longitudinal force within its grip;
- the road's width less the car's half width and a margin, soft (an exact penalty, far above
  any time the road's last metre could buy) but at the window's end: the car's own state may
  already be past it, or bound to cross it, and a plan must still come back (the live loop
  found 3 of 9 solves "infeasible" that way); forward along the track.

Built once (IPOPT on a C-compiled, stage-mapped problem: `Window`); each solve sets the
parameters and the warm start. The warm start is the last plan shifted by the distance driven,
its tail the cold guess from where the plan is left (`Window.warm`).
"""
import hashlib
import math
import os
import shutil
import subprocess
import tempfile
import time

from dataclasses import replace

import casadi as ca
import numpy as np

from race_planner import axle, model
from race_planner.ocp import STEER_RATE

G = 9.81
REG_STEER, REG_ACCEL = 1e-2, 1e-4
END_SLIP, END_HEADING = 0.15, 0.35      # the window's last node: vy / vx, xi (rad)
DROP = 0.2                              # the share of a plan's end not reused as a warm start
NZ = 11                                 # a stage's variables: x (6), u (4), edge slack (1)
EDGE_PENALTY = 10.0                     # s per metre past the road's edge (less the margin)
CACHE = os.environ.get("RACE_PLANNER_CACHE", os.path.expanduser("~/.cache/race_planner"))
BLEND = 10                              # nodes over which a warm start's tail joins the plan
X_SCALE = np.array([20.0, 2.0, 1.0, 3.0, 0.3, 0.3])
U_SCALE = np.array([1.0, 10.0, 10.0, 10.0])
TRACK_FIELDS = ("s", "x", "y", "z", "psi", "kappa", "w_left", "w_right", "mu", "grade", "sliding")
ROWS = ("ax", "ay", "fz_fl", "fz_fr", "fz_rl", "fz_rr", "use_fl", "use_fr", "use_rl", "use_rr",
        "drive", "power", "brake_front", "brake_rear", "sdot")


NLP_FUNCTIONS = ("nlp", "nlp_f", "nlp_g", "nlp_grad_f", "nlp_jac_g", "nlp_grad", "nlp_hess_l")


def _compiled_nlpsol(nlp, opts, hess=None):
    """IPOPT on nlp with its functions C-compiled (the Lagrangian's Hessian `hess` where given,
    CasADi's own otherwise): generated once, compiled once per distinct code (cached by its
    hash under CACHE), loaded; None without a compiler."""
    probe = ca.nlpsol("l1", "ipopt", nlp, opts)
    d = tempfile.mkdtemp(prefix="race_planner_")
    cg = ca.CodeGenerator("l1_nlp.c", {"with_header": False})
    for name in NLP_FUNCTIONS:
        cg.add(hess if name == "nlp_hess_l" and hess is not None
               else probe.get_function(name) if name != "nlp" else probe.oracle())
    cg.generate(d + os.sep)
    src = os.path.join(d, "l1_nlp.c")
    key = hashlib.sha1(open(src, "rb").read()).hexdigest()[:16]
    try:
        return _load(src, key, opts)
    finally:
        shutil.rmtree(d, ignore_errors=True)


def _load(src, key, opts):
    """The library for code `src` (compiled into CACHE unless there already), loaded."""
    os.makedirs(CACHE, exist_ok=True)
    lib = os.path.join(CACHE, f"l1_nlp_{key}.so")
    if not os.path.exists(lib):
        tmp = f"{lib}.{os.getpid()}.tmp"
        try:
            subprocess.run(["gcc", "-O2", "-fPIC", "-shared", src, "-o", tmp, "-lm"],
                           check=True, capture_output=True)
        except (OSError, subprocess.CalledProcessError):
            return None
        os.replace(tmp, lib)
    return ca.nlpsol("l1", "ipopt", lib, opts)


def node_functions(car):
    """f(x, u, p) -> (dx/ds, dt/ds); g(x, u, p) -> path constraints within (lbg, ubg)."""
    x, u, p = ca.SX.sym("x", 6), ca.SX.sym("u", 4), ca.SX.sym("p", 4)   # kappa mu grade sliding
    vx, vy, r, n, xi, delta = (x[i] for i in range(6))
    kappa, mu, grade = p[0], p[1], p[2]
    o = axle.dynamics(car, [vx, vy, r], [delta, u[1]], mu, [u[2], u[3]], p[3])
    sin_t = grade / ca.sqrt(1 + grade * grade)
    dvx = o["dx"][0] - G * sin_t * ca.cos(xi)
    dvy = o["dx"][1] + G * sin_t * ca.sin(xi)
    sdot = (vx * ca.cos(xi) - vy * ca.sin(xi)) / (1 - n * kappa)
    dt = ca.vertcat(dvx, dvy, o["dx"][2], vx * ca.sin(xi) + vy * ca.cos(xi),
                    r - kappa * sdot, u[0])
    f = ca.Function("f", [x, u, p], [dt / sdot, 1 / sdot])
    R = car.r_wheel
    omega = vx / R
    regen = ca.fmin(car.regen_torque_max, car.regen_power_max / ca.fmax(omega, 1.0))
    share = o["share"]
    # each row in units of its own limit, so the solver sees numbers near one (loads in the car's
    # weight, torques in their limits, power in the motor's)
    mg = car.m * G
    t_drive, t_brake = car.axle_torque_max / R, car.brake_torque_max
    g = ca.vertcat(o["acc"][0] - u[2], o["acc"][1] - u[3],
                   *[fz / mg for fz in o["fz"]], *o["use"],
                   o["drive"] / t_drive, o["drive"] * vx / car.power_max,
                   0.5 * share * o["brake"] * R / t_brake,      # each front wheel's brake torque
                   ((1 - share) * o["brake"] * R - regen) / t_brake,   # the rear pair's friction
                   sdot)
    lbg = [0, 0, 0, 0, 0, 0, -1, -1, -1, -1, 0, -ca.inf, 0, -ca.inf, 0.5]
    ubg = [0, 0, ca.inf, ca.inf, ca.inf, ca.inf, 1, 1, 1, 1, 1, 1, 1, 2, ca.inf]
    out = ca.Function("out", [x, u, p], [ca.vertcat(*o["fx"]), ca.vertcat(*o["fy"]),
                                         ca.vertcat(*o["fz"]), ca.vertcat(*o["use"]),
                                         ca.vertcat(*o["alpha"])])
    return f, ca.Function("g", [x, u, p], [g]), lbg, ubg, out


def rk4(f, x, u, p, h):
    k1, t1 = f(x, u, p)
    k2, t2 = f(x + 0.5 * h * k1, u, p)
    k3, t3 = f(x + 0.5 * h * k2, u, p)
    k4, t4 = f(x + h * k3, u, p)
    return x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4), h / 6 * (t1 + 2 * t2 + 2 * t3 + t4)


def controls_for(X, s, car):
    """Controls consistent with states X (6 x n) along s: the steering rate from delta, the
    request from the speed's change (plus grade's share, left to the solver), the accelerations
    of the speed and the turn. A warm start for a stretch the last plan did not cover."""
    vx, vy, r, n, xi, delta = X
    ds = np.gradient(s)
    dvds = np.gradient(vx) / ds
    a = dvds * vx
    return np.vstack([np.gradient(delta) / ds * vx, a, a, vx * r])[:, :-1]


class Window:
    """The L1 problem for a window of `length` metres on a grid of `ds`, built once.

    How it is fast enough to be live (M1, 2026-10-30). The problem is written once per stage (a
    CasADi function of one interval: its RK4 step, its time, its limit rows) and mapped over the
    window, not expanded: IPOPT's functions are then a loop over one stage's code and its
    derivatives, which C-compiles in seconds (the expanded problem's Hessian was 130 MB of C, and
    evaluated in CasADi's virtual machine took 8 ms of each 12 ms iteration). The library is
    cached on disk by its code's hash (`CACHE`); the Lagrangian's Hessian is assembled from each
    stage's own block. An iteration then costs about 4 ms (12 before), half of it MUMPS's.
    Warm re-solves along Haruna: 56 of 56 feasible, p50 0.22 s, p95 0.54 s, p99 0.77 s, the
    plan's time within 0.06 % of a tightly converged IPOPT on the old form on average (2.1 % at
    worst, another local optimum)."""

    def __init__(self, car, ds=4.0, length=200.0, grip=0.92, margin=0.3, max_iter=400,
                 tol=1e-3, budget=None, warm=True, compile=True, ipopt=None):
        self.car, self.ds, self.grip, self.margin = car, ds, grip, margin
        self.N = N = int(round(length / ds)) + 1
        self.M = M = N - 1
        self.f, self.g, lbg, ubg, self.out = node_functions(car)
        self.lbg, self.ubg = np.array(lbg, float), np.array(ubg, float)
        ng = len(lbg)
        # one stage in the scaled variables
        xs, us, p = ca.SX.sym("xs", 6), ca.SX.sym("us", 4), ca.SX.sym("p", 4)
        x, u = xs * X_SCALE, us * U_SCALE
        xn, dt = rk4(self.f, x, u, p, ds)
        stage = ca.Function("l1_stage", [xs, us, p], [xn / X_SCALE, dt, self.g(x, u, p)])
        # the decision vector stage by stage (x_k, u_k), then x_N
        # (x_k, u_k, e_k: how far node k is past the road's edge less the margin)
        W = ca.MX.sym("w", NZ * M + 6)
        Pw = ca.MX.sym("p", 6, M)                # kappa mu grade sliding, the free width l, r
        Pp = Pw[:4, :]
        Z = ca.reshape(W[:NZ * M], NZ, M)
        Xk, Uk, Ek, xN = Z[:6, :], Z[6:10, :], Z[10, :], W[NZ * M:]
        xn_s, dts, G = stage.map(M)(Xk, Uk, Pp)
        n = Xk[3, :] * X_SCALE[3]
        rows = ca.vertcat(xn_s - ca.horzcat(Xk[:, 1:], xN), G,     # each stage's, together
                          n + Ek + Pw[5, :], n - Ek - Pw[4, :])
        sx = X_SCALE
        end = ca.vertcat(sx[1] * xN[1] + END_SLIP * sx[0] * xN[0],
                         sx[1] * xN[1] - END_SLIP * sx[0] * xN[0])
        a = Uk[1, :] * U_SCALE[1]
        T = (ca.sum2(dts) + REG_STEER * ds * ca.sumsqr(Uk[0, :] * U_SCALE[0])
             + REG_ACCEL * ca.sumsqr(a[1:] - a[:-1]) + EDGE_PENALTY * ca.sum2(Ek))
        self.lbg_all = np.concatenate([np.tile(np.r_[np.zeros(6), self.lbg, 0.0, -np.inf], M),
                                       [0.0, -np.inf]])
        self.ubg_all = np.concatenate([np.tile(np.r_[np.zeros(6), self.ubg, np.inf, 0.0], M),
                                       [np.inf, 0.0]])
        per = 6 + ng + 2
        # warm-started in its multipliers too (shifted with the window), so the barrier starts
        # low (cold: IPOPT's own start, for the first plan and after a failure)
        extra, ipopt = ipopt or {}, {"max_iter": max_iter, "tol": tol, "acceptable_tol": 1e-3,
                                     "acceptable_iter": 5, "print_level": 0, "sb": "yes"}
        if warm:
            ipopt.update({"mu_init": 1e-6, "warm_start_init_point": "yes",
                          "warm_start_bound_push": 1e-6, "warm_start_slack_bound_push": 1e-6,
                          "warm_start_mult_bound_push": 1e-6})
        if budget is not None:
            ipopt["max_wall_time"] = float(budget)
        opts = {"print_time": False, "error_on_fail": False, "ipopt": {**ipopt, **extra}}
        nlp = {"x": W, "p": ca.vec(Pw), "f": T, "g": ca.vertcat(ca.vec(rows), end)}
        # the Lagrangian's Hessian by stages: CasADi's own, by colouring the whole map, took
        # 1.8 ms an iteration; each stage's own block is one pass of one stage
        lam_f, lam_g = ca.MX.sym("lam_f"), ca.MX.sym("lam_g", per * M + 2)
        zs, ps, lf, lm = ca.SX.sym("z", NZ), ca.SX.sym("p", 4), ca.SX.sym("lf"), ca.SX.sym("l", 6 + ng)
        o = stage(zs[:6], zs[6:10], ps)
        Hk, _ = ca.hessian(lf * o[1] + ca.dot(lm[:6], o[0]) + ca.dot(lm[6:], o[2]), zs)
        hs = ca.Function("l1_stage_hess", [zs, ps, lf, lm], [Hk])
        Hm = hs.map(M)(Z, Pp, lam_f, ca.reshape(lam_g[:per * M], per, M)[:6 + ng, :])
        reg = T - ca.sum2(dts)                   # the regularisation's, constant
        Hreg = ca.Function("hreg", [W], [ca.hessian(reg, W)[0]])(np.zeros(NZ * M + 6))
        H = ca.diagcat(*ca.horzsplit(Hm, NZ), ca.MX(6, 6)) + lam_f * Hreg
        hess = ca.Function("nlp_hess_l", [W, ca.vec(Pw), lam_f, lam_g], [ca.triu(H)],
                           ["x", "p", "lam_f", "lam_g"], ["triu_hess_gamma_x_x"])
        self.solver = _compiled_nlpsol(nlp, opts, hess) if compile else None
        if self.solver is None:
            self.solver = ca.nlpsol("l1", "ipopt", nlp, opts)
        self.budget = budget
        self.per_stage = per                     # each interval's rows: dynamics, path, edges
        self.lam = None                          # the last solve's multipliers (rows)
        self.lam_x = None                        # and the bounds'

    def bounds(self, wl, wr, x0, v_end):
        """The decision vector's bounds (scaled): the steering's lock and rate, the heading, the
        motor's top speed; the window's start fixed at x0; its end no faster than v_end, near
        straight and on the road less the car's half width and the margin (before it the road
        is soft: rows with a penalty, `EDGE_PENALTY`)."""
        car, M, N, inf = self.car, self.M, self.N, np.inf
        half = 0.5 * car.width
        rate = car.steer_rate_max or STEER_RATE
        lo = np.tile([-inf, -inf, -inf, 0.0, -math.pi / 2, -car.steer_max], (N, 1)).T
        hi = np.tile([car.wheel_speed_max or inf, inf, inf, 0.0, math.pi / 2, car.steer_max],
                     (N, 1)).T
        lo[3], hi[3] = -inf, inf
        lo[3, N - 1], hi[3, N - 1] = -(wr[-1] - half - self.margin), wl[-1] - half - self.margin
        hi[0, N - 1] = min(hi[0, N - 1], v_end)
        lo[4, N - 1], hi[4, N - 1] = -END_HEADING, END_HEADING
        lo[:, 0] = hi[:, 0] = x0                            # the window starts at the state
        ulo = np.tile([-rate, -inf, -inf, -inf], (M, 1)).T
        uhi = np.tile([rate, inf, inf, inf], (M, 1)).T
        return self.pack(lo, ulo, 0.0), self.pack(hi, uhi, inf)

    def pack(self, X, U, e=0.0):
        """The scaled decision vector from X (6 x N), U (4 x M) and the edge slack e."""
        Z = np.vstack([X[:, :self.M] / X_SCALE[:, None], U / U_SCALE[:, None],
                       np.full(self.M, e)])
        return np.concatenate([Z.ravel(order="F"), X[:, self.M] / X_SCALE])

    def unpack(self, w):
        M = self.M
        Z = np.asarray(w).ravel()[:NZ * M].reshape(NZ, M, order="F")
        X = np.hstack([Z[:6] * X_SCALE[:, None], (np.asarray(w).ravel()[NZ * M:] * X_SCALE)[:, None]])
        return X, Z[6:10] * U_SCALE[:, None]

    def params(self, track, i0):
        """The window's track parameters from node i0 of `track` (on this window's grid)."""
        sl = slice(i0, i0 + self.N)
        P = np.vstack([track.kappa[sl], self.grip * track.mu[sl], track.grade[sl],
                       track.sliding[sl]])[:, :self.M]
        return P, track.w_left[sl], track.w_right[sl], track.s[sl]

    def solve(self, track, i0, x0, v_end, X_init, U_init):
        """The plan from node i0 starting at state x0, warm-started from X_init (6 x N) and
        U_init (4 x M). Returns a dict: X, U, s, t (per node), time, ok, status, iters, solve_s,
        defect (the largest shooting gap)."""
        P, wl, wr, s = self.params(track, i0)
        lbx, ubx = self.bounds(wl, wr, x0, v_end)
        half = 0.5 * self.car.width
        Pw = np.vstack([P, wl[:self.M] - half - self.margin, wr[:self.M] - half - self.margin])
        args = {"x0": np.clip(self.pack(X_init, U_init), lbx, ubx), "p": Pw.ravel(order="F"),
                "lbx": lbx, "ubx": ubx, "lbg": self.lbg_all, "ubg": self.ubg_all}
        # the multipliers only as a pair: rows' without the bounds' (another window's handed
        # over, as live.py does from its cold one) cost more iterations than none (p95 1.0 s
        # against 0.56 s on Haruna)
        if (self.lam is not None and len(self.lam) == len(self.lbg_all)
                and self.lam_x is not None and len(self.lam_x) == len(lbx)):
            args["lam_g0"], args["lam_x0"] = self.lam, self.lam_x
        c = time.time()
        sol = self.solver(**args)
        dt = time.time() - c
        st = self.solver.stats()
        self.lam = np.asarray(sol["lam_g"]).ravel()
        self.lam_x = np.asarray(sol["lam_x"]).ravel()
        Xv, Uv = self.unpack(sol["x"])
        return self._result(P, s, Xv, Uv, bool(st["success"]), st["return_status"],
                            st["iter_count"], dt, i0)

    def _result(self, P, s, Xv, Uv, ok, status, iters, dt, i0):
        seg, gap, over = [], 0.0, 0.0
        for k in range(self.M):
            xn, tk = rk4(self.f, Xv[:, k], Uv[:, k], P[:, k], self.ds)
            seg.append(float(tk))
            gap = max(gap, float(np.max(np.abs(np.asarray(xn).ravel() - Xv[:, k + 1]))))
            gv = np.asarray(self.g(Xv[:, k], Uv[:, k], P[:, k])).ravel()
            over = max(over, float(np.max(np.maximum(gv - self.ubg, self.lbg - gv))))
        # a solve stopped by its budget or its iterations is still a plan if it is feasible:
        # the car is better off with it than with the last one (re-solved next second anyway)
        feasible = gap < 1e-2 and over < 1e-2
        ok = ok or status == "Solved_To_Acceptable_Level" or feasible
        return {"X": Xv, "U": Uv, "s": s, "t": np.concatenate([[0.0], np.cumsum(seg)]),
                "time": float(np.sum(seg)), "ok": ok, "status": status,
                "iters": iters, "solve_s": dt, "defect": gap, "violation": over,
                "i0": i0}

    def section(self, track, i0):
        """The window's stretch of `track` (on this window's grid) from node i0."""
        sl = slice(i0, i0 + self.N)
        return replace(track, **{k: getattr(track, k)[sl] for k in TRACK_FIELDS})

    def warm(self, plan, k, track, v_start=None):
        """The warm start for re-solving k nodes on from `plan`: its nodes from k up to the last
        DROP of it, then a cold guess for the rest (the minimum-curvature line with the
        quasi-steady speed: it follows the road, which repeating the plan's last state does not,
        and the plan's own end is a receding horizon's, the least to be trusted). The cold
        guess starts where the plan is left, at the plan's speed there (v_start overrides), its
        offset and heading eased from the plan's over BLEND nodes: a guess that jumps where the
        two meet (10 m/s and a metre on Haruna) is a QP the re-solve cannot linearise well."""
        if self.lam is not None:                             # the multipliers, shifted too
            n, sh = self.M * self.per_stage, k * self.per_stage
            lam = self.lam.copy()
            lam[:n - sh] = self.lam[sh:n]
            self.lam = lam
        if self.lam_x is not None:
            n, sh = self.M * NZ, k * NZ
            lx = self.lam_x.copy()
            lx[:n - sh] = self.lam_x[sh:n]
            self.lam_x = lx
        keep = max(1, int((1.0 - DROP) * self.N) - k)        # the plan's nodes reused
        cut = k + keep                                       # the plan's node after the last
        join = plan["i0"] + cut - 1
        v0 = float(plan["X"][0, cut - 1]) if v_start is None else v_start
        Xt, Ut = initial(self.section(track, join), self.car, self.margin, v0)
        need = self.N - keep
        Xt, Ut = Xt[:, 1:need + 1].copy(), Ut[:, 1:need + 1]
        ease = np.clip(1.0 - np.arange(1, need + 1) / BLEND, 0.0, 1.0)
        for row in (3, 4):                                   # n, xi
            Xt[row] += ease * (plan["X"][row, cut - 1] - Xt[row, 0])
        plan_cut = {"X": plan["X"][:, :cut], "U": plan["U"][:, :cut]}
        return shift(plan_cut, k, Xt, Ut[:, :self.M - keep + 1])


def shift(plan, k, tail_X, tail_U):
    """The warm start for the window k nodes on: the plan's nodes and controls from k, then
    tail_X (6 x k) and tail_U (4 x k) for the stretch it did not cover."""
    X = np.hstack([plan["X"][:, k:], tail_X])
    U = np.hstack([plan["U"][:, k:], tail_U])
    return X, U[:, :X.shape[1] - 1]


def initial(track, car, margin=0.3, v_start=None):
    """A cold warm start for a window (no plan yet): the minimum-curvature line with the
    quasi-steady speed on it (ocp.py's), its controls made to match."""
    from race_planner.ocp import _warm_start
    X, _ = _warm_start(track, car, margin, v_start)
    return X, controls_for(X, track.s, car)
