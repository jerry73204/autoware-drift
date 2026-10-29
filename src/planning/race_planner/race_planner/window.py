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
- the road's width less the car's half width and a margin; forward along the track.

Built once (CasADi Opti, IPOPT on the expanded graph); each solve sets the parameters and the
warm start. The warm start is the last plan shifted by the distance driven, its tail from the
long view (`tail`).
"""
import math
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
X_SCALE = np.array([20.0, 2.0, 1.0, 3.0, 0.3, 0.3])
U_SCALE = np.array([1.0, 10.0, 10.0, 10.0])
TRACK_FIELDS = ("s", "x", "y", "z", "psi", "kappa", "w_left", "w_right", "mu", "grade", "sliding")
ROWS = ("ax", "ay", "fz_fl", "fz_fr", "fz_rl", "fz_rr", "use_fl", "use_fr", "use_rl", "use_rr",
        "drive", "power", "brake_front", "brake_rear", "sdot")


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
    """The L1 problem for a window of `length` metres on a grid of `ds`, built once."""

    def __init__(self, car, ds=4.0, length=200.0, grip=0.92, margin=0.3, max_iter=400,
                 tol=1e-3, budget=None, warm=True):
        self.car, self.ds, self.grip, self.margin = car, ds, grip, margin
        self.N = N = int(round(length / ds)) + 1
        self.M = M = N - 1
        self.f, self.g, lbg, ubg, self.out = node_functions(car)
        opti = ca.Opti()
        self.Xs, self.Us = opti.variable(6, N), opti.variable(4, M)
        X = ca.diag(X_SCALE) @ self.Xs
        U = ca.diag(U_SCALE) @ self.Us
        self.x0, self.P = opti.parameter(6), opti.parameter(4, M)
        self.WL, self.WR, self.vend = opti.parameter(N), opti.parameter(N), opti.parameter()
        T = 0
        for k in range(M):
            xn, dt = rk4(self.f, X[:, k], U[:, k], self.P[:, k], ds)
            opti.subject_to(X[:, k + 1] == xn)
            T += dt + REG_STEER * U[0, k] ** 2 * ds
            gk = self.g(X[:, k], U[:, k], self.P[:, k])
            opti.subject_to(opti.bounded(np.array(lbg, float), gk, np.array(ubg, float)))
            if k + 1 < M:
                T += REG_ACCEL * (U[1, k + 1] - U[1, k]) ** 2
        half = 0.5 * car.width
        opti.subject_to(opti.bounded(-(self.WR - half - margin), X[3, :].T,
                                     self.WL - half - margin))
        opti.subject_to(opti.bounded(-car.steer_max, ca.vec(X[5, :]), car.steer_max))
        rate = car.steer_rate_max or STEER_RATE
        opti.subject_to(opti.bounded(-rate, ca.vec(U[0, :]), rate))
        opti.subject_to(opti.bounded(-math.pi / 2, ca.vec(X[4, :]), math.pi / 2))
        if car.wheel_speed_max is not None:
            opti.subject_to(ca.vec(X[0, :]) <= car.wheel_speed_max)
        opti.subject_to(X[:, 0] == self.x0)
        # the window's end: no faster than the long view allows, and running near straight
        # (a receding horizon's last nodes are otherwise free to do anything, since nothing
        # after them counts, and the next window inherits it)
        opti.subject_to(X[0, N - 1] <= self.vend)
        opti.subject_to(opti.bounded(-END_SLIP * X[0, N - 1], X[1, N - 1], END_SLIP * X[0, N - 1]))
        opti.subject_to(opti.bounded(-END_HEADING, X[4, N - 1], END_HEADING))
        opti.minimize(T)
        # warm-started in its multipliers too (shifted with the window), so the barrier starts
        # low: re-solves take 20-70 iterations instead of 40-400 (M1, 2026-10-29)
        # (cold: IPOPT's own start, for the first plan and after a failure)
        ipopt = {"max_iter": max_iter, "tol": tol, "acceptable_tol": 1e-3, "acceptable_iter": 5,
                 "print_level": 0, "sb": "yes"}
        if warm:
            ipopt.update({"mu_init": 1e-6, "warm_start_init_point": "yes",
                          "warm_start_bound_push": 1e-6, "warm_start_slack_bound_push": 1e-6,
                          "warm_start_mult_bound_push": 1e-6})
        if budget is not None:
            ipopt["max_wall_time"] = float(budget)
        opti.solver("ipopt", {"print_time": False, "expand": True}, ipopt)
        self.opti, self.X, self.U = opti, X, U
        self.lbg, self.ubg = np.array(lbg, float), np.array(ubg, float)
        self.per_stage = 6 + len(lbg)            # each interval's rows: its dynamics, its path
        self.lam = None                          # the last solve's multipliers

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
        o = self.opti
        o.set_value(self.x0, x0)
        o.set_value(self.P, P)
        o.set_value(self.WL, wl)
        o.set_value(self.WR, wr)
        o.set_value(self.vend, v_end)
        o.set_initial(self.Xs, X_init / X_SCALE[:, None])
        o.set_initial(self.Us, U_init / U_SCALE[:, None])
        o.set_initial(o.lam_g, self.lam if self.lam is not None else 0.0)
        c = time.time()
        try:
            sol = o.solve()
            get, ok = sol.value, True
        except RuntimeError:
            get, ok = o.debug.value, False
        dt = time.time() - c
        st = o.stats()
        Xv = np.asarray(get(self.X)).reshape(6, self.N)
        Uv = np.asarray(get(self.U)).reshape(4, self.M)
        self.lam = np.asarray(get(o.lam_g)).ravel()
        seg, gap, over = [], 0.0, 0.0
        for k in range(self.M):
            xn, tk = rk4(self.f, Xv[:, k], Uv[:, k], P[:, k], self.ds)
            seg.append(float(tk))
            gap = max(gap, float(np.max(np.abs(np.asarray(xn).ravel() - Xv[:, k + 1]))))
            gv = np.asarray(self.g(Xv[:, k], Uv[:, k], P[:, k])).ravel()
            over = max(over, float(np.max(np.maximum(gv - self.ubg, self.lbg - gv))))
        status = st["return_status"]
        # a solve stopped by its budget or its iterations is still a plan if it is feasible:
        # the car is better off with it than with the last one (re-solved next second anyway)
        feasible = gap < 1e-2 and over < 1e-2
        ok = ok or status == "Solved_To_Acceptable_Level" or feasible
        return {"X": Xv, "U": Uv, "s": s, "t": np.concatenate([[0.0], np.cumsum(seg)]),
                "time": float(np.sum(seg)), "ok": ok, "status": status,
                "iters": st["iter_count"], "solve_s": dt, "defect": gap, "violation": over,
                "i0": i0}

    def section(self, track, i0):
        """The window's stretch of `track` (on this window's grid) from node i0."""
        sl = slice(i0, i0 + self.N)
        return replace(track, **{k: getattr(track, k)[sl] for k in TRACK_FIELDS})

    def warm(self, plan, k, track, v_start=None):
        """The warm start for re-solving k nodes on from `plan`: its nodes from k up to the last
        DROP of it, then the cold guess of the new window (the minimum-curvature line with the
        quasi-steady speed: it follows the road, which repeating the plan's last state does not,
        and the plan's own end is a receding horizon's, the least to be trusted)."""
        Xc, Uc = initial(self.section(track, plan["i0"] + k), self.car, self.margin, v_start)
        if self.lam is not None:                             # the multipliers, shifted too
            n, sh = self.M * self.per_stage, k * self.per_stage
            lam = self.lam.copy()
            lam[:n - sh] = self.lam[sh:n]
            self.lam = lam
        keep = max(1, int((1.0 - DROP) * self.N) - k)        # the plan's nodes reused
        cut = k + keep
        plan_cut = {"X": plan["X"][:, :cut], "U": plan["U"][:, :cut]}
        return shift(plan_cut, k, Xc[:, keep:], np.hstack([Uc[:, keep:], Uc[:, -1:]])[:, :self.M - keep + 1])


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
