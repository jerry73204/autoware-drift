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
from race_planner.track import TYRE_FIELDS

G = 9.81
REG_STEER, REG_ACCEL = 1e-2, 1e-4
END_SLIP, END_HEADING = 0.15, 0.35      # the window's last node: vy / vx, xi (rad)
DROP = 0.2                              # the share of a plan's end not reused as a warm start
NU = 7                                  # a stage's controls: rate a ax ay, the wheels' speeds
NZ = 14                                 # a stage's variables: x (6), u (7), edge slack (1)
KAPPA_MAX = 0.6                         # a tyre's slip ratio, either way: past the peak (0.26-
# 0.38) the force is flat and its slope near zero, where IPOPT, its iterate wandered there, found
# the slope rows locally infeasible (KAPPA_MAX 1.0: the first plan on Haruna failed)
FRONT_KAPPA_UP = 0.05                   # a front (braking only) wheel's, at most
SLOPE_MIN = 0.01                        # dFx/dkappa, in the tyre's stiffness, at least
WARM_KAPPA = 0.15                       # a cold guess's slips, at most (the peak is 0.26-0.38)
EDGE_PENALTY = 10.0                     # s per metre past the road's edge (less the margin)
CACHE = os.environ.get("RACE_PLANNER_CACHE", os.path.expanduser("~/.cache/race_planner"))
BLEND = 10                              # nodes over which a warm start's tail joins the plan
X_SCALE = np.array([20.0, 2.0, 1.0, 3.0, 0.3, 0.3])
U_SCALE = np.array([1.0, 10.0, 10.0, 10.0, 20.0, 20.0, 20.0])
TRACK_FIELDS = ("s", "x", "y", "z", "psi", "kappa", "w_left", "w_right", "mu", "grade",
                "sliding") + TYRE_FIELDS
NP = 8                                  # a stage's parameters: kappa mu grade sliding, the
# surface's tyre (stiff_long stiff_lat peak_long peak_lat, model.tyre's `surface`)
ROWS = ("ax", "ay", "fz_fl", "fz_fr", "fz_rl", "fz_rr", "use_fl", "use_fr", "use_rl", "use_rr",
        "drive", "power", "brake_front", "brake_rear", "sdot", "rear_balance", "kappa_rl",
        "kappa_rr", "front_balance_l", "front_balance_r", "kappa_fl", "kappa_fr", "slope_fl",
        "slope_fr", "slope_rl", "slope_rr")


NLP_FUNCTIONS = ("nlp", "nlp_f", "nlp_g", "nlp_grad_f", "nlp_jac_g", "nlp_grad", "nlp_hess_l")


def _compiled_nlpsol(nlp, opts, own=None, plugin="ipopt"):
    """IPOPT (or another of CasADi's solvers, `plugin`) on nlp with its functions C-compiled
    (those in `own`, by their names, where given, CasADi's own otherwise): generated once,
    compiled once per distinct code (cached by its hash under CACHE), loaded; None without a
    compiler."""
    own = own or {}
    probe = ca.nlpsol("l1", "ipopt", nlp, opts if plugin == "ipopt" else {})
    d = tempfile.mkdtemp(prefix="race_planner_")
    cg = ca.CodeGenerator("l1_nlp.c", {"with_header": False})
    for name in NLP_FUNCTIONS:
        cg.add(own[name] if name in own
               else probe.get_function(name) if name != "nlp" else probe.oracle())
    cg.generate(d + os.sep)
    src = os.path.join(d, "l1_nlp.c")
    key = hashlib.sha1(open(src, "rb").read()).hexdigest()[:16]
    try:
        return _load(src, key, opts, plugin)
    finally:
        shutil.rmtree(d, ignore_errors=True)


def _load(src, key, opts, plugin="ipopt"):
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
    return ca.nlpsol("l1", plugin, lib, opts)


def friction_scale(car, mu_map, mu_ratio, demand, grip=1.0):
    """The share of the map's friction mu_map the car has found where it is, from its own
    estimate mu_ratio (sim_vehicle_ecu's estimator over the tyre's design friction) and the
    acceleration the plan asks of it there (demand, in g).

    The estimator finds the road's friction only at the limit, as the acceleration the car
    reaches there (in g, plus est_margin); away from it it drifts back to its prior, the design
    friction (car.mu, 1.0: the tyre's own, while dry asphalt gives 1.12 g). Its test for the
    limit fires on dry asphalt too (turning less than a neutral bicycle would: every car does at
    speed): on Haruna it read 0.3-0.6 on straights at 1.3 m/s^2, and 0.55 pulling away, where a
    rear-driven car's limit is its rear axle's, half the car's. So the estimate is believed only
    where the plan asks for the road's grip (its demand at least LIMIT_SHARE of its grip times
    the map's friction: a corner's entry, apex and exit, not a straight's drive) and says the
    car could not do it (below the demand by more than the estimator's own margin). Then: the
    estimate in g against the map's friction."""
    est = mu_ratio * car.mu
    if (mu_ratio >= 1.0 or demand < LIMIT_SHARE * grip * mu_map
            or est >= demand - EST_MARGIN):
        return 1.0
    return float(min(1.0, est / mu_map))


def friction_ahead(car, scale, s, speed):
    """The friction's scale along a window (s: its nodes' distances from the car, m): the car's
    scale at the car, back to 1 over the distance the car covers while the estimator itself
    recovers from it ((1 - scale) / est_mu_recover_rate seconds at the car's speed): the estimate
    is trusted as far as the estimator would keep it, no further (a patch of low friction is
    known only where the car has met it; the road beyond it is the map's)."""
    if scale >= 1.0:
        return np.ones_like(np.asarray(s, float))
    reach = max(speed, 1.0) * (1.0 - scale) / car.est_mu_recover_rate
    return scale + (1.0 - scale) * np.clip(np.asarray(s, float) / reach, 0.0, 1.0)


EST_MARGIN = 0.05      # g: sim_vehicle_ecu's est_mu_margin, what its estimate adds at the limit
LIMIT_SHARE = 0.7      # a plan asking this much of its grip there is near the road's limit


def node_functions(car, tyres=False):
    """f(x, u, p) -> (dx/ds, dt/ds); g(x, u, p) -> path constraints within (lbg, ubg).

    u = (delta's rate, a, ax, ay), and with `tyres` the wheels' peripheral speeds too (the rear
    axle's, the front left's, the front right's): every tyre is then PhysX's own on its slip
    (axle.py), with six more rows: the two rear tyres make the ECU's rear force and each front
    tyre its brake share (in the car's weight), and each wheel's slip ratio within KAPPA_MAX
    (past it PhysX's tyre is far into its slide; a plan has no business there; a front wheel,
    which only brakes, not above FRONT_KAPPA_UP).

    p = (kappa, mu, grade, sliding, then the surface's tyre: model.tyre's four shares)."""
    nu = NU if tyres else 4
    x, u, p = ca.SX.sym("x", 6), ca.SX.sym("u", nu), ca.SX.sym("p", NP)
    vx, vy, r, n, xi, delta = (x[i] for i in range(6))
    kappa, mu, grade = p[0], p[1], p[2]
    o = axle.dynamics(car, [vx, vy, r], [delta, u[1]], mu, [u[2], u[3]], p[3],
                      w_rear=u[4] if tyres else None, w_front=(u[5], u[6]) if tyres else None,
                      surface=p[4:8])
    sin_t = grade / ca.sqrt(1 + grade * grade)
    dvx = o["dx"][0] - G * sin_t * ca.cos(xi)
    dvy = o["dx"][1] + G * sin_t * ca.sin(xi)
    sdot = (vx * ca.cos(xi) - vy * ca.sin(xi)) / (1 - n * kappa)
    dt = ca.vertcat(dvx, dvy, o["dx"][2], vx * ca.sin(xi) + vy * ca.cos(xi),
                    r - kappa * sdot, u[0])
    f = ca.Function("f", [x, u, p], [dt / sdot, 1 / sdot])
    R = car.r_wheel
    # each row in units of its own limit, so the solver sees numbers near one (loads in the car's
    # weight, torques in their limits, power in the motor's)
    mg = car.m * G
    t_drive, t_brake = car.axle_torque_max / R, car.brake_torque_max
    g = ca.vertcat(o["acc"][0] - u[2], o["acc"][1] - u[3],
                   *[fz / mg for fz in o["fz"]], *o["use"],
                   o["drive"] / t_drive, o["drive"] * vx / car.power_max,
                   0.5 * o["front_brake"] * R / t_brake,        # each front wheel's brake torque
                   (o["rear_brake"] - o["regen"]) * R / t_brake,  # the rear pair's friction
                   sdot)
    lbg = [0, 0, 0, 0, 0, 0, -1, -1, -1, -1, 0, -ca.inf, -ca.inf, -ca.inf, 0.5]
    ubg = [0, 0, ca.inf, ca.inf, ca.inf, ca.inf, 1, 1, 1, 1, 1, 1, 1, 2, ca.inf]
    if tyres:
        g = ca.vertcat(g, o["rear_balance"] / mg, *o["kappa"],
                       *[b / mg for b in o["front_balance"]], *o["kappa_front"])
        lbg += [0, -KAPPA_MAX, -KAPPA_MAX, 0, 0, -KAPPA_MAX, -KAPPA_MAX]
        ubg += [0, KAPPA_MAX, KAPPA_MAX, 0, 0, FRONT_KAPPA_UP, FRONT_KAPPA_UP]
        # every wheel on its tyre's rising branch (axle._slope): with its force held by a
        # balance, a slip past the peak gives the same force as one before it, the cost on a
        # straight cannot tell them apart and the plans sat at slips of 0.4-1, which a wheel
        # with no traction control cannot hold (it spins up). In units of the tyre's
        # longitudinal stiffness: at least SLOPE_MIN of it (0.1 % of the force short of the
        # peak). The KAPPA_MAX rows stay as a backstop.
        c_lon = car.long_stiff_value * G
        g = ca.vertcat(g, *[s / c_lon for s in o["slope"]])
        lbg += [SLOPE_MIN] * 4
        ubg += [ca.inf] * 4
    out = ca.Function("out", [x, u, p], [ca.vertcat(*o["fx"]), ca.vertcat(*o["fy"]),
                                         ca.vertcat(*o["fz"]), ca.vertcat(*o["use"]),
                                         ca.vertcat(*o["alpha"])])
    return f, ca.Function("g", [x, u, p], [g]), lbg, ubg, out


FLAT = 0.95     # a surface's sliding share this high: no fall-off past the peak (schedule's "flat")
UNBOUND = 1e3   # the slip rows' multiple where a surface falls off (no bound in effect)
MU_LOW = 0.45   # a surface's friction this low (schedule's mu_fallback) and its sliding share
SLIDING_LOW = 0.75   # this low (ice's 0.7; packed snow's 0.8 is not): rear_slip_low applies
FREE_STAGES = 3   # the live loop's first stages free of the slip and held rows (12 m at ds 4)


def slip_rows(car, g, k_front, k_rear=None, k_rear_low=None, mu_low=0.0):
    """g(x, u, p) with a row per bounded axle: its slip angle (tan, as model.tyre takes it; the
    front at the bicycle's steering) over k times the surface's lateral peak (model.peaks, the
    surface's share p[7]), within [-1, 1]; only on a surface without fall-off past its peak
    (sliding share p[3] >= FLAT), where the tyre's force does not fall however deep it slides,
    so nothing but this keeps the plan out of a slide the car cannot be held in (docs/issues/
    002). Elsewhere the model's own fall-off does (the rows are there, UNBOUND wide), except
    k_rear_low: the rear's bound on a surface that falls off deepest (sliding share SLIDING_LOW
    or less) and whose planned friction p[1] is mu_low or less: ice (issue 003); None for none."""
    x, u, p = ca.SX.sym("x", 6), ca.SX.sym("u", NU), ca.SX.sym("p", NP)
    vx, vy, r, delta = x[0], x[1], x[2], x[5]
    tp = model.peaks(car)[1] * p[7]
    flat = p[3] >= FLAT
    rows = [g(x, u, p)]
    if k_front is not None:
        uy = vy + car.lf * r
        c, s = ca.cos(delta), ca.sin(delta)
        lx, ly = c * vx + s * uy, -s * vx + c * uy
        tan_a = ly / (ca.sqrt(lx * lx + 1e-4) + model.SLIP_LAT_OFFSET)
        rows.append(tan_a / (ca.if_else(flat, k_front, UNBOUND) * tp))
    if k_rear is not None or k_rear_low is not None:
        tan_a = (vy - car.lr * r) / (ca.sqrt(vx * vx + 1e-4) + model.SLIP_LAT_OFFSET)
        k_flat = UNBOUND if k_rear is None else k_rear
        k_fall = (UNBOUND if k_rear_low is None
                  else ca.if_else(ca.logic_and(p[1] <= mu_low, p[3] <= SLIDING_LOW),
                                  k_rear_low, UNBOUND))
        rows.append(tan_a / (ca.if_else(flat, k_flat, k_fall) * tp))
    return ca.Function("g", [x, u, p], [ca.vertcat(*rows)])


def rk4(f, x, u, p, h):
    k1, t1 = f(x, u, p)
    k2, t2 = f(x + 0.5 * h * k1, u, p)
    k3, t3 = f(x + 0.5 * h * k2, u, p)
    k4, t4 = f(x + h * k3, u, p)
    return x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4), h / 6 * (t1 + 2 * t2 + 2 * t3 + t4)


def controls_for(X, s, car):
    """Controls consistent with states X (6 x n) along s: the steering rate from delta, the
    request from the speed's change (plus grade's share, left to the solver), the accelerations
    of the speed and the turn, and the rear axle's speed for the force the request puts there
    (the slip that force takes at the tyres' longitudinal stiffness). A warm start for a stretch
    the last plan did not cover."""
    vx, vy, r, n, xi, delta = X
    ds = np.gradient(s)
    dvds = np.gradient(vx) / ds
    a = dvds * vx
    return np.vstack([np.gradient(delta) / ds * vx, a, a, vx * r,
                      wheel_speeds(car, X, a)])[:, :-1]


def wheel_lx(car, X):
    """Each wheel's ground speed along its own heading (fl, fr, rl, rr; 4 x n) for states X
    (6 x n: vx vy r n xi delta), the front pair steered by Ackermann (axle.py's kinematics)."""
    vx, vy, r, delta = (np.asarray(X[i], float) for i in (0, 1, 2, 5))
    tn, L, tr = np.tan(delta), car.wheel_base, car.track
    steer = [np.arctan2(L * tn, L - 0.5 * tr * tn), np.arctan2(L * tn, L + 0.5 * tr * tn), 0, 0]
    out = []
    for (px, py), d in zip(model.wheels(car), steer):
        out.append(np.cos(d) * (vx - r * py) + np.sin(d) * (vy + r * px))
    return np.array(out)


def wheel_speeds(car, X, a):
    """The wheels' peripheral speeds (rear axle, front left, front right; 3 x n) for states X
    (6 x n) making the forces the request a asks of the ECU: each tyre's slip at its
    longitudinal stiffness from its own ground speed, within WARM_KAPPA: on the rising branch
    (a guess past the peak started IPOPT where it found the slope rows locally infeasible)."""
    lx = wheel_lx(car, X)
    vx = np.asarray(X[0], float)
    F = car.m * np.asarray(a, float) + car.drag * vx * vx + car.roll * car.m * G
    share = car.lr / car.wheel_base
    rear = np.where(F > 0, F, (1 - share) * F)
    front = np.minimum(F, 0.0) * share
    c = 2 * car.long_stiff_value * G                     # a pair's stiffness

    def speed(force, v):
        kap = np.clip(force / c, -WARM_KAPPA, WARM_KAPPA)
        return np.maximum(v + kap * (np.abs(v) + model.SLIP_LONG_OFFSET), 0.0)

    return np.vstack([speed(rear, 0.5 * (lx[2] + lx[3])), speed(front, lx[0]),
                      speed(front, lx[1])])


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
    worst, another local optimum). With the rear tyres' own slip (a fifth control, three more
    rows): 57 of 59, p50 0.21 s, p95 0.72 s, p99 1.1 s; the plan's sideslip on the gentle bend
    at s 90-160 within 0.3 deg of L0's (it was 6 deg off). With every tyre's own (seven
    controls, nine rows more than the ellipse): 58 of 59, p50 0.27 s, p95 0.91 s, p99 1.04 s."""

    def __init__(self, car, ds=4.0, length=200.0, grip=0.92, margin=0.3, max_iter=400,
                 tol=1e-3, budget=None, warm=True, compile=True, ipopt=None, beta_max=None,
                 rear_alpha_max=None, front_slip_max=None, rear_slip_max=None,
                 rear_slip_low=None, free_stages=0):
        """beta_max: the body's sideslip, |atan(vy / vx)|, at most this (rad) at every node, the
        last one's END_SLIP with it; rear_alpha_max: the rear axle's slip angle, |atan((vy - lr
        r) / vx)|, at most this. None (the live planner's) leaves either free. Each is two linear
        rows per stage, |vy - c r| <= tan(bound) vx: for a study of what sliding buys (a plan
        held to grip driving against the free one, docs/drift-optimality.md), not a limit of
        the car.

        front_slip_max, rear_slip_max: the front (the bicycle's front wheel) and the rear axle's
        slip angle, model.tyre's tan alpha, at most this many times the surface's lateral peak
        (model.peaks times the surface's peak share) on a surface without fall-off past the
        peak (slip_rows), a row per stage each. A limit of control, not of the tyre: past it
        the tyre is saturated and on such a surface its force does not say so, the steering no
        longer turns the car (roadmap 061 M4b, docs/issues/002); None leaves it free.
        rear_slip_low: the rear's bound on a slippery surface that falls off (issue 003).
        free_stages: the first stages free of these rows and the held rows (the live loop's
        FREE_STAGES: a window starts at the car's state, which may be past a bound); 0, every
        stage bound (a study's)."""
        self.car, self.ds, self.grip, self.margin = car, ds, grip, margin
        self.beta_max, self.rear_alpha_max = beta_max, rear_alpha_max
        self.front_slip_max, self.rear_slip_max = front_slip_max, rear_slip_max
        held = [(c, math.tan(b)) for c, b in ((0.0, beta_max), (car.lr, rear_alpha_max))
                if b is not None]
        nb = 2 * len(held)
        end_slip = END_SLIP if beta_max is None else min(END_SLIP, math.tan(beta_max))
        self.N = N = int(round(length / ds)) + 1
        self.M = M = N - 1
        self.f, self.g, lbg, ubg, self.out = node_functions(car, tyres=True)
        self.rear_slip_low = rear_slip_low
        nk = (front_slip_max is not None) + (rear_slip_max is not None or
                                              rear_slip_low is not None)
        if nk:
            self.g = slip_rows(car, self.g, front_slip_max, rear_slip_max, rear_slip_low,
                               grip * MU_LOW)
            lbg, ubg = list(lbg) + [-1.0] * nk, list(ubg) + [1.0] * nk
        self.lbg, self.ubg = np.array(lbg, float), np.array(ubg, float)
        ng = len(lbg)
        # one stage in the scaled variables
        xs, us, p = ca.SX.sym("xs", 6), ca.SX.sym("us", NU), ca.SX.sym("p", NP)
        x, u = xs * X_SCALE, us * U_SCALE
        xn, dt = rk4(self.f, x, u, p, ds)
        stage = ca.Function("l1_stage", [xs, us, p], [xn / X_SCALE, dt, self.g(x, u, p)])
        # the decision vector stage by stage (x_k, u_k), then x_N
        # (x_k, u_k, e_k: how far node k is past the road's edge less the margin)
        W = ca.MX.sym("w", NZ * M + 6)
        Pw = ca.MX.sym("p", NP + 2, M)           # the stage's (NP), the free width l, r
        Pp = Pw[:NP, :]
        Z = ca.reshape(W[:NZ * M], NZ, M)
        Xk, Uk, Ek, xN = Z[:6, :], Z[6:6 + NU, :], Z[6 + NU, :], W[NZ * M:]
        xn_s, dts, G = stage.map(M)(Xk, Uk, Pp)
        n = Xk[3, :] * X_SCALE[3]
        sx = X_SCALE
        rows = ca.vertcat(xn_s - ca.horzcat(Xk[:, 1:], xN), G,     # each stage's, together
                          n + Ek + Pw[NP + 1, :], n - Ek - Pw[NP, :])
        for c, tb in held:                       # vy - c r - tb vx <= 0 <= vy - c r + tb vx
            v = sx[1] * Xk[1, :] - c * sx[2] * Xk[2, :]
            rows = ca.vertcat(rows, v - tb * sx[0] * Xk[0, :], v + tb * sx[0] * Xk[0, :])
        end = ca.vertcat(sx[1] * xN[1] + end_slip * sx[0] * xN[0],
                         sx[1] * xN[1] - end_slip * sx[0] * xN[0])
        a = Uk[1, :] * U_SCALE[1]
        T = (ca.sum2(dts) + REG_STEER * ds * ca.sumsqr(Uk[0, :] * U_SCALE[0])
             + REG_ACCEL * ca.sumsqr(a[1:] - a[:-1]) + EDGE_PENALTY * ca.sum2(Ek))
        blo, bhi = [-np.inf, 0.0] * len(held), [0.0, np.inf] * len(held)
        self.lbg_all = np.concatenate([np.tile(np.r_[np.zeros(6), self.lbg, 0.0, -np.inf, blo], M),
                                       [0.0, -np.inf]])
        self.ubg_all = np.concatenate([np.tile(np.r_[np.zeros(6), self.ubg, np.inf, 0.0, bhi], M),
                                       [np.inf, 0.0]])
        per = 6 + ng + 2 + nb
        # the slip rows and the held rows not on the first FREE_STAGES stages: their nodes are the
        # car's state and the next, which may already be past a bound (a car sliding past its
        # peak made every held window infeasible from there on, H the fallback with it: issue
        # 003); the plan must come back within the bound, not start there
        for k in range(min(free_stages, M)):
            lo = k * per
            for i in list(range(lo + 6 + ng - nk, lo + 6 + ng)) + list(range(lo + 6 + ng + 2,
                                                                              lo + per)):
                self.lbg_all[i], self.ubg_all[i] = -np.inf, np.inf
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
        zs, ps, lf, lm = ca.SX.sym("z", NZ), ca.SX.sym("p", NP), ca.SX.sym("lf"), ca.SX.sym("l", 6 + ng)
        o = stage(zs[:6], zs[6:6 + NU], ps)
        Hk, _ = ca.hessian(lf * o[1] + ca.dot(lm[:6], o[0]) + ca.dot(lm[6:], o[2]), zs)
        hs = ca.Function("l1_stage_hess", [zs, ps, lf, lm], [Hk])
        Hm = hs.map(M)(Z, Pp, lam_f, ca.reshape(lam_g[:per * M], per, M)[:6 + ng, :])
        reg = T - ca.sum2(dts)                   # the regularisation's, constant
        Hreg = ca.sparsify(ca.Function("hreg", [W], [ca.hessian(reg, W)[0]])(np.zeros(NZ * M + 6)))
        H = ca.diagcat(*ca.horzsplit(Hm, NZ), ca.MX(6, 6)) + lam_f * Hreg
        hess = ca.Function("nlp_hess_l", [W, ca.vec(Pw), lam_f, lam_g], [ca.triu(H)],
                           ["x", "p", "lam_f", "lam_g"], ["triu_hess_gamma_x_x"])
        # the constraints' Jacobian and the time's gradient by stages too (CasADi's own: 0.6 and
        # 0.3 ms an iteration): each stage's block, the rest constant
        js = ca.Function("l1_stage_jac", [zs, ps], [ca.jacobian(ca.vertcat(o[0], o[2]), zs)])
        gs = ca.Function("l1_stage_grad", [zs, ps], [ca.gradient(o[1], zs)])
        Jm, gm = js.map(M)(Z, Pp), gs.map(M)(Z, Pp)
        lin = ca.DM.zeros(2 + nb, NZ)             # the edge rows' (n, e), the slips' (vx vy r)
        lin[0, 3], lin[0, 6 + NU] = X_SCALE[3], 1.0
        lin[1, 3], lin[1, 6 + NU] = X_SCALE[3], -1.0
        for j, (c, tb) in enumerate(held):
            for i, sign in ((2 + 2 * j, -1.0), (3 + 2 * j, 1.0)):
                lin[i, 0], lin[i, 1], lin[i, 2] = sign * tb * sx[0], sx[1], -c * sx[2]
        blocks = [ca.vertcat(b, lin) for b in ca.horzsplit(Jm, NZ)]
        nw = NZ * M + 6
        couple = ca.DM.zeros(per * M + 2, nw)     # x_k+1 in stage k's dynamics; the end rows
        for k in range(M):
            couple[k * per:k * per + 6, (k + 1) * NZ:(k + 1) * NZ + 6] = -ca.DM.eye(6)
        couple[per * M, NZ * M:] = ca.horzcat(end_slip * sx[0], sx[1], 0, 0, 0, 0)
        couple[per * M + 1, NZ * M:] = ca.horzcat(-end_slip * sx[0], sx[1], 0, 0, 0, 0)
        couple = ca.DM(ca.sparsify(couple))
        Jg = ca.vertcat(ca.horzcat(ca.diagcat(*blocks), ca.MX(per * M, 6)), ca.MX(2, nw)) + couple
        gvec = ca.vertcat(ca.vec(rows), end)
        jac = ca.Function("nlp_jac_g", [W, ca.vec(Pw)], [gvec, Jg], ["x", "p"], ["g", "jac_g_x"])
        cvec = ca.DM.zeros(nw)
        cvec[[k * NZ + 6 + NU for k in range(M)]] = EDGE_PENALTY
        greg = ca.mtimes(Hreg, W) + cvec          # the regularisation is quadratic (no linear
        # term but the edge's: its own gradient at zero)
        grad = ca.Function("nlp_grad_f", [W, ca.vec(Pw)],
                           [T, ca.vertcat(ca.vec(gm), ca.DM.zeros(6)) + greg],
                           ["x", "p"], ["f", "grad_f_x"])
        own = {"nlp_hess_l": hess, "nlp_jac_g": jac, "nlp_grad_f": grad}
        self.nlp, self.own = nlp, own            # the problem, for another solver's study
        self.solver = _compiled_nlpsol(nlp, opts, own) if compile else None
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
        ulo = np.tile([-rate, -inf, -inf, -inf, 0.0, 0.0, 0.0], (M, 1)).T
        uhi = np.tile([rate, inf, inf, inf, car.wheel_speed_max or inf, inf, inf], (M, 1)).T
        return self.pack(lo, ulo, 0.0), self.pack(hi, uhi, inf)

    def pack(self, X, U, e=0.0):
        """The scaled decision vector from X (6 x N), U (7 x M) and the edge slack e."""
        Z = np.vstack([X[:, :self.M] / X_SCALE[:, None], U / U_SCALE[:, None],
                       np.full(self.M, e)])
        return np.concatenate([Z.ravel(order="F"), X[:, self.M] / X_SCALE])

    def unpack(self, w):
        M = self.M
        Z = np.asarray(w).ravel()[:NZ * M].reshape(NZ, M, order="F")
        X = np.hstack([Z[:6] * X_SCALE[:, None], (np.asarray(w).ravel()[NZ * M:] * X_SCALE)[:, None]])
        return X, Z[6:6 + NU] * U_SCALE[:, None]

    def params(self, track, i0, mu_scale=None):
        """The window's track parameters from node i0 of `track` (on this window's grid), the
        friction times mu_scale (a number, or one per node; see friction_ahead)."""
        sl = slice(i0, i0 + self.N)
        mu = track.mu[sl] * (1.0 if mu_scale is None else np.asarray(mu_scale)[:len(track.mu[sl])])
        P = np.vstack([track.kappa[sl], self.grip * mu, track.grade[sl], track.sliding[sl],
                       *(getattr(track, k)[sl] for k in TYRE_FIELDS)])[:, :self.M]
        return P, track.w_left[sl], track.w_right[sl], track.s[sl]

    def solve(self, track, i0, x0, v_end, X_init, U_init, mu_scale=None):
        """The plan from node i0 starting at state x0, warm-started from X_init (6 x N) and
        U_init (7 x M: rate, a, ax, ay, the wheels' speeds; fewer rows take the missing speeds
        from the request). mu_scale: the friction's scale over the window (friction_ahead).
        Returns a dict: X, U, s, t (per node), time, ok, status, iters, solve_s, defect (the
        largest shooting gap)."""
        P, wl, wr, s = self.params(track, i0, mu_scale)
        if U_init.shape[0] < NU:                 # a warm start without (some of) the wheels'
            w = wheel_speeds(self.car, X_init[:, :self.M], U_init[1])
            U_init = np.vstack([U_init[:4], w[:3] if U_init.shape[0] == 4 else
                                np.vstack([U_init[4:5], w[1:]])])
        # the first interval's wheels at the slips the warm start gave them, from the state the
        # window starts at (the car's, not the last plan's there: off by a slide's vy, a front
        # wheel's speed is off by metres a second, the slips by far past the peak)
        U_init = np.array(U_init, float)
        X0 = np.array(X_init[:, :1], float)
        lx_old = wheel_lx(self.car, X0)[:, 0]
        X0[:, 0] = x0
        lx_new = wheel_lx(self.car, X0)[:, 0]
        for row, old, new in ((4, 0.5 * (lx_old[2] + lx_old[3]), 0.5 * (lx_new[2] + lx_new[3])),
                              (5, lx_old[0], lx_new[0]), (6, lx_old[1], lx_new[1])):
            off = model.SLIP_LONG_OFFSET
            kap = (U_init[row, 0] - old) / (abs(old) + off)
            U_init[row, 0] = max(new + kap * (abs(new) + off), 0.0)
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
    tail_X (6 x k) and tail_U (7 x k) for the stretch it did not cover."""
    X = np.hstack([plan["X"][:, k:], tail_X])
    U = np.hstack([plan["U"][:, k:], tail_U])
    return X, U[:, :X.shape[1] - 1]


def initial(track, car, margin=0.3, v_start=None):
    """A cold warm start for a window (no plan yet): the minimum-curvature line with the
    quasi-steady speed on it (ocp.py's), its controls made to match."""
    from race_planner.ocp import _warm_start
    X, _ = _warm_start(track, car, margin, v_start)
    return X, controls_for(X, track.s, car)
