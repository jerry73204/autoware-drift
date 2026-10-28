"""The minimum-time trajectory: the two-track model (model.py) driven along the track as fast as
its tyres, motor, brakes and the road's width allow, with nothing told where to brake, where to
turn in, which line to take or whether to slide. Sideslip is free: a drift comes out only where
it is faster.

Spatial formulation (Perantoni and Limebeer 2014, TUM's global_racetrajectory_optimization's
mintime): the independent variable is s along the track's reference line; the car is at a
lateral offset n (left > 0) with its heading xi off the line's. Time is the objective,
the integral of 1 / s_dot ds, s_dot = (vx cos xi - vy sin xi) / (1 - n kappa).

States (per node): vx, vy, r (the body's velocity), n, xi, delta (the bicycle steering angle).
Controls (per interval, held): delta's rate, the four wheels' peripheral speeds (model.py: a
wheel's slip is set by its speed, its torque is what its tyre asks), and the accelerations the
load transfer is taken at (equal to the model's, a constraint).

Constraints at each node: the road's width less the car's half width and a margin; the front
wheels only brake; every wheel's brake torque within the spec's; the rear axle's drive within the
motor's torque through its gear and its power; the differential's (locked: one speed, open: one
torque); no wheel lifts; forward along the track. Grade adds gravity along the line.

Multiple shooting, one RK4 step per interval, IPOPT. The warm start is the minimum-curvature line
with the quasi-steady speed on it. A small penalty on the steering rate and on the wheels' speed
changes keeps the controls from chattering where the time does not care.

The only settings are the car's (STEER_RATE: a fast rack, a drift car's) and the solver's; none
is per track.
"""
import math

import casadi as ca
import numpy as np

from race_planner import model
from race_planner.line import min_curvature_line
from race_planner.qss import speed_profile

G = 9.81
STEER_RATE = 2.0             # rad/s at the bicycle angle
REG_STEER, REG_WHEEL = 1e-3, 1e-4
X_SCALE = np.array([20.0, 2.0, 1.0, 3.0, 0.3, 0.3])
U_SCALE = np.array([1.0, 20.0, 20.0, 20.0, 20.0, 10.0, 10.0])


def _node_functions(car):
    """f(x, u, p) -> (dx/ds, dt/ds); g(x, u, p) -> path constraints (see bounds)."""
    x, u, p = ca.SX.sym("x", 6), ca.SX.sym("u", 7), ca.SX.sym("p", 4)   # kappa mu grade sliding
    vx, vy, r, n, xi, delta = (x[i] for i in range(6))
    kappa, mu, grade = p[0], p[1], p[2]
    o = model.dynamics(car, [vx, vy, r], [delta, u[1], u[2], u[3], u[4]], mu, [u[5], u[6]],
                       p[3])
    sin_t = grade / ca.sqrt(1 + grade * grade)
    dvx = o["dx"][0] - G * sin_t * ca.cos(xi)
    dvy = o["dx"][1] + G * sin_t * ca.sin(xi)
    sdot = (vx * ca.cos(xi) - vy * ca.sin(xi)) / (1 - n * kappa)
    dt = ca.vertcat(dvx, dvy, o["dx"][2], vx * ca.sin(xi) + vy * ca.cos(xi),
                    r - kappa * sdot, u[0])
    f = ca.Function("f", [x, u, p], [dt / sdot, 1 / sdot])
    T = o["torque"]
    axle = T[2] + T[3]
    omega = 0.5 * (u[3] + u[4]) / car.r_wheel
    g = ca.vertcat(o["acc"][0] - u[5], o["acc"][1] - u[6], o["diff"],      # = 0
                   T[0], T[1],                                              # [-Tb, 0]
                   T[2], T[3],                                              # >= -Tb
                   axle, axle * omega,                                      # <= motor, power
                   *o["fz"], sdot)                                          # >= 0, > 0.5
    lbg = [0, 0, 0, -car.brake_torque_max, -car.brake_torque_max,
           -car.brake_torque_max, -car.brake_torque_max, -2 * car.brake_torque_max, -ca.inf,
           0, 0, 0, 0, 0.5]
    ubg = [0, 0, 0, 0, 0, ca.inf, ca.inf, car.axle_torque_max, car.power_max,
           ca.inf, ca.inf, ca.inf, ca.inf, ca.inf]
    out = ca.Function("out", [x, u, p], [ca.vertcat(*T), ca.vertcat(*o["fz"]),
                                         ca.vertcat(*o["kappa"]), ca.vertcat(*o["alpha"])])
    return f, ca.Function("g", [x, u, p], [g]), lbg, ubg, out


def _rk4(f, x, u, p, h):
    k1, t1 = f(x, u, p)
    k2, t2 = f(x + 0.5 * h * k1, u, p)
    k3, t3 = f(x + 0.5 * h * k2, u, p)
    k4, t4 = f(x + h * k3, u, p)
    return x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4), h / 6 * (t1 + 2 * t2 + 2 * t3 + t4)


def _warm_start(track, car, margin, v_start):
    line, a = min_curvature_line(track, 0.5 * car.width, margin)
    v = 0.95 * speed_profile(line, car, v_start=v_start)
    v = np.maximum(v, 1.0)
    xi = np.unwrap(line.psi) - np.unwrap(track.psi)
    xi -= 2 * math.pi * np.round(xi / (2 * math.pi))
    delta = np.clip(np.arctan(car.wheel_base * line.kappa), -car.steer_max, car.steer_max)
    X = np.vstack([v, 0 * v, v * line.kappa, a, xi, delta])
    ds = np.diff(line.s)
    dv = np.gradient(v, line.s) * v
    U = np.vstack([np.zeros_like(v), v, v, v, v, dv, v * v * line.kappa])
    return X, U


def solve_min_time(track, car, v_start=None, margin=0.3, max_iter=3000, verbose=False):
    """The minimum-time trajectory along `track` for `car`. Open tracks start at v_start (m/s,
    None: free) on the reference line; closed ones are periodic. Returns a dict of arrays per
    node (s, n, xi, vx, vy, r, delta, beta, t, x, y; torque, fz, kappa, alpha, w per node of
    the controls) and ok, status, time."""
    f, g, lbg, ubg, out = _node_functions(car)
    closed = track.closed
    N = len(track.s)                      # nodes; intervals: N - 1 open, N closed
    s = track.s
    if closed:
        gap = math.hypot(track.x[0] - track.x[-1], track.y[0] - track.y[-1])
        h = np.append(np.diff(s), gap)
    else:
        h = np.diff(s)
    M = len(h)
    P = np.vstack([track.kappa, track.mu, track.grade, track.sliding])
    X0, U0 = _warm_start(track, car, margin, v_start)

    opti = ca.Opti()
    Xs, Us = opti.variable(6, N), opti.variable(7, M)
    X = ca.diag(X_SCALE) @ Xs
    U = ca.diag(U_SCALE) @ Us
    time = 0
    half = 0.5 * car.width
    for k in range(M):
        j = (k + 1) % N
        xn, dt = _rk4(f, X[:, k], U[:, k], P[:, k], h[k])
        opti.subject_to(X[:, j] == xn)
        time += dt
        opti.subject_to(opti.bounded(np.array(lbg, float), g(X[:, k], U[:, k], P[:, k]),
                                     np.array(ubg, float)))
        time += REG_STEER * U[0, k] ** 2 * h[k]
        kn = (k + 1) % M
        if closed or k + 1 < M:
            time += REG_WHEEL * ca.sumsqr(U[1:5, kn] - U[1:5, k])
    for k in range(N):
        opti.subject_to(opti.bounded(-(track.w_right[k] - half - margin), X[3, k],
                                     track.w_left[k] - half - margin))
    opti.subject_to(opti.bounded(-car.steer_max, ca.vec(X[5, :]), car.steer_max))
    opti.subject_to(opti.bounded(-STEER_RATE, ca.vec(U[0, :]), STEER_RATE))
    opti.subject_to(opti.bounded(-math.pi / 2, ca.vec(X[4, :]), math.pi / 2))
    opti.subject_to(ca.vec(U[1:5, :]) >= 0)
    if not closed:
        opti.subject_to(X[3, 0] == 0)
        opti.subject_to(X[4, 0] == 0)
        opti.subject_to(X[1, 0] == 0)
        if v_start is not None:
            opti.subject_to(X[0, 0] == v_start)
    opti.minimize(time)
    opti.set_initial(Xs, X0 / X_SCALE[:, None])
    opti.set_initial(Us, U0[:, :M] / U_SCALE[:, None])
    opti.solver("ipopt", {"print_time": False},
                {"max_iter": max_iter, "tol": 1e-6, "print_level": 5 if verbose else 0,
                 "sb": "yes", "mu_strategy": "adaptive"})
    try:
        sol = opti.solve()
        ok, get = True, sol.value
    except RuntimeError:
        ok, get = False, opti.debug.value
    status = opti.stats()["return_status"]
    Xv = np.asarray(get(X)).reshape(6, N)
    Uv = np.asarray(get(U)).reshape(7, M)
    vx, vy, r, n, xi, delta = Xv
    sdot = (vx * np.cos(xi) - vy * np.sin(xi)) / (1 - n * track.kappa)
    seg = np.array([float(_rk4(f, Xv[:, k], Uv[:, k], P[:, k], h[k])[1]) for k in range(M)])
    t = np.concatenate([[0.0], np.cumsum(seg)])[:N]
    outs = [out(Xv[:, k], Uv[:, k], P[:, k]) for k in range(M)]
    torque, fz, kap, alp = (np.array([np.asarray(o[i]).ravel() for o in outs]) for i in range(4))
    nx, ny = -np.sin(track.psi), np.cos(track.psi)
    return {"ok": ok, "status": status, "time": float(np.sum(seg)), "s": s, "t": t,
            "n": n, "xi": xi, "vx": vx, "vy": vy, "r": r, "delta": delta,
            "beta": np.arctan2(vy, vx), "sdot": sdot,
            "x": track.x + n * nx, "y": track.y + n * ny,
            "w": Uv[1:5].T, "torque": torque, "fz": fz, "kappa": kap, "alpha": alp}
