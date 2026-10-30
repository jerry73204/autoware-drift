"""The car's steady states on the two-track model: the body's velocity, the steering and the wheel
speeds that hold a given speed and yaw rate (or steering) with no acceleration along the path.
The model's checks against Isaac use them (a constant-radius ramp, a steady step), and so will the
speed limits on a line (the most speed a radius allows, with the sideslip free).

A steady state is a root of the model: dvx = dvy = dr = 0 in the body frame, the front wheels
free-rolling (no torque), the rear axle's differential holding, the load transfer consistent with
the accelerations. Solved with IPOPT; `maximise_speed` turns it into the largest speed on a radius.
"""
import math

import casadi as ca

from race_planner import model


def steady_state(car, mu, speed=None, radius=None, delta=None, maximise_speed=False,
                 sliding=1.0, surface=None):
    """A steady state on `car` with friction `mu` (and its share past the peak, `sliding`; the
    surface's tyre, model.tyre's `surface`, None the rig's).
    Give two of speed (m/s, the CoM's), radius (m, > 0 left) and delta (the bicycle steering
    angle, rad), or a radius and maximise_speed.

    Returns a dict: ok, speed, radius, delta, vx, vy, r, beta (rad, at the CoM), ay, w (the four
    wheels' peripheral speeds), torque, kappa, alpha (tan), fz.
    """
    z = ca.SX.sym("z", 9)               # vx vy r w_fl w_fr w_rl w_rr ax ay
    d = ca.SX.sym("delta") if delta is None else delta
    v = ca.SX.sym("speed") if speed is None else speed
    o = model.dynamics(car, z[0:3], [d, z[3], z[4], z[5], z[6]], mu, [z[7], z[8]], sliding,
                       surface)
    g = [*o["dx"], o["torque"][0], o["torque"][1], o["diff"],
         o["acc"][0] - z[7], o["acc"][1] - z[8], z[0] ** 2 + z[1] ** 2 - v ** 2]
    guess_v = 10.0 if speed is None else float(speed)
    guess_r = guess_v / radius if radius else 0.0
    x, x0 = [z], [guess_v, 0.0, guess_r] + [guess_v] * 4 + [0.0, guess_v * guess_r]
    lbx = [0.1, -60.0, -6.0, 0.0, 0.0, 0.0, 0.0, -40.0, -40.0]
    ubx = [90.0, 60.0, 6.0, 120.0, 120.0, 120.0, 120.0, 40.0, 40.0]
    if radius is not None:
        g.append(z[2] * radius - v)
    if delta is None:
        x.append(d)
        x0.append(math.atan(car.wheel_base / radius) if radius else 0.0)
        lbx.append(-car.steer_max)
        ubx.append(car.steer_max)
    if speed is None:
        x.append(v)
        x0.append(guess_v)
        lbx.append(0.5)
        ubx.append(90.0)
    X = ca.vertcat(*x)
    nlp = {"x": X, "f": -v if maximise_speed else 0, "g": ca.vertcat(*g)}
    solver = ca.nlpsol("steady", "ipopt", nlp, {"ipopt.print_level": 0, "ipopt.sb": "yes",
                                                 "print_time": False})
    sol = solver(x0=x0, lbx=lbx, ubx=ubx, lbg=0, ubg=0)
    vals = ca.Function("vals", [X], [z, d, v, ca.vertcat(*o["torque"]), ca.vertcat(*o["kappa"]),
                                     ca.vertcat(*o["alpha"]), ca.vertcat(*o["fz"])])(sol["x"])
    zz, dd, vv, T, k, a, fz = [ca.DM(q).full().ravel() for q in vals]
    vx, vy, r = zz[0], zz[1], zz[2]
    return {"ok": solver.stats()["success"], "speed": float(vv[0]),
            "radius": float(vv[0] / r) if abs(r) > 1e-9 else math.inf, "delta": float(dd[0]),
            "vx": vx, "vy": vy, "r": r, "beta": math.atan2(vy, vx), "ay": float(zz[8]),
            "w": zz[3:7], "torque": T, "kappa": k, "alpha": a, "fz": fz}
