"""The quasi-steady-state speed profile on a given line: the fastest speed at each point that the
car's grip, drive, brakes, drag and the road's grade allow, by a forward (accelerate) and a
backward (brake) pass. It is the minimum-time optimiser's warm start, and its braking points
are a first answer to where to brake; it knows nothing of sideslip or of the line's choice.

The car is a point mass on a friction circle of radius mu g cos(theta) (theta the road's
pitch, tan(theta) = grade), with:

- cornering: a_y = v^2 kappa; the corner speed cap is sqrt(mu g cos(theta) / |kappa|);
- braking on all four wheels: what the circle leaves of mu g cos(theta) after a_y;
- drive on the driven axle only: its normal load with the longitudinal load transfer
  (rear: m (g cos(theta) lr + a h) / L), the circle left after that axle's share of a_y, the
  motor's axle torque / wheel radius, and its power / v;
- resistance: drag v^2 / m, rolling roll g cos(theta), and gravity g sin(theta) along the road.
"""
import math

import numpy as np

G = 9.81


def _resist(car, v, cos_t, sin_t):
    return car.drag * v * v / car.m + car.roll * G * cos_t + G * sin_t


def _drive_accel(car, v, kappa, mu, cos_t):
    """The car's best forward acceleration from drive alone (before resistance)."""
    L, h = car.wheel_base, car.h_cg
    ay = v * v * kappa
    # the driven axle's static share and its share of the cornering force
    if car.drive_axle == "rear":
        static, lat_share, sign = car.lf / L, car.lf / L, 1.0
    elif car.drive_axle == "front":
        static, lat_share, sign = car.lr / L, car.lr / L, -1.0
    else:
        static, lat_share, sign = 1.0, 1.0, 0.0
    # a = mu' g cos static / (1 - sign mu' h / L) with mu' the circle's longitudinal part, from
    # the axle's lateral need at that load (a fixed point: two iterations do)
    a = 0.0
    for _ in range(3):
        load = G * cos_t * static + sign * a * h / L          # per unit mass on the axle
        lat = abs(ay) * lat_share
        lon = math.sqrt(max(0.0, (mu * load) ** 2 - lat * lat))
        a = lon
    motor = car.axle_torque_max / car.r_wheel / car.m
    if v > 0.1:
        motor = min(motor, car.power_max / (car.m * v))
    return max(0.0, min(a, motor))


def _brake_accel(car, v, kappa, mu, cos_t):
    ay = v * v * kappa
    return math.sqrt(max(0.0, (mu * G * cos_t) ** 2 - ay * ay))


def speed_profile(track, car, v_start=None, v_end=None, v_max=None):
    """v(s) on `track`'s line (its kappa, mu and grade). v_start / v_end: the speed at the ends
    (None: free); v_max: a cap (None: none)."""
    s, kappa, mu = track.s, np.abs(track.kappa), track.mu
    theta = np.arctan(track.grade)
    cos_t, sin_t = np.cos(theta), np.sin(theta)
    n = len(s)
    cap = np.full(n, np.inf if v_max is None else float(v_max))
    with np.errstate(divide="ignore"):
        cap = np.minimum(cap, np.sqrt(mu * G * cos_t / np.where(kappa > 1e-9, kappa, 1e-12)))
    if v_start is not None:
        cap[0] = min(cap[0], v_start)
    if v_end is not None:
        cap[-1] = min(cap[-1], v_end)
    fwd = cap.copy()
    for i in range(n - 1):
        ds = s[i + 1] - s[i]
        v = fwd[i]
        a = _drive_accel(car, v, kappa[i], mu[i], cos_t[i]) - _resist(car, v, cos_t[i], sin_t[i])
        fwd[i + 1] = min(fwd[i + 1], math.sqrt(max(0.0, v * v + 2 * a * ds)))
    bwd = fwd.copy()
    for i in range(n - 1, 0, -1):
        ds = s[i] - s[i - 1]
        v = bwd[i]
        a = _brake_accel(car, v, kappa[i], mu[i], cos_t[i]) + _resist(car, v, cos_t[i], sin_t[i])
        bwd[i - 1] = min(bwd[i - 1], math.sqrt(max(0.0, v * v + 2 * max(a, 0.0) * ds)))
    return bwd


def braking_points(track, v, min_drop=0.5):
    """Where the profile brakes: each run of falling speed, as {s, v_from, s_to, v_to}."""
    out, i, n = [], 0, len(v)
    while i < n - 1:
        if v[i + 1] < v[i] - 1e-6:
            j = i
            while j < n - 1 and v[j + 1] < v[j] - 1e-6:
                j += 1
            if v[i] - v[j] >= min_drop:
                out.append({"s": float(track.s[i]), "v_from": float(v[i]),
                            "s_to": float(track.s[j]), "v_to": float(v[j])})
            i = j
        i += 1
    return out
