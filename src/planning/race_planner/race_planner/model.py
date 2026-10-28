"""The car's planar dynamics as the planner optimises them: a two-track body on four tyres with
PhysX's own tyre model, every number from the spec and the surface's mu (no fitted constants).
Written in CasADi so that the same expressions serve the minimum-time problem (symbolic) and
the checks against Isaac (numeric, through `ca.Function`).

The tyre is PhysX's (PhysX 4/5 vehicle tyre, checked against the Isaac manoeuvre suite's sweeps
of drift_mustang on dry asphalt: Fx/Fz rms 0.012, Fy/Fz rms 0.041 over 3800 wheel-frames):

- lateral stiffness (N/rad): Fz0 lat_stiff_value f1(min(3 Fz / (Fz0 lat_stiff_max_load), 3)),
  with Fz0 the wheel's rest load and f1(K) = K - K^2/3 + K^3/27;
- longitudinal stiffness (N per unit slip): long_stiff_value g;
- combined: K = |(C_alpha tan(alpha), C_kappa kappa)| / (mu Fz), the force mu Fz f1(K) along that
  vector (f1 saturates at K = 3 with zero slope: the brush model's full slide);
- mu: the surface class's absolute peak times the tyre's friction (swb's surface layer; dry
  asphalt 1.12 at tyre friction 1.0, as measured);
- past the peak, swb's surface layer lowers mu with the combined slip rho = |(kappa / kappa_p,
  tan(alpha) / tan(alpha_p))| toward the class's `sliding` share: mu (sliding + (1 - sliding) /
  rho), the peaks the rig's own (2.3 Fz0 / (0.9 C) in each direction, surface_layer.rig_peaks);
  rho's knee at 1 is softened for the optimiser. (The layer also lags the drop over the class's
  relaxation length, 0.5 m on asphalt; a steady model leaves that out.)

The body is planar, x forward, y left, yaw rate r; the wheels at (lf, +-track/2), (-lr, +-track/2);
the front pair steered by Ackermann from the bicycle angle delta; the loads by the static split
and the quasi-static transfer from the body's accelerations (longitudinal over the wheel base,
lateral over the track, each axle its static share; the spec's suspension is the same on both
axles and has no anti-roll bars).

The wheels' spin is not a state: each wheel's peripheral speed (omega R, m/s) is an input, so a
wheel's slip is set directly and its torque is what the tyre's force asks of it (T = Fx R; the
wheel's inertia is left out, 22 kg at R 0.32 is a few milliseconds). The minimum-time problem
bounds those torques by the motor's and the brakes'. The rear axle's differential is a
constraint between its wheels: locked (one speed) or open (one torque).
"""
import casadi as ca

G = 9.81
# PhysX 5's slips (VhTireFunctions.cpp) with aib's minimum denominators (params.py): the
# longitudinal (omega R - v_x) / (|v_x| + 4), the lateral v_y / (|v_x| + 1). The tyre's
# stiffnesses act on these, as Isaac's logged slips are.
SLIP_LONG_OFFSET, SLIP_LAT_OFFSET = 4.0, 1.0


def _f1_over_k(K):
    """f1(K) / K, smooth: 1 - K/3 + K^2/27 below K = 3, 1/K above (C1 at 3)."""
    return ca.if_else(K < 3.0, 1.0 - K / 3.0 + K * K / 27.0, 1.0 / ca.fmax(K, 3.0))


def rest_loads(car):
    """Each wheel's static load (N): fl, fr, rl, rr."""
    L = car.wheel_base
    f, r = car.m * G * car.lr / L / 2.0, car.m * G * car.lf / L / 2.0
    return [f, f, r, r]


PEAK_SPAN, STIFFNESS_IN_USE = 2.3, 0.9    # swb's surface_layer.rig_peaks
KNEE = 30.0                                # the softened knee's sharpness (1 / rho)


def peaks(car):
    """The rig's peak slips at friction 1.0: (kappa_p, tan alpha_p), swb's rule."""
    load = car.m * G / 4
    return (PEAK_SPAN * load / (STIFFNESS_IN_USE * car.long_stiff_value * G),
            PEAK_SPAN / (STIFFNESS_IN_USE * car.lat_stiff_value))


def tyre(car, kappa, tan_alpha, fz, fz0, mu, sliding=1.0):
    """PhysX's tyre under swb's surface layer: (Fx, Fy) in the wheel's frame (Fy opposes the
    slip angle)."""
    kp, tp = peaks(car)
    rho = ca.sqrt((kappa / kp) ** 2 + (tan_alpha / tp) ** 2 + 1e-12)
    z = KNEE * (rho - 1.0)                 # softplus, in its overflow-free form
    rho = 1.0 + (ca.fmax(z, 0.0) + ca.log(1.0 + ca.exp(-ca.fabs(z)))) / KNEE
    mu = mu * (sliding + (1.0 - sliding) / rho)
    load = ca.fmin(3.0 * fz / (fz0 * car.lat_stiff_max_load), 3.0)
    c_lat = fz0 * car.lat_stiff_value * (load - load * load / 3.0 + load ** 3 / 27.0)
    c_lon = car.long_stiff_value * G
    sx, sy = c_lon * kappa, c_lat * tan_alpha
    K = ca.sqrt(sx * sx + sy * sy + 1e-9) / (mu * ca.fmax(fz, 1.0))
    q = _f1_over_k(K)
    return sx * q, -sy * q


def ackermann(car, delta):
    """The front wheels' angles (left, right) for the bicycle angle delta (> 0 left)."""
    L, t = car.wheel_base, car.track
    # cot(d_in) = cot(delta) - t / 2L, cot(d_out) = cot(delta) + t / 2L, about the rear axle
    tn = ca.tan(delta)
    left = ca.atan2(L * tn, L - 0.5 * t * tn)
    right = ca.atan2(L * tn, L + 0.5 * t * tn)
    return left, right


def wheels(car):
    """Each wheel's (x, y) on the body from the CoM: fl, fr, rl, rr."""
    h = 0.5 * car.track
    return [(car.lf, h), (car.lf, -h), (-car.lr, h), (-car.lr, -h)]


def loads(car, ax, ay):
    """Each wheel's load (N) under the body's accelerations ax, ay (m/s^2)."""
    L, t, h, m = car.wheel_base, car.track, car.h_cg, car.m
    st = rest_loads(car)
    dlon = m * ax * h / L / 2.0                       # per wheel, from the front to the rear
    share_f, share_r = car.lr / L, car.lf / L
    dlat_f = m * ay * h / t * share_f                 # per axle, from the right to the left
    dlat_r = m * ay * h / t * share_r
    return [st[0] - dlon - dlat_f, st[1] - dlon + dlat_f,
            st[2] + dlon - dlat_r, st[3] + dlon + dlat_r]


def dynamics(car, x, u, mu, acc, sliding=1.0):
    """The body's accelerations and what the wheels need.

    x = (vx, vy, r): the body's velocity (m/s, m/s, rad/s).
    u = (delta, w_fl, w_fr, w_rl, w_rr): the bicycle steering angle and each wheel's peripheral
        speed (m/s; omega R).
    mu: the friction under the wheels (a number or four); sliding: its share past the peak (the
        surface class's; a number or four).
    acc = (ax, ay): the body's accelerations the loads are taken at (the minimum-time problem
        makes them variables equal to the result; a simulation takes the last step's).

    Returns a dict: dx (dvx, dvy, dr), acc (ax, ay: the body's, ax = dvx - r vy, ay = dvy + r vx),
    torque (each wheel's, N m, > 0 driving), fz, kappa, alpha (tan), fx, fy (wheel frames),
    diff (the rear axle's constraint residual: 0 when the differential holds).
    """
    vx, vy, r = x[0], x[1], x[2]
    delta, w = u[0], [u[1], u[2], u[3], u[4]]
    mus = mu if isinstance(mu, (list, tuple)) else [mu] * 4
    sls = sliding if isinstance(sliding, (list, tuple)) else [sliding] * 4
    dl, dr_ = ackermann(car, delta)
    steer = [dl, dr_, 0.0, 0.0]
    fz = loads(car, acc[0], acc[1])
    fz0 = rest_loads(car)
    Fx = Fy = Mz = 0.0
    out = {k: [] for k in ("torque", "kappa", "alpha", "fx", "fy")}
    for i, (px, py) in enumerate(wheels(car)):
        ux, uy = vx - r * py, vy + r * px                 # the wheel's ground velocity, body
        c, s = ca.cos(steer[i]), ca.sin(steer[i])
        lx, ly = c * ux + s * uy, -s * ux + c * uy        # in the wheel's frame
        speed = ca.sqrt(lx * lx + 1e-4)
        kappa = (w[i] - lx) / (speed + SLIP_LONG_OFFSET)
        tan_a = ly / (speed + SLIP_LAT_OFFSET)
        fx, fy = tyre(car, kappa, tan_a, fz[i], fz0[i], mus[i], sls[i])
        bx, by = c * fx - s * fy, s * fx + c * fy         # back to the body
        Fx, Fy = Fx + bx, Fy + by
        Mz = Mz + px * by - py * bx
        for k, v in (("torque", fx * car.r_wheel), ("kappa", kappa), ("alpha", tan_a),
                     ("fx", fx), ("fy", fy)):
            out[k].append(v)
    speed = ca.sqrt(vx * vx + vy * vy + 1e-6)
    Fx = Fx - car.drag * speed * vx - car.roll * car.m * G * vx / speed
    Fy = Fy - car.drag * speed * vy
    ax, ay = Fx / car.m, Fy / car.m
    out["dx"] = [ax + r * vy, ay - r * vx, Mz / car.Iz]
    out["acc"] = [ax, ay]
    out["fz"] = fz
    if car.diff == "locked":
        out["diff"] = w[2] - w[3]
    else:   # open: the same torque each side
        out["diff"] = out["torque"][2] - out["torque"][3]
    return out
