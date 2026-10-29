"""The car driven by its real inputs, for the live planner (L1): steering and one acceleration
request, the ECU's split of it into wheel forces, and the tyres' lateral grip in what is left.

model.py's two-track car takes each wheel's spin as an input, which lets a plan ask for any
combination of wheel torques; a live planner re-solved every second from an imperfect warm start
does not converge on it in time (the live planner's design, M1). Here the four wheels keep their
geometry, their loads (the same quasi-static transfer) and PhysX's lateral tyre with swb's
sliding falloff, but their longitudinal forces are set by the request the way sim_vehicle_ecu
sets them, so a plan is made of what the car's vehicle interface can ask for (R4 by
construction, not by penalty):

- the request a (m/s^2) becomes a force at the tyres F = m a + drag v^2 + rolling m g, the
  ECU's feedforward (grade is the feedback's, not the feedforward's);
- F > 0 drives the rear axle; F < 0 brakes all four, the front pair taking the brake
  distribution's share (static: lr / L), each pair alike; drive and brake are the smooth
  positive and negative parts of one F, so they never act at once;
- the rear pair's split: a locked axle turns both wheels at one speed, so each takes force in
  proportion to its load; an open differential, the same each side;
- each wheel's lateral force is PhysX's pure-slip force at its slip angle and load (model.tyre
  with no longitudinal slip), reduced by the friction ellipse for the longitudinal force it
  carries: Fy = Fy0 sqrt(1 - (Fx / mu Fz)^2). |Fx| <= mu Fz is a constraint of the planner:
  past it the wheel spins or locks, which this model does not represent.

What it leaves out: the wheels' slip ratios (so wheelspin as such, and the drop of grip past the
longitudinal peak), and the locked axle's scrub torque in a corner beyond the load-proportional
split. L0 and the checks keep the two-track model.

With the rear wheels' speed given (`w_rear`, the locked axle's one peripheral speed omega R),
the rear tyres are PhysX's own instead (model.tyre on each wheel's slip ratio and angle, the
force along the slip vector, swb's sliding share), and the ECU's rear force is not split by the
model but must be what the two tyres make: `rear_balance` = Fx_rl + Fx_rr - (drive - (1 -
share) brake), held at zero by the planner (L1). Why: the ellipse let a driven rear carry
0.98 mu Fz along and 0.18 mu Fz across at 7 deg of slip, which PhysX's tyre cannot (0.84 mu Fz
at most there, sliding); so L1 planned a 7 deg slide under full drive on Haruna's gentle bend at
s 90-160 that the car could not deliver (L0 holds 1 deg there), fell 4 m/s behind and left the
road (M3, 2026-10-30). Likewise the front pair with its wheels' speeds given (`w_front`, one
each: they only brake, through no differential): PhysX's tyre on each, and `front_balance`
holds each one's Fx to its share of the brakes, -share brake / 2. Why: past the tyre's peak slip
angle (8.5 deg at rest load) the ellipse is optimistic under braking (at 8 deg and 0.8 of the
grip along, 0.58 mu Fz across against PhysX's 0.44), and a brake force past the tyre's real
peak at that angle locks the wheel in a car without ABS: the inner front wheel locked (slip
-0.6 to -0.8) braking into Haruna's hairpin at s 270, the car could not turn and left the road
(M3, 2026-10-30).
"""
import casadi as ca

from race_planner import model

G = model.G
SMOOTH = 50.0          # N: the width over which drive hands over to brake (smooth max)
ELLIPSE_EPS = 0.02     # the friction ellipse's rounding at its edge (share of the grip)


def _pos(x):
    """x's positive part, smooth (C-infinity, within SMOOTH / 2 of the exact one)."""
    return 0.5 * (x + ca.sqrt(x * x + SMOOTH * SMOOTH))


def _smin(a, b):
    """min(a, b), smooth (within SMOOTH / 2)."""
    return 0.5 * (a + b - ca.sqrt((a - b) * (a - b) + SMOOTH * SMOOTH))


def _smax(a, b):
    return 0.5 * (a + b + ca.sqrt((a - b) * (a - b) + SMOOTH * SMOOTH))


def brake_share(car, decel=0.0):
    """The front axle's share of the friction brakes' split (sim_vehicle_ecu's
    calibration.brake_split, its deceleration capped at the tyres' mu g as the ECU's EBD)."""
    share = car.lr / car.wheel_base
    if car.brake_distribution == "dynamic_axle_load":
        decel = ca.fmin(ca.fmax(decel, 0.0), car.mu * G)
        share = share + car.h_cg * decel / (G * car.wheel_base)
    return ca.fmin(ca.fmax(share, 0.05), 0.95)


def feedforward(car, a, vx):
    """The ECU's tractive force for the request a at forward speed vx (sim_vehicle_ecu's
    powertrain.feedforward): the car and its wheels' inertia times a, the road load and the
    driveline's drag. Above the rolling ramp's 0.5 m/s, which a plan does not go under."""
    mass = car.m + 2 * car.wheel_mass
    return (mass * a + car.drag * vx * vx + car.roll * car.m * G
            + 4 * car.wheel_damping * vx / car.r_wheel ** 2)


def regen_max(car, vx):
    """The most braking force (N at the tyres) the motor regenerates at forward speed vx: its
    torque and power limits, faded out below regen_fade (powertrain.regen_limit)."""
    R = car.r_wheel
    lim = _smin(car.regen_torque_max / R, car.regen_power_max / ca.sqrt(vx * vx + 1.0))
    lo, hi = car.regen_fade
    return lim * ca.fmin(ca.fmax((vx - lo) / (hi - lo), 0.0), 1.0)


def brakes(car, a, vx):
    """What the ECU makes of the request a at forward speed vx (sim_vehicle_ecu's ev
    powertrain): (drive, regen, front, rear), N at the tyres, all >= 0. A positive force drives
    the rear. A braking force B is regenerative first, within regen_max, on the rear (the
    driven) axle; the friction brakes are split by brake_share and the regen taken off its axle's
    share, the excess of it (mild braking: more regen than the rear's share) coming off the
    front's friction. So `rear` = max(regen, (1 - share) B) with the regen in it, `front` = B -
    rear. The friction brakes' hydraulic lag (dead time 0.05 s, then 0.04 s first order on
    drift_mustang) and the motor's (0.02 s) are left out: under a tenth of a second, inside one
    of L1's intervals (0.13 s at 30 m/s), while a 1 s window turns on the steady split."""
    F = feedforward(car, a, vx)
    drive, B = _pos(F), _pos(-F)
    share = brake_share(car, -a)
    regen = _smin(B, regen_max(car, vx))
    rear = _smax(regen, (1.0 - share) * B)
    return drive, regen, B - rear, rear


def dynamics(car, x, u, mu, acc, sliding=1.0, w_rear=None, w_front=None):
    """x = (vx, vy, r); u = (delta, a): the bicycle steering angle and the acceleration request;
    acc = (ax, ay) the loads are taken at (equal to the result in the planner); w_rear: the
    rear axle's peripheral speed (m/s), or None for the ellipse there (see above); w_front:
    the front wheels' (left, right), or None for the ellipse on them.

    Returns a dict: dx, acc, fz, fx and fy (each wheel's, wheel frame), use (each wheel's
    |Fx| / (mu Fz): <= 1 for the force to be carried), drive and brake (the tyre force the
    motor and the brakes make, N, >= 0), alpha (tan).
    """
    vx, vy, r = x[0], x[1], x[2]
    delta, a = u[0], u[1]
    mus = mu if isinstance(mu, (list, tuple)) else [mu] * 4
    sls = sliding if isinstance(sliding, (list, tuple)) else [sliding] * 4
    speed = ca.sqrt(vx * vx + vy * vy + 1e-6)
    drive, regen, front_brake, rear_brake = brakes(car, a, vx)
    brake = front_brake + rear_brake
    share = brake_share(car, -a)
    fz = model.loads(car, acc[0], acc[1])
    fz0 = model.rest_loads(car)
    # what of the ECU's wheel torques the wheels keep: the driveline's drag (the feedforward
    # adds it, PhysX's wheel damping takes it) and their own inertia at the body's acceleration
    loss = car.wheel_damping * vx / car.r_wheel ** 2 + 0.5 * car.wheel_mass * acc[0]
    rear = drive - rear_brake - 2 * loss
    if car.diff == "locked":
        tot = fz[2] + fz[3]
        rl, rr = rear * fz[2] / tot, rear * fz[3] / tot
    else:
        rl = rr = 0.5 * rear
    fx = [-0.5 * front_brake - loss, -0.5 * front_brake - loss, rl, rr]
    if w_rear is not None:
        assert car.diff == "locked", "the rear tyres' own slip: a locked axle only"
    kappas, kappas_f = [], []
    dl, dr_ = model.ackermann(car, delta)
    steer = [dl, dr_, 0.0, 0.0]
    Fx = Fy = Mz = 0.0
    out = {k: [] for k in ("fx", "fy", "use", "alpha")}
    for i, (px, py) in enumerate(model.wheels(car)):
        ux, uy = vx - r * py, vy + r * px
        c, s = ca.cos(steer[i]), ca.sin(steer[i])
        lx, ly = c * ux + s * uy, -s * ux + c * uy
        tan_a = ly / (ca.sqrt(lx * lx + 1e-4) + model.SLIP_LAT_OFFSET)
        w_own = (w_rear if i >= 2 else None if w_front is None else w_front[i])
        if w_own is not None:                          # PhysX's tyre, combined slip
            kap = (w_own - lx) / (ca.sqrt(lx * lx + 1e-4) + model.SLIP_LONG_OFFSET)
            fx[i], fy = model.tyre(car, kap, tan_a, fz[i], fz0[i], mus[i], sls[i])
            (kappas if i >= 2 else kappas_f).append(kap)
            use = fx[i] / (mus[i] * ca.sqrt(fz[i] * fz[i] + 1.0))
            bx, by = c * fx[i] - s * fy, s * fx[i] + c * fy
            Fx, Fy = Fx + bx, Fy + by
            Mz = Mz + px * by - py * bx
            for k, v in (("fx", fx[i]), ("fy", fy), ("use", use), ("alpha", tan_a)):
                out[k].append(v)
            continue
        _, fy0 = model.tyre(car, 0.0, tan_a, fz[i], fz0[i], mus[i], sls[i])
        cap = mus[i] * ca.sqrt(fz[i] * fz[i] + 1.0)
        use = fx[i] / cap
        # the ellipse's share left for Fy, smooth through the limit (a kink there stalls the
        # optimiser's Newton steps exactly where a fast plan lives)
        # (its positive part written without cancellation: l + sqrt(l^2 + e^2) = e^2 /
        # (sqrt(l^2 + e^2) - l) for l < 0, else a far overdrawn wheel, |use| ~ 1e3 when an
        # iterate unloads it, rounds the root to 0 and its derivative to NaN)
        left = 1.0 - use * use
        root = ca.sqrt(left * left + ELLIPSE_EPS ** 2)
        pos = ca.if_else(left > 0, left + root, ELLIPSE_EPS ** 2 / (root - left))
        fy = fy0 * ca.sqrt(0.5 * pos)
        bx, by = c * fx[i] - s * fy, s * fx[i] + c * fy
        Fx, Fy = Fx + bx, Fy + by
        Mz = Mz + px * by - py * bx
        for k, v in (("fx", fx[i]), ("fy", fy), ("use", use), ("alpha", tan_a)):
            out[k].append(v)
    Fx = Fx - car.drag * speed * vx - car.roll * car.m * G * vx / speed
    Fy = Fy - car.drag * speed * vy
    ax, ay = Fx / car.m, Fy / car.m
    out["dx"] = [ax + r * vy, ay - r * vx, Mz / car.Iz]
    out["acc"] = [ax, ay]
    out["fz"] = fz
    out["drive"], out["brake"], out["share"] = drive, brake, share
    out["regen"], out["front_brake"], out["rear_brake"] = regen, front_brake, rear_brake
    if w_rear is not None:
        out["kappa"] = kappas
        out["rear_balance"] = out["fx"][2] + out["fx"][3] - rear
    if w_front is not None:
        out["kappa_front"] = kappas_f
        out["front_balance"] = [out["fx"][j] + 0.5 * front_brake + loss for j in (0, 1)]
    return out
