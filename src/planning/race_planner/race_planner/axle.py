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
- the motor regenerates first, on the rear; on a road too slick for its full regen that falls
  off past its peak (`held`: ice) within regen_grip_share of the rear's grip, as the ECU's regen slip control
  holds it once a slip has shown the grip (issue 009), the front's friction brakes taking
  their share of the rest;
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
from dataclasses import replace

import casadi as ca
import numpy as np

from race_planner import model

G = model.G
HOLD_SLIDING = 0.75    # the hold's surfaces fall off past their peak this far (window.SLIDING_LOW)
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


_SLOPES = {}


def _slope(car, fz0, surface=False):
    """dFx/dkappa of PhysX's tyre (model.tyre) at a wheel of rest load fz0, as a function of
    (kappa, tan alpha, fz, mu, sliding), and of the surface's tyre (model.tyre's `surface`, four
    more inputs) when `surface`. A wheel whose force falls as its slip grows is unstable
    without traction control or ABS: its spin runs away (sim.py's plant: a request whose rear
    force is just past the peak, a 6.0 on drift_mustang at 15 m/s with sliding 0.8, takes the slip
    from 0.27 to 0.54 in 1.5 s; 6.5 to 1.0 at once), so a plan must stay on the rising branch."""
    key = (id(car), float(fz0), bool(surface))
    if key not in _SLOPES:
        k, ta, fz, mu, sl = (ca.SX.sym(n) for n in ("k", "ta", "fz", "mu", "sl"))
        sf = ca.SX.sym("surface", 4) if surface else None
        fx, _ = model.tyre(car, k, ta, fz, fz0, mu, sl, sf)
        _SLOPES[key] = ca.Function("tyre_slope", [k, ta, fz, mu, sl] + ([sf] if surface else []),
                                   [ca.jacobian(fx, k)])
    return _SLOPES[key]


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


def regen_slip_mu(car):
    """The friction below which the motor's full regen slips the rear (the driven) axle: its
    regen force at the tyres over the rear axle's load at the deceleration that force alone
    gives (the ECU's load shift, on the CoM's height). Below it the car's regen slip control
    (sim_vehicle_ecu, issue 009) finds the axle's grip at the first stop and holds regen within
    car.regen_grip_share of it from then on; above it regen never slips the rear and is not
    held. drift_mustang: 2812 N over 6644 N, 0.42 (ice, snow, sand and mud below; grass, gravel,
    dirt and asphalt above)."""
    F = car.regen_torque_max / car.r_wheel
    a = F / car.m
    rear = car.m * G * car.lf / car.wheel_base - car.m * a * car.h_cg / car.wheel_base
    return F / rear


def held(car, mu, sliding):
    """Whether the model holds regen (issue 009) on a road of friction `mu` and sliding share
    `sliding` (numbers or symbols): slick enough for the motor's full regen to slip the rear
    (regen_slip_mu), and falling off past its peak as ice does (sliding at most HOLD_SLIDING).

    Why only there: on ice the rear's grip is below even mild regen and its force falls past
    the peak, so a rear-only brake leaves the car at 0.46 mu g and L1 brakes with its front
    tyres steered to lock (the issue); the car, braking on ice, finds the rear's grip at once.
    On snow and mud the rear keeps its force past the peak, the plans keep it on its rising
    branch and brake through the rear alone, the car never slips it and so never holds: with
    the hold in the model there (and the car told the grip) the plans used the fronts in a
    braking drift that left every 6 m packed and loose snow road but one (M4g: packed snow 28
    of 36 against 36, loose snow 24 against 36)."""
    if isinstance(mu, (int, float)) and isinstance(sliding, (int, float)):
        return mu < regen_slip_mu(car) and sliding <= HOLD_SLIDING
    return ca.logic_and(mu < regen_slip_mu(car), sliding <= HOLD_SLIDING)


def regen_hold_car(car, mu, sliding):
    """`car` for planning on a road of friction `mu` and sliding share `sliding` (the nodes',
    arrays): as it is when a node is `held`, where the car's regen slip control holds its
    regen; with regen_grip_share None (axle.dynamics without the hold) when none is, so the
    plans and L2 elsewhere are what they were before issue 009's hold, bit for bit (a switch
    in the model, even one that picks the old expression exactly, changes its derivatives'
    rounding)."""
    mu, sliding = np.asarray(mu, float), np.asarray(sliding, float)
    if car.regen_grip_share and np.any((mu < regen_slip_mu(car)) & (sliding <= HOLD_SLIDING)):
        return car
    return replace(car, regen_grip_share=None)


def brakes(car, a, vx, cap=None):
    """What the ECU makes of the request a at forward speed vx (sim_vehicle_ecu's ev
    powertrain): (drive, regen, front, rear), N at the tyres, all >= 0. A positive force drives
    the rear. A braking force B is regenerative first, within regen_max and `cap` (the regen slip
    control's hold, N, or None: none; see `dynamics`), on the rear (the driven) axle; the friction
    brakes are split by brake_share and the regen taken off its axle's share, the excess of it
    (mild braking: more regen than the rear's share) coming off the front's friction. So `rear`
    = max(regen, (1 - share) B) with the regen in it, `front` = B - rear. The friction brakes'
    hydraulic lag (dead time 0.05 s, then 0.04 s first order on drift_mustang) and the motor's
    (0.02 s) are left out: under a tenth of a second, inside one of L1's intervals (0.13 s at
    30 m/s), while a 1 s window turns on the steady split."""
    F = feedforward(car, a, vx)
    drive, B = _pos(F), _pos(-F)
    share = brake_share(car, -a)
    regen = _smin(B, regen_max(car, vx))
    if cap is not None:
        regen = _smin(regen, cap)
    rear = _smax(regen, (1.0 - share) * B)
    return drive, regen, B - rear, rear


def dynamics(car, x, u, mu, acc, sliding=1.0, w_rear=None, w_front=None, surface=None):
    """x = (vx, vy, r); u = (delta, a): the bicycle steering angle and the acceleration request;
    acc = (ax, ay) the loads are taken at (equal to the result in the planner); w_rear: the
    rear axle's peripheral speed (m/s), or None for the ellipse there (see above); w_front:
    the front wheels' (left, right), or None for the ellipse on them; surface: the road's tyre
    (model.tyre's, the four wheels alike; None the rig's).

    Returns a dict: dx, acc, fz, fx and fy (each wheel's, wheel frame), use (each wheel's
    |Fx| / (mu Fz): <= 1 for the force to be carried), drive and brake (the tyre force the
    motor and the brakes make, N, >= 0), alpha (tan).
    """
    vx, vy, r = x[0], x[1], x[2]
    delta, a = u[0], u[1]
    mus = mu if isinstance(mu, (list, tuple)) else [mu] * 4
    sls = sliding if isinstance(sliding, (list, tuple)) else [sliding] * 4
    speed = ca.sqrt(vx * vx + vy * vy + 1e-6)
    fz = model.loads(car, acc[0], acc[1])
    fz0 = model.rest_loads(car)
    drive, regen, front_brake, rear_brake = brakes(car, a, vx)
    if car.regen_grip_share and car.regen_torque_max > 0 and car.drive_axle == "rear":
        # the regen slip control (issue 009): where the road is too slick for the motor's full
        # regen, the car has found the rear's grip and holds regen within regen_grip_share of
        # it (smooth: a kink here stalled L2's QPs every other tick, M4g); a switch on the
        # road's friction and sliding share (`held`), parameters
        mu_rear, sl_rear = 0.5 * (mus[2] + mus[3]), 0.5 * (sls[2] + sls[3])
        hold = brakes(car, a, vx, car.regen_grip_share * mu_rear * (fz[2] + fz[3]))
        slick = held(car, mu_rear, sl_rear)
        drive, regen, front_brake, rear_brake = (ca.if_else(slick, h, f) for h, f in
                                                 zip(hold, (drive, regen, front_brake,
                                                            rear_brake)))
    brake = front_brake + rear_brake
    share = brake_share(car, -a)
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
    kappas, kappas_f, slopes = [], [], []
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
            fx[i], fy = model.tyre(car, kap, tan_a, fz[i], fz0[i], mus[i], sls[i], surface)
            (kappas if i >= 2 else kappas_f).append(kap)
            sf = [] if surface is None else [ca.vertcat(*(surface[j] for j in range(4)))]
            slopes.append(_slope(car, fz0[i], surface is not None)(kap, tan_a, fz[i], mus[i],
                                                                   sls[i], *sf))
            use = fx[i] / (mus[i] * ca.sqrt(fz[i] * fz[i] + 1.0))
            bx, by = c * fx[i] - s * fy, s * fx[i] + c * fy
            Fx, Fy = Fx + bx, Fy + by
            Mz = Mz + px * by - py * bx
            for k, v in (("fx", fx[i]), ("fy", fy), ("use", use), ("alpha", tan_a)):
                out[k].append(v)
            continue
        _, fy0 = model.tyre(car, 0.0, tan_a, fz[i], fz0[i], mus[i], sls[i], surface)
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
    if slopes:
        out["slope"] = slopes            # dFx/dkappa per tyre-modelled wheel, in wheel order
    if w_front is not None:
        out["kappa_front"] = kappas_f
        out["front_balance"] = [out["fx"][j] + 0.5 * front_brake + loss for j in (0, 1)]
    return out
