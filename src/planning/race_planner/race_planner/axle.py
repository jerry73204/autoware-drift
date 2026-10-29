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
road (M3, 2026-10-30). The front pair, which only brakes, keeps the ellipse.
"""
import casadi as ca

from race_planner import model

G = model.G
SMOOTH = 50.0          # N: the width over which drive hands over to brake (smooth max)
ELLIPSE_EPS = 0.02     # the friction ellipse's rounding at its edge (share of the grip)


def _pos(x):
    """x's positive part, smooth (C-infinity, within SMOOTH / 2 of the exact one)."""
    return 0.5 * (x + ca.sqrt(x * x + SMOOTH * SMOOTH))


def brake_share(car, decel=0.0):
    """The front axle's share of a brake force (sim_vehicle_ecu's calibration.brake_split)."""
    share = car.lr / car.wheel_base
    if car.brake_distribution == "dynamic_axle_load":
        share = share + car.h_cg * ca.fmax(decel, 0.0) / (G * car.wheel_base)
    return ca.fmin(ca.fmax(share, 0.05), 0.95)


def dynamics(car, x, u, mu, acc, sliding=1.0, w_rear=None):
    """x = (vx, vy, r); u = (delta, a): the bicycle steering angle and the acceleration request;
    acc = (ax, ay) the loads are taken at (equal to the result in the planner); w_rear: the
    rear axle's peripheral speed (m/s), or None for the ellipse on all four (see above).

    Returns a dict: dx, acc, fz, fx and fy (each wheel's, wheel frame), use (each wheel's
    |Fx| / (mu Fz): <= 1 for the force to be carried), drive and brake (the tyre force the
    motor and the brakes make, N, >= 0), alpha (tan).
    """
    vx, vy, r = x[0], x[1], x[2]
    delta, a = u[0], u[1]
    mus = mu if isinstance(mu, (list, tuple)) else [mu] * 4
    sls = sliding if isinstance(sliding, (list, tuple)) else [sliding] * 4
    speed = ca.sqrt(vx * vx + vy * vy + 1e-6)
    road = car.drag * speed * speed + car.roll * car.m * G
    F = car.m * a + road
    drive, brake = _pos(F), _pos(-F)
    share = brake_share(car, -a)
    fz = model.loads(car, acc[0], acc[1])
    fz0 = model.rest_loads(car)
    rear = drive - (1.0 - share) * brake
    if car.diff == "locked":
        tot = fz[2] + fz[3]
        rl, rr = rear * fz[2] / tot, rear * fz[3] / tot
    else:
        rl = rr = 0.5 * rear
    fx = [-0.5 * share * brake, -0.5 * share * brake, rl, rr]
    if w_rear is not None:
        assert car.diff == "locked", "the rear tyres' own slip: a locked axle only"
    kappas = []
    dl, dr_ = model.ackermann(car, delta)
    steer = [dl, dr_, 0.0, 0.0]
    Fx = Fy = Mz = 0.0
    out = {k: [] for k in ("fx", "fy", "use", "alpha")}
    for i, (px, py) in enumerate(model.wheels(car)):
        ux, uy = vx - r * py, vy + r * px
        c, s = ca.cos(steer[i]), ca.sin(steer[i])
        lx, ly = c * ux + s * uy, -s * ux + c * uy
        tan_a = ly / (ca.sqrt(lx * lx + 1e-4) + model.SLIP_LAT_OFFSET)
        if w_rear is not None and i >= 2:              # PhysX's tyre, combined slip
            kap = (w_rear - lx) / (ca.sqrt(lx * lx + 1e-4) + model.SLIP_LONG_OFFSET)
            fx[i], fy = model.tyre(car, kap, tan_a, fz[i], fz0[i], mus[i], sls[i])
            kappas.append(kap)
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
    if w_rear is not None:
        out["kappa"] = kappas
        out["rear_balance"] = out["fx"][2] + out["fx"][3] - rear
    return out
