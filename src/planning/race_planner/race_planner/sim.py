"""A model-in-the-loop plant: the car the planner models, integrated in time, behind the car's own
onboard computers, so the live planner and tracker can be developed faster than real time before
Isaac judges them (the live planner's design, section 6).

- The body and tyres: model.py's two-track car (PhysX's tyre, swb's sliding falloff, the
  quasi-static load transfer) with each wheel's spin a state: I w' = drive - brake - Fx R
  - d w, d the spec's wheel damping_rate (PhysX's wheel damping, which the ECU's feedforward
  makes up for: without it the car would get 0.13 m/s^2 more than asked at 20 m/s).
  Richer than L1's axle car on purpose (wheel slip, the locked axle's scrub), so plan and plant
  disagree as plan and Isaac will.
- The onboard computers: sim_vehicle_ecu itself (the code aib runs in Isaac), stepped at
  Isaac's 60 Hz with the car's own sensors (wheel speeds, yaw rate, accelerations); the body
  and wheels at 8 substeps of it, as PhysX's vehicle.
- The road: the track's friction, sliding share, tyre (the surface's peak, model.tyre's
  `surface`) and grade at the car's nearest node. A patch or mu_scale lowers the friction
  alone, the stiffness kept: the tyre's peak slip falls with it (as the planner's grip share
  does), where swb's road condition would keep the class's peak (roadmap 061 M2).
- A wrong plant on request: grip and load transfer scaled (mu_scale, transfer_scale), the
  plan's model left as it is; and local friction patches the planner's map does not know
  (`patches`: (s_from, s_to, scale) each, the friction there times scale), as Isaac's M6 test
  puts one at a hairpin's entry.
- The car's own friction estimate, the ECU's (sim_vehicle_ecu.estimator), in the state as
  `mu_ratio`: the estimate over the tyre's design friction, as Isaac's car reports it.

sim_vehicle_ecu is found beside this repository's checkout of the superproject
(SWB/src/sim_vehicle_ecu, SWB from the environment, default ~/repos/carla-scenario-bridge).
"""
from dataclasses import replace
import math
import os
import sys

import casadi as ca
import numpy as np
import yaml

from race_planner import model
from race_planner.track import TYRE_FIELDS

SWB = os.environ.get("SWB", os.path.expanduser("~/repos/carla-scenario-bridge"))
ECU_RATE, SUBSTEPS = 60.0, 8


def _ecu_module():
    path = os.path.join(SWB, "src", "sim_vehicle_ecu")
    if path not in sys.path:
        sys.path.insert(0, path)
    import sim_vehicle_ecu
    return sim_vehicle_ecu


class Plant:
    """The car on `track`. State: x, y, psi (map frame, the CoM), vx, vy, r (body), the four
    wheels' spin w (rad/s) and the steering's actual angle."""

    def __init__(self, car, spec_path, track, mu_scale=1.0, transfer_scale=1.0, patches=()):
        ecu = _ecu_module()
        doc = yaml.safe_load(open(spec_path))
        self.ecu_mod = ecu
        self.ecu = ecu.Ecu(ecu.Calibration.from_spec(doc))
        self.car = replace(car, transfer_height=car.transfer_height * transfer_scale)
        self.track, self.mu_scale = track, mu_scale
        self.patches = [tuple(map(float, p)) for p in patches]
        w = doc["wheels"] if "wheels" in doc else {}
        m_wheel = float(w.get("mass", 22.0))
        self.I_wheel = 0.5 * m_wheel * self.car.r_wheel ** 2
        self.damping = float(w.get("damping_rate", 0.0))
        xs, us, acc = ca.SX.sym("x", 3), ca.SX.sym("u", 5), ca.SX.sym("acc", 2)
        mu, sl, sf = ca.SX.sym("mu"), ca.SX.sym("sl"), ca.SX.sym("surface", 4)
        o = model.dynamics(self.car, [xs[0], xs[1], xs[2]], [us[i] for i in range(5)], mu,
                           [acc[0], acc[1]], sl, sf)
        self.f = ca.Function("f", [xs, us, acc, mu, sl, sf],
                             [ca.vertcat(*o["dx"]), ca.vertcat(*o["acc"]),
                              ca.vertcat(*o["torque"]), ca.vertcat(*o["fz"])])
        self.t = 0.0
        self.acc = np.zeros(2)
        self.last_act = None

    def reset(self, s=None, n=0.0, speed=5.0, heading_error=0.0):
        """Put the car on the track at node nearest s (default its start), at offset n, rolling
        straight at `speed`."""
        tr = self.track
        i = 0 if s is None else int(np.argmin(np.abs(tr.s - s)))
        psi = float(tr.psi[i]) + heading_error
        nx, ny = -math.sin(tr.psi[i]), math.cos(tr.psi[i])
        self.x, self.y, self.psi = float(tr.x[i] + n * nx), float(tr.y[i] + n * ny), psi
        self.v = np.array([speed, 0.0, 0.0])
        self.w = np.full(4, speed / self.car.r_wheel)
        self.delta = 0.0
        self.t, self.i, self.t_target = 0.0, i, 0.0
        self.acc = np.zeros(2)
        self.ecu.reset() if hasattr(self.ecu, "reset") else None
        return self.state()

    def patch(self, s):
        """The friction's scale at s from the patches (1 off them)."""
        for lo, hi, scale in self.patches:
            if lo <= s <= hi:
                return scale
        return 1.0

    def nearest(self):
        tr = self.track
        lo, hi = max(0, self.i - 20), min(len(tr.s), self.i + 60)
        d = np.hypot(tr.x[lo:hi] - self.x, tr.y[lo:hi] - self.y)
        self.i = lo + int(np.argmin(d))
        return self.i

    def state(self):
        """The car's state in the planner's terms: s, n, xi (on the track) and vx, vy, r, delta,
        plus x, y, psi and the wheels."""
        tr, i = self.track, self.nearest()
        ps = float(tr.psi[i])
        dx, dy = self.x - tr.x[i], self.y - tr.y[i]
        along = dx * math.cos(ps) + dy * math.sin(ps)
        n = -dx * math.sin(ps) + dy * math.cos(ps)
        xi = (self.psi - ps + math.pi) % (2 * math.pi) - math.pi
        return {"t": self.t, "s": float(tr.s[i] + along), "n": n, "xi": xi,
                "vx": float(self.v[0]), "vy": float(self.v[1]), "r": float(self.v[2]),
                "delta": self.delta, "x": self.x, "y": self.y, "psi": self.psi,
                "w": self.w.copy(), "ax": float(self.acc[0]), "ay": float(self.acc[1]),
                "i": i, "mu_ratio": self.ecu.estimator.friction / self.ecu.c.tire_friction}

    def step(self, accel, tire_angle, velocity=60.0, dt=1.0 / ECU_RATE):
        """Advance dt with the vehicle interface's request held: the ECU's 60 Hz steps that fall
        in it (the remainder carried to the next call, as a fixed-rate physics would)."""
        ecu = self.ecu_mod
        self.t_target = getattr(self, "t_target", self.t) + dt
        R = self.car.r_wheel
        while self.t + 0.5 / ECU_RATE <= self.t_target:
            sensors = ecu.Sensors(tuple(float(w) for w in self.w), float(self.v[2]),
                                  float(self.acc[0]), float(self.acc[1]))
            act = self.ecu.step(ecu.Request(float(accel), float(velocity), float(tire_angle)),
                                sensors, 1.0 / ECU_RATE)
            self.last_act = act
            self.delta = float(act.tire_angle)
            h = 1.0 / ECU_RATE / SUBSTEPS
            i = self.nearest()
            mu = float(self.track.mu[i]) * self.mu_scale * self.patch(float(self.track.s[i]))
            sl = float(self.track.sliding[i])
            sf = [float(getattr(self.track, k)[i]) for k in TYRE_FIELDS]
            grade = float(self.track.grade[i])
            for _ in range(SUBSTEPS):
                u = [self.delta] + list(self.w * R)
                dx, acc, torque, fz = (np.asarray(q).ravel() for q in
                                       self.f(self.v, u, self.acc, mu, sl, sf))
                # grade along the body's heading on the track (the planner's sign: a + grade climbs)
                xi = self.psi - float(self.track.psi[i])
                g_t = 9.81 * grade / math.sqrt(1 + grade * grade)
                dx = dx + np.array([-g_t * math.cos(xi), g_t * math.sin(xi), 0.0])
                # the wheels: drive and brake torque against the tyre's (torque = Fx R)
                wdot = np.zeros(4)
                for k in range(4):
                    brake = act.brake[k] * math.tanh(self.w[k] / 0.5)
                    wdot[k] = (act.drive[k] - brake - torque[k]
                               - self.damping * self.w[k]) / self.I_wheel
                self.v = self.v + h * dx
                self.w = self.w + h * wdot
                self.w = np.where(np.abs(self.w) < 1e-3, 0.0, self.w)
                self.acc = acc
                c, s = math.cos(self.psi), math.sin(self.psi)
                self.x += h * (c * self.v[0] - s * self.v[1])
                self.y += h * (s * self.v[0] + c * self.v[1])
                self.psi += h * self.v[2]
            self.t += 1.0 / ECU_RATE
        return self.state()


class NoTracker:
    """A tracker for PlanFollower's car: it commands nothing (the car is on its plan)."""

    def step(self, state, plan, plan_t0):
        return {"accel": 0.0, "tire_angle": 0.0, "ok": True, "solve_s": 0.0, "fallback": False}


class PlanFollower:
    """No plant at all: the car on the plan its driver has adopted, exactly (roadmap 061 M3's
    receding chain). It moves along the plan's own time and its state is the plan's where it is,
    so what is measured is the re-planning alone. Drive it with a live.Driver given NoTracker."""

    def __init__(self, driver, s0, v0):
        self.d, self.t, self.s = driver, 0.0, s0
        self.x = np.array([v0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def state(self):
        vx, vy, r, n, xi, delta = (float(v) for v in self.x)
        return {"t": self.t, "s": self.s, "vx": vx, "vy": vy, "r": r, "n": n, "xi": xi,
                "delta": delta}

    def step(self, dt):
        from race_planner.live import plan_at
        p = self.d.plan
        tp = float(np.interp(self.s, p["s"], p["t"]))
        if tp + dt <= p["t"][-1]:
            self.s = float(np.interp(tp + dt, p["t"], p["s"]))
        else:
            self.s += dt * max(float(self.x[0]), 1.0)
        self.x, _, _ = plan_at(p, self.s)
        self.t += dt
        return self.state()
