"""The two-track model against PhysX's tyre and against Isaac's drift_mustang.

test/data/isaac_drift_mustang_dry.npz is from swb_isaac_world's manoeuvre suite (spikes/
manoeuvres.py, EGO_RIG=drift_mustang.rig.yaml, PADS=dry, 60 Hz, 2026-10-29): `tyre`, 800
wheel-frames of the lateral and longitudinal sweeps (wheel, kappa, tan alpha, Fz, Fx, Fy);
`circle`, the 40 m constant-radius ramp every 7.5 s (CoM speed, radius, tyre angle, CoM
sideslip; Isaac logs its sideslip at the rear axle, moved here by r lr); `step`, the 5 deg step at
40 km/h (speed, angle, steady yaw rate, steady ay).

Isaac's rear axle is the onboard computers' "locked" differential, which in a steady corner is
nearly open (swb docs/issues/022: its rear wheels' slips differ by 0.004 on the 40 m circle where
a spool's would by t/R = 0.040), so the steady checks run the model with an open differential.
"""
import dataclasses
import math
import os

import casadi as ca
import numpy as np
import pytest

from race_planner import model
from race_planner.car import Car
from race_planner.steady import steady_state

SWB = os.environ.get("SWB", os.path.expanduser("~/repos/carla-scenario-bridge"))
SPEC = os.path.join(SWB, "src/sim_vehicle_spec/vehicles/drift_mustang.yaml")
ISAAC = np.load(os.path.join(os.path.dirname(__file__), "data/isaac_drift_mustang_dry.npz"))
MU = 1.12   # dry asphalt, swb's surface classes (absolute, at tyre friction 1.0)


def tyre_fn(car):
    k, t, fz, fz0, mu = ca.SX.sym("k"), ca.SX.sym("t"), ca.SX.sym("fz"), ca.SX.sym("fz0"), ca.SX.sym("mu")
    return ca.Function("tyre", [k, t, fz, fz0, mu], list(model.tyre(car, k, t, fz, fz0, mu)))


def test_the_tyre_is_physx_s():
    car = Car.from_spec(SPEC)
    f = tyre_fn(car)
    fz0 = model.rest_loads(car)[0]
    # small slips: the stiffnesses (lateral at rest load: lat_stiff_value f1(3 / lat_stiff_max_load))
    k1 = 1.5
    c_lat = fz0 * car.lat_stiff_value * (k1 - k1 ** 2 / 3 + k1 ** 3 / 27)
    assert float(f(0, 1e-4, fz0, fz0, MU)[1]) == pytest.approx(-c_lat * 1e-4, rel=1e-3)
    assert float(f(1e-4, 0, fz0, fz0, MU)[0]) == pytest.approx(car.long_stiff_value * 9.81e-4, rel=1e-3)
    # full slide: mu Fz along the slip, never more
    for kappa, ta in ((1.0, 0.0), (0.0, 1.0), (0.3, 0.3), (0.05, 0.05)):
        fx, fy = (float(q) for q in f(kappa, ta, fz0, fz0, MU))
        assert math.hypot(fx, fy) <= MU * fz0 * (1 + 1e-9)
    fx, fy = (float(q) for q in f(1.0, 1.0, fz0, fz0, MU))
    assert math.hypot(fx, fy) == pytest.approx(MU * fz0, rel=1e-6)


def test_the_tyre_matches_isaac_s_sweeps():
    car = Car.from_spec(SPEC)
    f = tyre_fn(car)
    w, k, t, fz, fx, fy = ISAAC["tyre"].T.astype(float)
    fz0 = np.array(model.rest_loads(car))[w.astype(int)]
    px, py = (np.asarray(q).ravel() for q in f(k, t, fz, fz0, MU))
    assert np.sqrt(np.mean(((px - fx) / fz) ** 2)) < 0.02
    assert np.sqrt(np.mean(((py - fy) / fz) ** 2)) < 0.05


def test_steady_cornering_matches_isaac_s_circle():
    """Steering and sideslip on the 40 m ramp, below the limit (to 0.75 g)."""
    car = dataclasses.replace(Car.from_spec(SPEC), diff="open")
    for V, R, tyre_angle, beta in ISAAC["circle"]:
        if V * V / R > 0.75 * 9.81:
            continue
        s = steady_state(car, MU, speed=V, radius=R)
        assert s["ok"]
        assert math.degrees(s["delta"] - tyre_angle) == pytest.approx(0.0, abs=0.15), V
        assert math.degrees(s["beta"] - beta) == pytest.approx(0.0, abs=0.5), V


def test_steady_step_matches_isaac_s():
    car = dataclasses.replace(Car.from_spec(SPEC), diff="open")
    V, delta, r, ay = ISAAC["step"]
    s = steady_state(car, MU, speed=V, delta=delta)
    assert s["ok"]
    assert s["r"] == pytest.approx(r, rel=0.05)
    assert s["ay"] == pytest.approx(ay, rel=0.06)


def test_a_spool_understeers_in_a_steady_corner():
    """A locked axle's rear wheels are forced to one speed, so they scrub and push the car
    straight: more steering for the same radius than an open differential."""
    spool = Car.from_spec(SPEC)
    open_ = dataclasses.replace(spool, diff="open")
    a, b = (steady_state(c, MU, speed=10.0, radius=40.0) for c in (spool, open_))
    assert a["ok"] and b["ok"]
    assert a["delta"] > b["delta"] + math.radians(0.2)
    assert a["kappa"][2] - a["kappa"][3] == pytest.approx(spool.track / 40.0, rel=0.1)
