"""The two-track model against PhysX's tyre and against Isaac's drift_mustang.

test/data/isaac_drift_mustang_dry.npz is from swb_isaac_world's manoeuvre suite (spikes/
manoeuvres.py, EGO_RIG=drift_mustang.rig.yaml, PADS=dry, 60 Hz, 2026-10-29 on 243): `tyre`, 800
wheel-frames of the lateral and longitudinal sweeps (wheel, kappa, tan alpha, Fz, Fx, Fy; PhysX's
slips); `circle`, the 40 m constant-radius ramp every 7.5 s (CoM speed, radius, tyre angle, CoM
sideslip; Isaac logs its sideslip at the rear axle, moved here by r lr); `step`, the 5 deg step at
40 km/h (speed, angle, steady yaw rate, steady ay). The tyre rows are from the first run
(play_log/model_check/dry1); the circle and the step from dry3, with sim_vehicle_ecu d68637a,
whose locked axle holds as a spool (swb docs/issues/022), and aib abaf103, whose tyre forces act
at the contact patch (swb docs/issues/023), so they check the spec's own car.
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
    """With asphalt's sliding share (0.8) past the peak, as swb's surface layer writes it; the
    sweeps' far side is closer still at 0.9, the layer's 0.5 m lag of the drop, which a steady
    model leaves out."""
    car = Car.from_spec(SPEC)
    s = ca.SX.sym("s", 6)
    f = ca.Function("tyre", [s], list(model.tyre(car, *(s[i] for i in range(6)))))
    w, k, t, fz, fx, fy = ISAAC["tyre"].T.astype(float)
    fz0 = np.array(model.rest_loads(car))[w.astype(int)]
    px, py = (np.asarray(q).ravel() for q in f(np.vstack([k, t, fz, fz0, MU + 0 * k, 0.8 + 0 * k])))
    ex, ey = (px - fx) / fz, (py - fy) / fz
    assert np.sqrt(np.mean(ex ** 2)) < 0.02 and np.sqrt(np.mean(ey ** 2)) < 0.04
    kp, tp = model.peaks(car)
    past = np.hypot(k / kp, t / tp) > 1.5
    assert past.sum() > 50
    assert np.sqrt(np.mean(ex[past] ** 2)) < 0.04 and np.sqrt(np.mean(ey[past] ** 2)) < 0.05


def test_past_the_peak_the_tyre_falls_to_the_sliding_share():
    car = Car.from_spec(SPEC)
    f = tyre_fn(car)
    fz0 = model.rest_loads(car)[0]
    s = ca.SX.sym("s", 6)
    g = ca.Function("tyre", [s], list(model.tyre(car, *(s[i] for i in range(6)))))
    fx, fy = (float(q) for q in g(ca.vertcat(5.0, 0.0, fz0, fz0, MU, 0.8)))
    assert math.hypot(fx, fy) == pytest.approx(MU * fz0 * (0.8 + 0.2 / (5.0 / model.peaks(car)[0])), rel=0.01)
    assert float(g(ca.vertcat(0.05, 0.0, fz0, fz0, MU, 0.8))[0]) == pytest.approx(
        float(f(0.05, 0.0, fz0, fz0, MU)[0]), rel=1e-3)     # below the peak: untouched


def test_steady_cornering_matches_isaac_s_circle():
    """Steering and sideslip on the 40 m ramp, with asphalt's sliding share, to 0.76 g: Isaac's
    car holds 0.75 g (dry3's ay_max_held); its last two rows, 0.85 and 0.93 g, are on the way out
    of the circle, not steady (there the model steers 0.1 and 0.4 deg less)."""
    car = Car.from_spec(SPEC)
    for V, R, tyre_angle, beta in ISAAC["circle"]:
        if V * V / R > 0.8 * 9.81:
            continue
        s = steady_state(car, MU, speed=V, radius=R, sliding=0.8)
        assert s["ok"]
        assert math.degrees(s["delta"] - tyre_angle) == pytest.approx(0.0, abs=0.1), V
        assert math.degrees(s["beta"] - beta) == pytest.approx(0.0, abs=0.7), V


def test_load_transfer_is_a_real_car_s():
    """Isaac's sweeps (play_log/model_check/dry3, aib abaf103): 221 N per m/s^2 per axle
    sideways and 513 front to rear; m h / t / 2 = 219 and 2 m h / L = 482."""
    car = Car.from_spec(SPEC)
    lat = (model.loads(car, 0.0, 1.0)[1] - model.loads(car, 0.0, 1.0)[0]) / 2
    lon = sum(model.loads(car, -1.0, 0.0)[:2]) - sum(model.loads(car, -1.0, 0.0)[2:])
    assert float(lat) == pytest.approx(221.1, rel=0.02)
    assert float(lon) == pytest.approx(513.1, rel=0.07)


def test_steady_step_matches_isaac_s():
    car = Car.from_spec(SPEC)
    V, delta, r, ay = ISAAC["step"]
    s = steady_state(car, MU, speed=V, delta=delta)
    assert s["ok"]
    assert s["r"] == pytest.approx(r, rel=0.02)
    assert s["ay"] == pytest.approx(ay, rel=0.03)


def test_a_spool_understeers_in_a_steady_corner():
    """A locked axle's rear wheels are forced to one speed, so they scrub and push the car
    straight: more steering for the same radius than an open differential. Their slips differ
    by t/R in PhysX's slip, v / (|v| + 4) of the kinematic t/R."""
    spool = Car.from_spec(SPEC)
    open_ = dataclasses.replace(spool, diff="open")
    a, b = (steady_state(c, MU, speed=10.0, radius=40.0) for c in (spool, open_))
    assert a["ok"] and b["ok"]
    assert a["delta"] > b["delta"] + math.radians(0.2)
    assert a["kappa"][2] - a["kappa"][3] == pytest.approx(
        spool.track / 40.0 * 10.0 / (10.0 + model.SLIP_LONG_OFFSET), rel=0.1)
