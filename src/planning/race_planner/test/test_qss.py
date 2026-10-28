"""The quasi-steady-state speed profile: grip, drive, brake, drag and grade limits on a given line."""
import dataclasses
import math
import os

import numpy as np
import pytest

from race_planner.car import Car
from race_planner.qss import speed_profile, braking_points
from race_planner.track import Track

SWB = os.environ.get("SWB", os.path.expanduser("~/repos/carla-scenario-bridge"))
SPEC = os.path.join(SWB, "src/sim_vehicle_spec/vehicles/drift_mustang.yaml")
G = 9.81


def line(kappa, s_end=400.0, ds=1.0, mu=1.0, grade=0.0):
    s = np.arange(0.0, s_end + ds / 2, ds)
    k = np.vectorize(kappa)(s).astype(float)
    psi = np.concatenate([[0.0], np.cumsum(0.5 * (k[1:] + k[:-1]) * ds)])
    one = np.ones_like(s)
    return Track(s=s, x=np.cumsum(np.cos(psi)) * ds, y=np.cumsum(np.sin(psi)) * ds,
                 z=grade * s, psi=psi, kappa=k, w_left=4 * one, w_right=4 * one, mu=mu * one,
                 closed=False, grade=grade * one)


def no_drag(car):
    return dataclasses.replace(car, drag=0.0, roll=0.0)


def test_a_constant_radius_holds_the_grip_limit():
    car = no_drag(Car.from_spec(SPEC))
    for mu, r in ((1.0, 20.0), (1.0, 100.0), (0.3, 50.0)):
        t = line(lambda s: 1.0 / r, mu=mu)
        v = speed_profile(t, car, v_start=None, v_end=None)
        mid = v[len(v) // 3: 2 * len(v) // 3]
        assert np.allclose(mid, math.sqrt(mu * G * r), rtol=0.01), (mu, r, mid.min(), mid.max())


def test_braking_into_a_hairpin_on_the_friction_limit():
    """30 m/s on a straight into an R 10 m bend at 300 m: the brake starts where a friction-limited
    stop from 30 m/s to the bend's speed ends at the bend, and grade moves it."""
    car = no_drag(Car.from_spec(SPEC))
    kappa = lambda s: 0.1 if s >= 300.0 else 0.0  # noqa: E731
    v_bend = math.sqrt(1.0 * G * 10.0)
    for grade, a_brake in ((0.0, G), (-0.08, G * math.cos(math.atan(0.08)) - G * math.sin(math.atan(0.08)))):
        t = line(kappa, s_end=360.0, grade=grade)
        v = speed_profile(t, car, v_start=30.0, v_end=None, v_max=30.0)
        d = (30.0 ** 2 - v_bend ** 2) / (2 * a_brake)
        b = braking_points(t, v)
        assert len(b) == 1
        assert b[0]["s"] == pytest.approx(300.0 - d, abs=3.0), (grade, b, 300 - d)
        assert b[0]["v_from"] == pytest.approx(30.0, abs=0.5)
        assert b[0]["v_to"] == pytest.approx(v_bend, rel=0.03)


def test_drive_is_limited_by_the_rear_axle_and_the_motor():
    """From standstill on a flat straight: the start is traction-limited (the rear axle's load
    times mu, with the load transfer), then the motor's torque, then its power."""
    car = no_drag(Car.from_spec(SPEC))
    t = line(lambda s: 0.0, s_end=400.0)
    v = speed_profile(t, car, v_start=0.0, v_end=None, v_max=80.0)
    a0 = (v[2] ** 2 - v[1] ** 2) / (2 * (t.s[2] - t.s[1]))
    # rear load with transfer: Fz_r = m g lr/L + m a h/L; drive = mu Fz_r -> a = mu g lr/(L - mu h)
    a_trac = G * car.lf / (car.wheel_base - car.h_cg) * car.mu
    a_motor = car.axle_torque_max / car.r_wheel / car.m
    assert a0 == pytest.approx(min(a_trac, a_motor), rel=0.05), (a0, a_trac, a_motor)
    i = np.searchsorted(v, 40.0)
    a40 = (v[i + 1] ** 2 - v[i] ** 2) / (2 * (t.s[i + 1] - t.s[i]))
    assert a40 == pytest.approx(car.power_max / (car.m * v[i]), rel=0.05)   # power-limited


def test_drag_sets_a_top_speed_on_power():
    car = Car.from_spec(SPEC)
    t = line(lambda s: 0.0, s_end=6000.0, ds=5.0)
    v = speed_profile(t, car, v_start=0.0, v_end=None, v_max=200.0)
    # P = (drag v^2 + roll m g) v at the top speed
    f = lambda u: (car.drag * u * u + car.roll * car.m * G) * u - car.power_max  # noqa: E731
    lo, hi = 10.0, 150.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if f(mid) < 0 else (lo, mid)
    assert v[-1] == pytest.approx(lo, rel=0.02)
