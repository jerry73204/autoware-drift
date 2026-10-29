"""The model-in-the-loop plant (sim.py): the two-track car behind sim_vehicle_ecu, stepped in
time. What a request does to it is what the ECU makes of it on that car."""
import math

import numpy as np
import pytest

from race_planner.car import Car
from race_planner.sim import Plant
from test_ocp import SPEC, road


def plant(**kw):
    c = Car.from_spec(SPEC)
    return Plant(c, SPEC, road(lambda s: 0.0, 800.0, ds=1.0, w=10.0), **kw)


def test_time_advances_by_the_step_asked():
    p = plant()
    p.reset(speed=5.0)
    for _ in range(60):
        st = p.step(0.0, 0.0, dt=0.02)
    assert st["t"] == pytest.approx(1.2, abs=1.0 / 60)


def test_a_request_is_carried_out_within_the_car_s_grip():
    p = plant()
    p.reset(speed=10.0)
    for _ in range(50):
        st = p.step(3.0, 0.0, dt=0.02)
    assert st["vx"] == pytest.approx(13.0, abs=0.6)             # 3 m/s^2 for 1 s, the ECU's lag
    v = st["vx"]
    for _ in range(50):
        st = p.step(-6.0, 0.0, velocity=0.0, dt=0.02)
    assert st["vx"] - v == pytest.approx(-6.0, abs=0.8)          # dead time and lag cost a little


def test_a_steady_turn_has_the_kinematic_yaw_rate_at_low_speed():
    p = plant()
    p.reset(speed=4.0)
    delta = math.radians(3.0)
    for _ in range(150):
        st = p.step(0.0, delta, dt=0.02)
    v = math.hypot(st["vx"], st["vy"])
    assert st["r"] == pytest.approx(v * math.tan(delta) / 2.886, rel=0.1)


def test_a_wrong_plant_grips_less():
    lo, hi = plant(mu_scale=0.7), plant()
    out = []
    for p in (lo, hi):
        p.reset(speed=20.0)
        for _ in range(100):
            st = p.step(0.0, math.radians(8.0), dt=0.02)
        out.append(abs(st["ay"]))
    assert out[0] < out[1]
