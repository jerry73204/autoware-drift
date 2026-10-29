"""The car's own limits in the plan (SR3 of the live planner's design): its top speed (the motor's
rpm through its gear), its braking (friction brakes and the motor's regeneration) and its
steering rate. Each from the spec, each checked on a road that pushes the plan against it."""
import dataclasses
import math

import numpy as np
import pytest

from race_planner.car import Car
from race_planner.ocp import solve_min_time
from test_ocp import SPEC, road


def test_the_spec_s_limits_are_read():
    c = Car.from_spec(SPEC)
    # 16000 rpm through 6:1 on a 0.32 m wheel
    assert c.wheel_speed_max == pytest.approx(16000 * 2 * math.pi / 60 / 6.0 * c.r_wheel, rel=1e-6)
    assert c.regen_torque_max == pytest.approx(150.0 * 6.0)     # at the axle
    assert c.regen_power_max == pytest.approx(60000.0)
    assert c.steer_rate_max is None                             # drift_mustang's spec: none


def test_the_rear_wheels_never_pass_the_motor_s_top_speed():
    c = dataclasses.replace(Car.from_spec(SPEC), wheel_speed_max=20.0)
    t = road(lambda s: 0.0, 400.0, ds=2.0)
    sol = solve_min_time(t, c, v_start=5.0)
    assert sol["ok"], sol["status"]
    assert sol["w"][:, 2:].max() <= 20.0 + 1e-3
    assert sol["vx"].max() <= 20.5
    assert sol["vx"][-1] > 19.0                                 # and it does get there


def test_braking_stays_within_the_brakes_and_the_regeneration():
    tb = 400.0                                                  # weak brakes: they bind, not grip
    c = dataclasses.replace(Car.from_spec(SPEC), brake_torque_max=tb, diff="open")
    r = 15.0
    t = road(lambda s: 1.0 / r if s >= 200.0 else 0.0, 260.0, ds=2.0)
    sol = solve_min_time(t, c, v_start=25.0)
    assert sol["ok"], sol["status"]
    T = sol["torque"]
    assert T[:, :2].min() >= -tb - 1.0                           # each front wheel
    w_rear = 0.5 * (sol["w"][:, 2] + sol["w"][:, 3]) / c.r_wheel
    regen = np.minimum(c.regen_torque_max, c.regen_power_max / np.maximum(w_rear, 1.0))
    assert np.all(T[:, 2] + T[:, 3] >= -(2 * tb + regen) - 1.0)  # the rear axle
    assert (T[:, :2].sum(axis=1) < -0.9 * 2 * tb).any()          # the brakes were the limit


def test_steering_moves_no_faster_than_the_spec_allows():
    rate = 0.3
    c = dataclasses.replace(Car.from_spec(SPEC), steer_rate_max=rate)
    t = road(lambda s: 0.05 * math.sin(2 * math.pi * s / 60.0), 240.0, ds=2.0)
    sol = solve_min_time(t, c, v_start=10.0)
    assert sol["ok"], sol["status"]
    d_delta = np.diff(sol["delta"]) / np.maximum(np.diff(sol["t"]), 1e-6)
    assert np.abs(d_delta).max() <= rate + 0.02
