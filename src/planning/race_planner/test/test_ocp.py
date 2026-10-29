"""The minimum-time problem on the two-track model: it finds a feasible trajectory inside the
road, close to the quasi-steady baseline's time on the minimum-curvature line (the baseline is a
point mass with the whole friction circle, so the two-track car, losing grip to load transfer, is
somewhat slower), never faster through a corner than the car's steady limit for the radius it
drives, and on a steady circle at that limit."""
import dataclasses
import math
import os

import numpy as np
import pytest

from race_planner.car import Car
from race_planner.line import min_curvature_line
from race_planner.ocp import solve_min_time
from race_planner.qss import lap_time, speed_profile
from race_planner.steady import steady_state
from race_planner.track import SurfaceTable, Track

SWB = os.environ.get("SWB", os.path.expanduser("~/repos/carla-scenario-bridge"))
SPEC = os.path.join(SWB, "src/sim_vehicle_spec/vehicles/drift_mustang.yaml")
CLASSES = os.path.join(SWB, "src/swb_isaac_world/config/surface_classes.yaml")
HARUNA = os.path.join(SWB, "data/courses/haruna")
MU = 1.12


def road(kappa, s_end, ds=1.0, w=5.0, mu=MU, closed=False):
    s = np.arange(0.0, s_end + ds / 2, ds)
    k = np.vectorize(kappa)(s).astype(float)
    psi = np.concatenate([[0.0], np.cumsum(0.5 * (k[1:] + k[:-1]) * ds)])
    x = np.concatenate([[0.0], np.cumsum(0.5 * (np.cos(psi[1:]) + np.cos(psi[:-1])) * ds)])
    y = np.concatenate([[0.0], np.cumsum(0.5 * (np.sin(psi[1:]) + np.sin(psi[:-1])) * ds)])
    one = np.ones_like(s)
    return Track(s=s, x=x, y=y, z=0 * s, psi=psi, kappa=k, w_left=w * one, w_right=w * one,
                 mu=mu * one, closed=closed)


def car(diff="open"):
    return dataclasses.replace(Car.from_spec(SPEC), diff=diff)


def check_feasible(t, sol, c, margin=0.3):
    half = 0.5 * c.width
    assert sol["ok"], sol["status"]
    assert np.all(sol["n"] <= t.w_left - half - margin + 1e-3)
    assert np.all(sol["n"] >= -(t.w_right - half - margin) - 1e-3)
    assert np.all(np.abs(sol["delta"]) <= c.steer_max + 1e-6)
    axle = sol["torque"][:, 2] + sol["torque"][:, 3]
    assert np.all(axle <= c.axle_torque_max + 1.0)
    assert np.all(sol["torque"][:, :2] <= 1.0)                       # the front only brakes
    assert np.all(sol["fz"] >= -1.0)


def path_radius(sol):
    x, y = sol["x"], sol["y"]
    heading = np.unwrap(np.arctan2(np.gradient(y), np.gradient(x)))
    return 1.0 / np.maximum(np.abs(np.gradient(heading) / np.hypot(np.gradient(x), np.gradient(y))), 1e-6)


def test_a_bend_is_near_the_baseline_and_within_the_car_s_limit():
    r, a, b = 20.0, 120.0, 120.0 + 0.5 * math.pi * 20.0
    t = road(lambda s: 1.0 / r if a <= s < b else 0.0, b + 100.0).resampled(2.0)
    c = car()
    sol = solve_min_time(t, c, v_start=25.0)
    check_feasible(t, sol, c)
    line, _ = min_curvature_line(t, 0.5 * c.width, 0.3)
    qss = lap_time(line, speed_profile(line, c, v_start=25.0))
    assert qss * 0.95 < sol["time"] < qss * 1.10, (sol["time"], qss)
    v = np.hypot(sol["vx"], sol["vy"])
    i = int(np.argmin(v))
    assert a - 10.0 < t.s[i] < b + 40.0                              # slowest in the bend
    limit = steady_state(c, MU, radius=path_radius(sol)[i], maximise_speed=True)["speed"]
    assert v[i] <= limit * 1.02
    assert t.s[int(np.argmax(v[: i + 1]))] < a                       # braking before it


def test_a_circle_runs_at_the_steady_limit():
    """A closed circle of R 40 with a 3 m road: the time is the circumference at the steady
    state's most speed, near the outer edge."""
    r = 40.0
    t = road(lambda s: 1.0 / r, 2 * math.pi * r - 2.0, ds=2.0, w=3.0, closed=True)
    c = car()
    sol = solve_min_time(t, c)
    check_feasible(t, sol, c)
    v = np.mean(np.hypot(sol["vx"], sol["vy"]))
    radius = r - np.mean(sol["n"])
    best = steady_state(c, MU, radius=radius, maximise_speed=True)
    assert v == pytest.approx(best["speed"], rel=0.03)


@pytest.mark.skipif(not os.path.isdir(HARUNA), reason="Haruna's course data is not here")
def test_haruna_section_solves():
    c = car("locked")
    t = Track.from_swb_course(HARUNA, SurfaceTable.from_yaml(CLASSES), c).resampled(2.0)
    t = dataclasses.replace(t, **{k: getattr(t, k)[:200] for k in
                                  ("s", "x", "y", "z", "psi", "kappa", "w_left", "w_right",
                                   "mu", "grade", "sliding")})
    sol = solve_min_time(t, c, v_start=15.0)
    check_feasible(t, sol, c)


def test_the_plan_brakes_with_the_car_s_own_bias():
    """drift_mustang's brakes are static (50:50): wherever the plan brakes, the front pair takes
    half of it, alike, and the motor does not drive at the same time."""
    r, a, b = 20.0, 120.0, 120.0 + 0.5 * math.pi * 20.0
    t = road(lambda s: 1.0 / r if a <= s < b else 0.0, b + 60.0).resampled(2.0)
    c = Car.from_spec(SPEC)
    sol = solve_min_time(t, c, v_start=25.0)
    check_feasible(t, sol, c)
    T = sol["torque"]
    braking = (T[:, 0] + T[:, 1]) < -200.0
    assert braking.sum() > 5
    front = T[braking, 0] + T[braking, 1]
    assert np.allclose(front / T[braking].sum(axis=1), 0.5, atol=0.02)
    assert np.allclose(T[braking, 0], T[braking, 1], atol=5.0)
    # the motor's drive under the brakes: the rear's torque beyond its brake share
    drive = T[braking, 2] + T[braking, 3] - (T[braking, 0] + T[braking, 1])
    assert drive.max() < 50.0
