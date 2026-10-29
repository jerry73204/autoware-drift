"""L1's window planner (window.py) on axle.py's car: what the car's inputs make, against the
two-track model on the same road, and re-solving along a road from its own last plan."""
import dataclasses
import math

import casadi as ca
import numpy as np
import pytest

from race_planner import axle
from race_planner.car import Car
from race_planner.ocp import solve_min_time
from race_planner.window import Window, initial
from test_ocp import SPEC, road

MU = 1.12


def car():
    return Car.from_spec(SPEC)


def axle_fn(c):
    x, u, acc = ca.SX.sym("x", 3), ca.SX.sym("u", 2), ca.SX.sym("acc", 2)
    o = axle.dynamics(c, [x[0], x[1], x[2]], [u[0], u[1]], MU, [acc[0], acc[1]])
    return ca.Function("a", [x, u, acc], [ca.vertcat(*o["acc"]), o["drive"], o["brake"],
                                         ca.vertcat(*o["fx"])])


def test_the_request_is_what_the_ecu_makes_of_it():
    c = car()
    f = axle_fn(c)
    acc, drive, brake, fx = f([20, 0, 0], [0, 0.0], [0, 0])     # hold speed: the road's load
    assert float(acc[0]) == pytest.approx(0.0, abs=0.02)
    acc, drive, brake, fx = f([20, 0, 0], [0, -6.0], [-6, 0])   # brake: 50:50, never driving
    fx = np.asarray(fx).ravel()
    assert float(acc[0]) == pytest.approx(-6.0, abs=0.05)
    assert float(drive) < 30.0
    assert fx[0] == pytest.approx(fx[1]) and (fx[0] + fx[1]) == pytest.approx(fx[2] + fx[3], rel=0.02)
    acc, drive, brake, fx = f([10, 0, 0], [0, 3.0], [3, 0])     # drive: the rear only
    fx = np.asarray(fx).ravel()
    assert float(brake) < 30.0 and abs(fx[0]) < 15 and fx[2] > 0


def test_a_window_through_a_bend_is_close_to_the_two_track_plan():
    """The axle car loses the wheels' own slip control, so it may be a little slower than the
    two-track car, never much, and never faster than it by more than the models differ."""
    c = car()
    t = road(lambda s: 1.0 / 20.0 if 80.0 <= s < 80.0 + math.pi * 20.0 else 0.0, 200.0, ds=4.0)
    w = Window(c, ds=4.0, length=200.0, warm=False)
    X, U = initial(t, c, v_start=15.0)
    x0 = X[:, 0].copy(); x0[0] = 15.0
    plan = w.solve(t, 0, x0, 60.0, X, U)
    assert plan["ok"], plan["status"]
    assert plan["defect"] < 1e-3
    ref = solve_min_time(t, c, v_start=15.0, grip=0.92)
    assert plan["time"] == pytest.approx(ref["time"], rel=0.06)


def test_re_solving_from_the_last_plan_is_feasible_within_its_budget():
    """Live, a solve is cut at its budget; what it returns must still be a plan the car can
    drive (feasible), which the next solve then improves."""
    c = car()
    t = road(lambda s: (1.0 / 25.0) * math.sin(2 * math.pi * s / 300.0), 600.0, ds=4.0)
    cold = Window(c, ds=4.0, length=200.0, warm=False)
    w = Window(c, ds=4.0, length=200.0, budget=1.5)
    X, U = initial(w.section(t, 0), c, v_start=15.0)
    x0 = X[:, 0].copy(); x0[0] = 15.0
    plan = cold.solve(t, 0, x0, 60.0, X, U)          # the first plan: IPOPT's own start
    assert plan["ok"], plan["status"]
    w.lam = cold.lam                                  # then warm, multipliers and all
    i, misses, worst_miss_run = 0, 0, 0
    for step in range(8):
        j = i - plan["i0"]                            # where the car is on the plan it drives
        j += max(1, int(np.searchsorted(plan["t"] - plan["t"][j], 1.0)))   # a second on
        i = plan["i0"] + j
        Xw, Uw = w.warm(plan, j, t)
        new = w.solve(t, i, plan["X"][:, j], 60.0, Xw, Uw)
        if step > 0:                                  # the first warm call builds the solver
            assert new["solve_s"] < 1.5 + 0.3
        if new["ok"]:
            plan, misses = new, 0
        else:                                         # the car keeps the last plan
            misses += 1
        worst_miss_run = max(worst_miss_run, misses)
    assert worst_miss_run <= 1
