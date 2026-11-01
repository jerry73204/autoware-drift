"""A forced drift (forced.py, roadmap 065 M2): the floor rows hold the rear past its peak on the
stretch asked, cost time where a drift does not pay, and open fully they are the free window."""
import dataclasses
import math

import numpy as np
import pytest

from race_planner import forced, modes
from race_planner.car import Car
from race_planner.window import Window, initial
from test_ocp import SPEC, road

R, A = 25.0, 80.0                     # a left 90 deg bend from s 80, asphalt (no drift pays)


def _setup():
    c = Car.from_spec(SPEC)
    b = A + 0.5 * math.pi * R
    t = road(lambda s: 1.0 / R if A <= s < b else 0.0, b + 160.0, ds=4.0, w=4.0)
    t = dataclasses.replace(t, sliding=np.full_like(t.s, 0.8))   # asphalt's fall-off
    peak = modes.rear_peak(c, t.mu[0], 0.8)
    return c, t, b, peak


def test_refusals():
    with pytest.raises(ValueError, match="^'angle'"):
        forced.floor_kwargs(0.0)
    c = Car.from_spec(SPEC)
    w = Window(c, ds=4.0, length=40.0, warm=False, compile=False)
    with pytest.raises(ValueError, match="^'window'"):
        forced.set_floor(w, np.arange(11) * 4.0, 0.0, 10.0, 1)


def test_angle_for_rounds_up_to_a_degree():
    assert forced.angle_for(math.radians(12.1), 3.0) == math.radians(16.0)
    assert forced.angle_for(math.radians(12.0), 3.0) == math.radians(15.0)


def test_the_floor_holds_on_its_stretch_and_costs_time_and_opens_to_free():
    c, t, b, peak = _setup()
    v0 = 20.0
    x0 = np.array([v0, 0, 0, 0, 0, 0.0])
    L = float(t.s[-1] - 4.0)
    free = Window(c, ds=4.0, length=L, grip=1.0, warm=False, tol=1e-6, max_iter=3000,
                  compile=False)
    angle = forced.angle_for(peak, 3.0)
    fw = Window(c, ds=4.0, length=L, grip=1.0, warm=False, tol=1e-6, max_iter=3000,
                compile=False, **forced.floor_kwargs(angle))
    sec = free.section(t, 0)
    X0, U0 = initial(sec, c, free.margin, v0)
    X0[:, 0] = x0
    pf = free.solve(t, 0, x0, 1e3, X0, U0)
    assert pf["ok"]
    a, apex = A, A + 0.25 * math.pi * R
    seed = modes.seed_rear_out(pf["X"], pf["U"], sec, A, b, c, peak, t.mu[0], 1.0)
    forced.set_floor(fw, sec.s, a, apex, 1)
    pd = fw.solve(t, 0, x0, 1e3, *seed)
    assert pd["ok"], pd["status"]
    assert forced.floor_met(c, pd["X"], pd["s"], a, apex, 1, angle)
    assert not forced.floor_met(c, pf["X"], pf["s"], a, apex, 1, angle)
    assert pd["time"] > pf["time"] + 1e-3            # no drift pays here: the floor costs
    forced.set_floor(fw, sec.s, 1.0, 0.0, 1)           # open: the free window's plan
    fw.lam = fw.lam_x = None
    po = fw.solve(t, 0, x0, 1e3, X0, U0)
    assert po["ok"] and po["time"] == pytest.approx(pf["time"], abs=2e-3)
