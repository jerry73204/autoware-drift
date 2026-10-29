"""The live loop (live.py) and its process (serve.py): L1 re-planning, L2 tracking, the car's
place on the track found from its pose alone."""
import json
import math
import subprocess
import sys

import numpy as np
import pytest

from race_planner.car import Car
from race_planner.live import Driver, run
from race_planner.serve import Frenet
from race_planner.sim import Plant
from race_planner.tracker import Tracker
from test_ocp import SPEC, road


def test_frenet_finds_the_offset_and_the_heading_off_the_line():
    t = road(lambda s: 0.01, 300.0, ds=1.0, w=6.0)          # a 100 m circle's arc
    f = Frenet(t)
    i = 120
    ps = float(t.psi[i])
    x = t.x[i] - 1.5 * math.sin(ps)                          # 1.5 m left of node 120
    y = t.y[i] + 1.5 * math.cos(ps)
    s, n, xi = f(x, y, ps + 0.1)
    assert s == pytest.approx(t.s[i], abs=0.1)
    assert n == pytest.approx(1.5, abs=0.02)
    assert xi == pytest.approx(0.1, abs=1e-9)
    s2, _, _ = f(t.x[i + 5], t.y[i + 5], ps)                 # searched on from the last node
    assert s2 == pytest.approx(t.s[i + 5], abs=0.1)


def test_the_live_loop_drives_a_bend_on_the_road_with_l2():
    c = Car.from_spec(SPEC)
    k = lambda s: 0.0 if s < 150 else (1 / 60.0 if s < 250 else 0.0)
    tr = road(k, 700.0, ds=4.0, w=5.0)
    plant = Plant(c, SPEC, road(k, 700.0, ds=1.0, w=5.0))
    plant.reset(s=20.0, speed=10.0)
    d = Driver(c, tr, Tracker(c, tr))
    out = run(d, plant, s_end=350.0, t_max=40.0)
    assert out["result"] == "done", out
    assert out["solves_ok"] >= 0.8 * out["solves"], out
    assert max(r["v"] for r in d.log) > 15.0                # it does not crawl


def test_serve_answers_each_state_with_a_command():
    p = subprocess.Popen([sys.executable, "-m", "race_planner.serve"], stdin=subprocess.PIPE,
                         stdout=subprocess.PIPE, text=True)
    try:
        p.stdin.write(json.dumps({"bad": 1}) + "\n")
        p.stdin.flush()
        p.stdin.close()
        assert p.wait(timeout=60) != 0                       # no init: it refuses, it does not hang
    finally:
        p.kill()


class _Failing:
    """A window solver that never finds a plan."""

    def __init__(self, w):
        self.w = w

    def __getattr__(self, k):
        return getattr(self.w, k)

    def solve(self, *a, **kw):
        out = self.w.solve(*a, **kw)
        return dict(out, ok=False, status="forced")


def test_three_failed_solves_in_a_row_stop_the_car():
    c = Car.from_spec(SPEC)
    tr = road(lambda s: 0.0, 700.0, ds=4.0, w=5.0)
    plant = Plant(c, SPEC, road(lambda s: 0.0, 700.0, ds=1.0, w=5.0))
    plant.reset(s=20.0, speed=10.0)
    d = Driver(c, tr, Tracker(c, tr))
    st = plant.state()
    d.start(st)
    d.warm = _Failing(d.warm)
    out = None
    while st["t"] < 30.0:
        cmd = d.step(st)
        if d.stopping and math.hypot(st["vx"], st["vy"]) < 0.5:
            out = d.stopping
            break
        st = plant.step(cmd["accel"], cmd["tire_angle"], dt=0.02)
    assert out == "solves failed"
    assert st["t"] < 11.0              # 3 timer solves (~4 s), then braking from ~32 m/s at 6 m/s^2
    assert abs(st["n"]) < 5.0 - 0.5 * c.width                # stopped on the road
    assert abs(st["n"] - d.log[-1]["n_plan"]) < 0.3          # on the line it had


def test_the_live_loop_drives_through_the_finish():
    c = Car.from_spec(SPEC)
    tr = road(lambda s: 0.0, 400.0, ds=4.0, w=5.0)
    plant = Plant(c, SPEC, road(lambda s: 0.0, 400.0, ds=1.0, w=5.0).extended(400.0))
    plant.reset(s=150.0, speed=20.0)
    d = Driver(c, tr, Tracker(c, tr))
    out = run(d, plant, s_end=420.0, t_max=30.0)          # 20 m past the finish
    assert out["result"] == "done", out


def test_on_a_circuit_the_car_is_found_on_the_first_lap():
    t = road(lambda s: 0.01, 2 * math.pi * 100 - 1.0, ds=1.0, w=6.0)
    t = __import__("dataclasses").replace(t, closed=True)
    u = t.unrolled(3).resampled(4.0)
    lap = t.length + 1.0
    f = Frenet(u, int(lap / 4.0))
    s, n, _ = f(float(t.x[50]), float(t.y[50]), float(t.psi[50]))
    assert s == pytest.approx(50.0, abs=2.0)
