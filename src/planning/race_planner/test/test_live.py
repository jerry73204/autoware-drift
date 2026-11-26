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
    # the car follows its last plan while the plan still holds its stop, then stops before
    # the plan's end, on the road and on the line it had
    first = next(r for r in d.log if r["stopping"])
    assert first["s"] > 150.0                                # not at the third failure (~s 90)
    assert st["s"] < d.plan["s"][-1]
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


def test_a_car_slower_than_its_plan_is_never_behind_the_plan_it_drives():
    """A plant with less grip than the planner's map (x0.9): the car runs slower than its plans.
    The window starts where the car will be, its speed's deviation counted, and a plan is
    adopted only once the car has reached its first node: the car is never behind the plan it
    drives by more than a node (plan_at would clamp it there, frozen)."""
    c = Car.from_spec(SPEC)
    k = lambda s: 0.0 if s < 150 else (1 / 60.0 if s < 250 else 0.0)
    tr = road(k, 700.0, ds=4.0, w=5.0)
    plant = Plant(c, SPEC, road(k, 700.0, ds=1.0, w=5.0), mu_scale=0.9)
    plant.reset(s=20.0, speed=10.0)
    d = Driver(c, tr, Tracker(c, tr))
    out = run(d, plant, s_end=350.0, t_max=40.0)
    assert out["result"] == "done", out
    assert min(r["behind"] for r in d.log) >= -d.ds
    assert getattr(d, "behind", 0) == 0


def test_a_first_plan_whose_cold_solve_fails_is_rescued_from_more_starts():
    # issue 001: the cold window's one solve fails; the driver tries G with more iterations
    # (then H, then D) before it gives up, and the car starts
    c = Car.from_spec(SPEC)
    tr = road(lambda s: 0.0 if s < 150 else (1 / 60.0 if s < 250 else 0.0), 700.0, ds=4.0, w=5.0)
    plant = Plant(c, SPEC, road(lambda s: 0.0, 700.0, ds=1.0, w=5.0))
    plant.reset(s=20.0, speed=10.0)
    d = Driver(c, tr, Tracker(c, tr))
    d.cold = _Failing(d.cold)
    d.start(plant.state())
    assert d.plan["ok"]
    assert [x[0] for x in d.first_rescue][:2] == ["G", "G+"]


def test_no_reading_in_a_planned_slide():
    """Issue 004: the car's estimate is read where the car slides more than a grip plan, never
    in a planned slide (the reached acceleration there is not the road's friction)."""
    from race_planner.surface import FrictionEstimate
    c = Car.from_spec(SPEC)
    tr = road(lambda s: 0.0, 300.0, ds=4.0, w=5.0)
    d = Driver(c, tr, Tracker(c, tr), use_estimate=True, estimator=FrictionEstimate())
    n = 20
    ss = 100.0 + 4.0 * np.arange(n + 1)

    def plan(beta_deg):
        X = np.zeros((6, n + 1))
        X[0] = 10.0
        X[1] = 10.0 * math.tan(math.radians(beta_deg))
        U = np.zeros((4, n))
        U[2] = 0.8 * 9.81                 # the plan asks 0.8 g there: near the limit
        return {"s": ss, "X": X, "U": U, "t": np.linspace(0.0, 8.0, n + 1)}

    def state(t, beta_deg):
        return {"t": t, "s": 120.0, "vx": 10.0, "vy": 10.0 * math.tan(math.radians(beta_deg)),
                "mu_ratio": 0.3}

    d.plan = plan(20.0)                               # a planned slide, the car 5 deg wider
    assert d.mu_scale(state(0.0, 25.0)) == 1.0
    d.plan = plan(0.0)                                # a grip plan, the car sliding 5 deg
    assert d.mu_scale(state(0.1, 5.0)) < 0.5
    d.close()


def test_a_cut_solve_warms_the_next():
    """Issue 005: a solve cut at the budget near feasible is the next solve's warm start (never
    adopted); a far one, or a plan, is not."""
    from race_planner.live import cut_iterate
    cut = {"ok": False, "violation": 0.1, "defect": 0.01}
    assert cut_iterate(cut) is cut
    assert cut_iterate({"ok": False, "violation": 0.3, "defect": 0.002}) is None
    assert cut_iterate({"ok": False, "violation": 0.01, "defect": 0.5}) is None
    assert cut_iterate({"ok": True, "violation": 0.0, "defect": 0.0}) is None


def test_a_cut_solve_warms_the_next_unless_its_window_meets_ice():
    """Issue 016: issue 005's cut warm start is withheld where the window meets ice (the rear's
    falling branch), decided by the window, not the course: with ice anywhere on a course
    every window elsewhere restarted from a staler plan each time (the zone chain's loose snow
    after its ice: no plan adopted for 8 s)."""
    import dataclasses
    c = Car.from_spec(SPEC)
    tr = road(lambda s: 0.0, 1200.0, ds=4.0, w=5.0)
    ice = np.asarray(tr.s) >= 900.0
    tr = dataclasses.replace(tr, mu=np.where(ice, 0.265, 1.12), sliding=np.where(ice, 0.7, 0.8))
    d = Driver(c, tr, Tracker(c, tr))
    near = d.node(900.0) - d.warm.N + 2          # its window's last nodes on the ice
    assert d.rear_slip_low is not None            # the course has ice
    assert d.cut_warms(0) and d.cut_warms(near - 2)
    assert not d.cut_warms(near) and not d.cut_warms(d.node(950.0))
    d.close()


def test_cold_retry_replaces_a_failed_warm_solve():
    """cold_retry (FastSlowRR 003 R5): a warm re-solve that fails is solved again from the cold
    guess in a window of its own, and the car drives on its plan, not into the stop."""
    c = Car.from_spec(SPEC)
    tr = road(lambda s: 0.0, 700.0, ds=4.0, w=5.0)
    plant = Plant(c, SPEC, road(lambda s: 0.0, 700.0, ds=1.0, w=5.0))
    plant.reset(s=20.0, speed=10.0)
    d = Driver(c, tr, Tracker(c, tr), cold_retry=400)
    st = plant.state()
    d.start(st)
    d.warm = _Failing(d.warm)
    while st["t"] < 6.0:
        cmd = d.step(st)
        st = plant.step(cmd["accel"], cmd["tire_angle"], dt=0.02)
    retried = [x for x in d.solves[1:] if x["status"].startswith("cold_retry:")]
    assert retried and all(x["ok"] for x in retried), d.solves
    assert d.stopping is None and d.fails == 0


def test_serve_takes_a_track_file_the_rear_axle_and_answers_the_plan_in_the_map_frame(tmp_path):
    """serve's opt-in keys (FastSlowRR 003 R5): the track from arrays, the pose Autoware's
    base_link (the rear axle, moved to the CoM by lr), no clock (det), and the plan's line in
    the map frame (plan_xy) starting where the car's CoM is."""
    import dataclasses
    c = Car.from_spec(SPEC)
    tr = road(lambda s: 0.0, 600.0, ds=2.0, w=5.0)
    arrays = {f.name: getattr(tr, f.name) for f in dataclasses.fields(tr)
              if getattr(tr, f.name) is not None}
    np.savez(tmp_path / "track.npz", **arrays)
    init = {"spec": SPEC, "track_npz": str(tmp_path / "track.npz"), "ds": 4.0,
            "pose": "rear_axle", "det": True, "plan_xy": True, "cold_retry": 400}
    p = subprocess.Popen([sys.executable, "-m", "race_planner.serve"], stdin=subprocess.PIPE,
                         stdout=subprocess.PIPE, text=True)
    try:
        p.stdin.write(json.dumps({"init": init}) + "\n")
        p.stdin.flush()
        assert json.loads(p.stdout.readline())["ready"]
        i = 10
        ps = float(tr.psi[i])
        cx, cy = float(tr.x[i]), float(tr.y[i])                 # the CoM on the line
        state = {"t": 0.0, "x": cx - c.lr * math.cos(ps), "y": cy - c.lr * math.sin(ps),
                 "yaw": ps, "vx": 10.0, "vy": 0.0, "r": 0.0, "delta": 0.0}
        p.stdin.write(json.dumps({"state": state}) + "\n")
        p.stdin.flush()
        out = json.loads(p.stdout.readline())
        assert out["s"] == pytest.approx(float(tr.s[i]), abs=0.5)
        assert abs(out["n"]) < 0.05
        plan = out["plan"]
        assert len(plan["x"]) == len(plan["s"]) == len(plan["t"]) == len(plan["psi"])
        assert math.hypot(plan["x"][0] - cx, plan["y"][0] - cy) < 0.5
        assert plan["psi"][0] == pytest.approx(ps, abs=0.05)
        p.stdin.write(json.dumps({"stop": True}) + "\n")
        p.stdin.flush()
        assert json.loads(p.stdout.readline())["done"]
    finally:
        p.kill()


def test_tracker_cold_retry_solves_again_from_the_plan_before_the_lqr():
    """Tracker(cold_retry=True) (FastSlowRR 003 R5): a QP from the last solution that fails is
    solved again from the plan's guess in the same tick; without it, that tick is the LQR's."""
    c = Car.from_spec(SPEC)
    tr = road(lambda s: 0.0, 700.0, ds=4.0, w=5.0)
    plant = Plant(c, SPEC, road(lambda s: 0.0, 700.0, ds=1.0, w=5.0))
    plant.reset(s=20.0, speed=10.0)
    d = Driver(c, tr, Tracker(c, tr))
    st = plant.state()
    d.start(st)
    outs = {}
    for retry in (False, True):
        t = Tracker(c, tr, cold_retry=retry, budget=1e9)
        real = t._solve

        def warm_fails(x0, state, t=t, real=real):
            w, dw, p, ok = real(x0, state)
            return w, dw, p, ok and t.W is None          # every QP from a last solution fails

        t._solve = warm_fails
        t.step(st, d.plan, 0.0)                           # cold: solves
        outs[retry] = t.step(st, d.plan, 0.0)             # warm: fails, then?
    assert outs[False]["fallback"] and not outs[False]["ok"]
    assert not outs[True]["fallback"] and outs[True]["ok"]
