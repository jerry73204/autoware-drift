"""Grip or drift chosen live (multistart.py, roadmap 061 M3): the hysteresis gate on its own, then
live.Driver with its seeds on a slippery hairpin and on an asphalt bend, and the default (one
start, 060's) unchanged."""
import math
import multiprocessing

import numpy as np

from race_planner.car import Car
from race_planner.live import Driver
from race_planner.multistart import MultiStart
from race_planner.sim import NoTracker, PlanFollower
from test_ocp import SPEC, road


def chooser(threshold=0.003, confirm=2, ahead=25.0):
    m = MultiStart.__new__(MultiStart)          # the gate alone: no seed processes
    m.threshold, m.confirm, m.ahead, m.mode, m.count = threshold, confirm, ahead, "grip", 0
    return m


def results(grip, drift, onset=40.0):
    r = {"G": {"ok": True, "time": grip, "mode": "grip", "onset": None}}
    if drift is not None:
        r["D"] = {"ok": True, "time": drift, "mode": "drift", "onset": onset}
    return r


def test_drift_is_chosen_when_faster_beyond_the_threshold_on_two_replans_in_a_row():
    m = chooser()
    assert m.choose(results(20.0, 19.9)) == "G"           # 0.5 % faster: once is not enough
    assert m.choose(results(20.0, 19.9)) == "D"           # twice is
    assert m.mode == "drift"
    m = chooser()
    assert m.choose(results(20.0, 19.9)) == "G"
    assert m.choose(results(20.0, 19.99)) == "G"          # the run broken: counted again
    assert m.choose(results(20.0, 19.9)) == "G"
    assert m.choose(results(20.0, 19.9)) == "D"


def test_a_tie_stays_grip():
    m = chooser()
    for _ in range(5):                                     # 0.2 % < 0.3 %: a tie
        assert m.choose(results(20.0, 19.96)) == "G"
    assert m.mode == "grip"


def test_a_drift_whose_slide_starts_too_near_is_not_taken():
    m = chooser()
    for _ in range(3):
        assert m.choose(results(20.0, 19.0, onset=12.0)) == "G"


def test_back_to_grip_by_the_same_gate_or_when_the_slide_is_over():
    m = chooser()
    m.choose(results(20.0, 19.9))
    m.choose(results(20.0, 19.9))
    assert m.mode == "drift"
    assert m.choose(results(19.95, 20.0)) == "D"           # grip faster within the threshold
    assert m.choose(results(19.9, 20.0)) == "D"            # beyond it, once
    assert m.choose(results(19.9, 20.0)) == "G"            # twice
    m.mode = "drift"
    assert m.choose(results(19.9, None)) == "G"            # no drift plan left: grip at once
    assert m.mode == "grip"


def drive(track, multistart, s_end, v0=12.0):
    """The driver with its seeds along `track` on the plans it adopts. Each seed is given 3 s
    rather than 0.9 so that a loaded host cannot cut a seed's solve short and so change the
    choice; a slow solve only delays its adoption."""
    multistart = dict(multistart, seed_budget=3.0)
    c = Car.from_spec(SPEC)
    d = Driver(c, track, NoTracker(), multistart=multistart)
    f = PlanFollower(d, 10.0, v0)
    st = f.state()
    try:
        d.start(st)
        while st["s"] < s_end and st["t"] < 120.0 and d.stopping is None:
            d.step(st)
            st = f.step(0.05)
    finally:
        d.close()
    return d, st


def hairpin(R, mu, theta=math.pi, w=4.0):
    a = 100.0
    b = a + theta * R
    return road(lambda s: 1.0 / R if a <= s < b else 0.0, b + 300.0, ds=4.0, w=w, mu=mu)


def switches(d):
    """The chooser's mode changes, as (s0, mode) in order."""
    return [(r["s0"], r["mode"]) for r in d.multi.records if r["mode"] != r["mode_before"]]


def test_the_driver_takes_a_slippery_hairpin_in_a_drift_and_keeps_to_it():
    """mu 0.45 (the planner's 0.41), R 9 m x 180 deg: the drift study's clearest drift case. The
    car is planned into it in a drift before the turn-in (s 100), keeps the drift through it,
    and is back in grip after it: one change in, one out."""
    t = hairpin(9.0, 0.45)
    d, st = drive(t, {"cpus": None}, s_end=200.0)
    assert d.stopping is None and st["s"] >= 200.0
    sw = switches(d)
    assert sw and sw[0][1] == "drift" and sw[0][0] < 100.0, sw
    assert len(sw) <= 2 and all(m == "grip" for _, m in sw[1:]), sw


def test_on_an_asphalt_bend_the_driver_stays_grip():
    """mu 1.12, R 40 m x 90 deg: the study never found a drift faster there. A rear-out seed is
    tried, and the car stays grip throughout."""
    t = hairpin(40.0, 1.12, theta=math.pi / 2)
    d, st = drive(t, {"cpus": None}, s_end=250.0, v0=20.0)
    assert d.stopping is None and st["s"] >= 250.0
    assert not switches(d), switches(d)
    assert any("D" in r["seeds"] for r in d.multi.records)


def test_one_start_by_default():
    """No multistart: 060's driver, one solver in this process, no seed processes. The budget is
    made generous (3 s) so that a loaded host cannot cut a solve short: what is tested is the
    path, not its speed, so every re-plan must converge."""
    c = Car.from_spec(SPEC)
    t = road(lambda s: 0.0 if s < 150 else (1 / 60.0 if s < 250 else 0.0), 700.0, ds=4.0, w=5.0)
    d = Driver(c, t, NoTracker(), budget=3.0)
    assert d.multi is None and not multiprocessing.active_children()
    f = PlanFollower(d, 20.0, 10.0)
    st = f.state()
    d.start(st)
    while len(d.solves) < 4:
        d.step(st)
        st = f.step(0.05)
    assert all(x["ok"] for x in d.solves), [(x["status"], x["iters"]) for x in d.solves]
    assert not multiprocessing.active_children()
    assert np.isfinite(d.plan["time"])
