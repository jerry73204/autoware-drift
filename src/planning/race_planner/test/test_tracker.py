"""L2, the tracker (tracker.py): L1's plan followed in closed loop on sim.py's plant (the two-track
car with wheel slip, behind the real sim_vehicle_ecu) at 50 Hz, one plan for its whole length.
The plans drift (L1 finds sliding through these bends faster), so what is checked is that the
slide is held, not killed, within the road and the time a tick may take."""
import math

import numpy as np
import pytest

from race_planner.car import Car
from race_planner.ocp import STEER_RATE
from race_planner.sim import Plant
from race_planner.tracker import BUDGET, LqrTracker, Tracker, accel_limits
from race_planner.window import Window, initial
from test_ocp import SPEC, road

TICK = 0.02
WIDTH = 5.0           # test_ocp.road's free width either side of the centre line
# p99 of a tick's solve: the design's 10 ms; asserted with room for a loaded test machine
P99_LIMIT = 1.5 * BUDGET


def bend():
    """A 180-degree bend of radius 20 m after 80 m of straight, entered at 15 m/s."""
    return road(lambda s: 1.0 / 20.0 if 80.0 <= s < 80.0 + math.pi * 20.0 else 0.0, 300.0, ds=4.0)


def s_bend():
    """Alternating bends down to radius 25 m."""
    return road(lambda s: (1.0 / 25.0) * math.sin(2 * math.pi * s / 150.0) if 30 <= s < 330
                else 0.0, 400.0, ds=4.0)


@pytest.fixture(scope="module")
def car():
    return Car.from_spec(SPEC)


def make_plan(track, c):
    w = Window(c, ds=4.0, length=200.0, warm=False)
    X, U = initial(w.section(track, 0), c, v_start=15.0)
    x0 = X[:, 0].copy()
    x0[0] = 15.0
    plan = w.solve(track, 0, x0, 60.0, X, U)
    assert plan["ok"], plan["status"]
    return plan


@pytest.fixture(scope="module")
def bend_case(car):
    t = bend()
    return t, make_plan(t, car)


@pytest.fixture(scope="module")
def s_case(car):
    t = s_bend()
    return t, make_plan(t, car)


def drive(tracker, track, plan, c, dn=0.0, vy=0.0):
    """The plant on `track` from the plan's start (dn off its line, vy sideways) under `tracker`
    until the plan's end. Returns errors against the plan at the car's s, commands and solves."""
    p = Plant(c, SPEC, track.resampled(1.0))
    p.reset(s=0.0, n=plan["X"][3, 0] + dn, speed=plan["X"][0, 0])
    p.v[1] = vy
    st = p.state()
    rec = {k: [] for k in ("lat", "speed", "n", "solve", "fallback", "accel", "tire", "vx")}
    while st["s"] < plan["s"][-1] - 5.0 and p.t < plan["time"] + 3.0:
        o = tracker.step(st, plan, 0.0)
        rec["solve"].append(o["solve_s"]); rec["fallback"].append(o["fallback"])
        rec["accel"].append(o["accel"]); rec["tire"].append(o["tire_angle"])
        rec["vx"].append(st["vx"])
        st = p.step(o["accel"], o["tire_angle"], dt=TICK)
        rec["lat"].append(st["n"] - np.interp(st["s"], plan["s"], plan["X"][3]))
        rec["speed"].append(st["vx"] - np.interp(st["s"], plan["s"], plan["X"][0]))
        rec["n"].append(st["n"])
        if abs(st["n"]) > 2 * WIDTH:            # off the road and gone: stop
            break
    rec = {k: np.asarray(v) for k, v in rec.items()}
    rec["s_end"] = st["s"]
    return rec


def rms(x):
    return float(np.sqrt(np.mean(np.square(x))))


def check_on_road(rec, plan, c):
    assert rec["s_end"] > plan["s"][-1] - 10.0                     # reached the plan's end
    assert np.max(np.abs(rec["n"])) <= WIDTH - 0.5 * c.width        # the body never off the road


def check_limits(rec, c):
    """SR3: no command the car cannot carry out."""
    rate = c.steer_rate_max or STEER_RATE
    assert np.all(np.abs(rec["tire"]) <= c.steer_max + 1e-9)
    assert np.all(np.abs(np.diff(rec["tire"])) <= rate * TICK + 1e-9)
    for a, v in zip(rec["accel"], rec["vx"]):
        lo, hi = accel_limits(c, v)
        assert lo - 1e-9 <= a <= hi + 1e-9


@pytest.mark.parametrize("case", ["bend_case", "s_case"])
def test_the_nmpc_holds_the_planned_slide_on_the_line(case, car, request):
    track, plan = request.getfixturevalue(case)
    assert np.max(np.abs(plan["X"][1])) > 3.0            # the plan drifts: vy past 3 m/s
    tr = Tracker(car, track)
    rec = drive(tr, track, plan, car)
    check_on_road(rec, plan, car)
    check_limits(rec, car)
    assert rms(rec["lat"]) < 0.5
    assert rms(rec["speed"]) < 1.5
    solve = rec["solve"][1:]                              # the first tick pages the solver in
    p50, p99 = np.percentile(solve, [50, 99])
    print(f"{case}: lateral rms {rms(rec['lat']):.2f} m max {np.max(np.abs(rec['lat'])):.2f}, "
          f"speed rms {rms(rec['speed']):.2f} m/s, solve p50 {1e3 * p50:.2f} ms "
          f"p99 {1e3 * p99:.2f} ms, fallbacks {int(rec['fallback'].sum())}")
    assert p99 <= P99_LIMIT
    assert rec["fallback"].mean() < 0.02


@pytest.mark.parametrize("case", ["bend_case", "s_case"])
def test_the_nmpc_brings_a_disturbed_car_back(case, car, request):
    """0.8 m off the plan's line and sliding 1 m/s sideways at the start: back on it within two
    seconds, never off the road."""
    track, plan = request.getfixturevalue(case)
    rec = drive(Tracker(car, track), track, plan, car, dn=0.8, vy=1.0)
    check_on_road(rec, plan, car)
    check_limits(rec, car)
    after = int(2.0 / TICK)
    assert np.max(np.abs(rec["lat"][:5])) > 0.7
    assert rms(rec["lat"][after:]) < 0.5
    assert rms(rec["speed"]) < 1.5


def test_the_lqr_alone_tracks_the_s_bend(car, s_case):
    """The fallback on its own, every tick: looser, near the line and through the plan, but no
    longer on the road: since L1 plans with PhysX's tyres on every wheel (window.py) the S-bend's
    plan drifts at the edge of the road, and the LQR, whose model keeps the ellipse and has no
    grip limit, overshoots it by up to 0.7 m (alone through the 20 m bend's deep drift it spun
    before too). It is a one-tick fallback near the plan: see the next test."""
    track, plan = s_case
    rec = drive(LqrTracker(car, track), track, plan, car)
    assert rec["s_end"] > plan["s"][-1] - 10.0
    check_limits(rec, car)
    assert rms(rec["lat"]) < 1.0
    assert rms(rec["speed"]) < 2.5
    assert np.percentile(rec["solve"], 99) <= P99_LIMIT


class _Missing(Tracker):
    """The NMPC missing its budget every other tick: the LQR's then."""

    def step(self, *a):
        self.ticks = getattr(self, "ticks", 0) + 1
        self.budget = 0.0 if self.ticks % 2 == 0 else BUDGET
        return super().step(*a)


@pytest.mark.parametrize("case", ["bend_case", "s_case"])
def test_the_lqr_as_a_fallback_keeps_the_car_on_the_road(case, car, request):
    """The LQR as it is used: for the ticks the NMPC misses, here every other one."""
    track, plan = request.getfixturevalue(case)
    rec = drive(_Missing(car, track), track, plan, car)
    assert 0.4 < rec["fallback"].mean() < 0.6
    check_on_road(rec, plan, car)
    check_limits(rec, car)
    assert rms(rec["lat"]) < 0.8
    assert rms(rec["speed"]) < 1.5


def test_an_overrun_tick_falls_back_to_the_lqr(car, s_case):
    """A tick past its budget (here: any) is the LQR's, within the car's limits."""
    track, plan = s_case
    tr = Tracker(car, track, budget=0.0)
    p = Plant(car, SPEC, track.resampled(1.0))
    st = p.reset(s=0.0, n=plan["X"][3, 0], speed=plan["X"][0, 0])
    for _ in range(20):
        o = tr.step(st, plan, 0.0)
        assert o["fallback"] and o["ok"]
        assert abs(o["tire_angle"]) <= car.steer_max
        st = p.step(o["accel"], o["tire_angle"], dt=TICK)
    assert abs(st["n"] - plan["X"][3, 0]) < 0.3
