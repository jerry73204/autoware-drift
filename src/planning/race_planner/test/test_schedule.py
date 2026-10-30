"""L1's starts chosen from what lies ahead (schedule.py, surface.py, roadmap 061 M3b): the window
scan, the friction's lower quantile, the table's rules, H's seed and bound, the simulated
perception's decay, the car's estimate held low; and the default path unchanged."""
import math
import os
import types

import numpy as np

from race_planner import modes
from race_planner.car import Car
from race_planner.multistart import MultiStart, held_bound
from race_planner.schedule import CornerAhead, DriftMap, Scheduler, node_classes
from race_planner.surface import FrictionEstimate, Perception, SurfaceMap, Zone
from race_planner.track import SurfaceTable, Track
from test_ocp import SPEC, road

SWB = os.environ.get("SWB", os.path.expanduser("~/repos/carla-scenario-bridge"))
CLASSES = os.path.join(SWB, "src/swb_isaac_world/config/surface_classes.yaml")


def table():
    return SurfaceTable.from_yaml(CLASSES)


def car():
    return Car.from_spec(SPEC)


def bend(R, theta, w=4.0, before=200.0, after=300.0):
    """A straight, a bend of radius R through theta (left), a straight; 4 m grid."""
    b = before + theta * R
    return road(lambda s: 1.0 / R if before <= s < b else 0.0, b + after, ds=4.0, w=w,
                mu=1.12), before, b


def corner(R, turn, probs, dist=100.0, grade=0.0, width=8.0):
    c = modes.Corner(0.0, R * turn, 1.0, R, turn, 1.0, 1.0)
    return CornerAhead(c, dist, probs, grade, width)


# ---- the scan ----

def test_the_scan_finds_a_hairpin_ahead_with_its_radius_turn_and_distance():
    t, a, b = bend(12.0, math.pi)
    c = car()
    sch = Scheduler(t, c, table(), grip=0.92)
    scn = sch.scan(50.0, 310.0)
    assert len(scn.corners) == 1
    ca = scn.corners[0]
    assert abs(ca.corner.R - 12.0) < 0.5
    assert abs(math.degrees(ca.corner.turn) - 180.0) < 15.0      # on the 4 m grid
    assert abs(ca.dist - (ca.corner.s_in - 50.0)) < 1e-9 and 120.0 < ca.dist < 160.0
    assert ca.width == 8.0
    assert ca.probs["asphalt"] == 1.0 and set(node_classes(t, table(), c)) == {"asphalt"}
    assert sch.scan(t.s[-1] - 50.0, t.s[-1]).corners == []      # nothing past the bend


def test_the_planned_friction_is_the_mixtures_lower_quantile():
    tab, c = table(), car()
    m = SurfaceMap(tab, c, [Zone(100.0, 200.0, "asphalt", 0.8, None, ("mud_wet",))])
    mean, sigma, q = m.friction(np.array([50.0, 150.0]))
    mu_a, mu_m = tab.mu("asphalt", c), tab.mu("mud_wet", c)
    assert mean[0] == mu_a and sigma[0] == 0.0 and q[0] == mu_a     # certain: as 060 plans
    want = 0.8 * mu_a + 0.2 * mu_m
    sd = math.sqrt(0.8 * 0.2) * (mu_a - mu_m)
    assert abs(mean[1] - want) < 1e-12 and abs(sigma[1] - sd) < 1e-12
    assert abs(q[1] - max(want - sd, mu_m)) < 1e-12
    # an unlikely class does not set the floor; a likely one bounds it from below
    m2 = SurfaceMap(tab, c, [Zone(0.0, 10.0, "asphalt", 0.5, None, ("ice",))], k=3.0)
    assert abs(m2.friction(np.array([5.0]))[2][0] - tab.mu("ice", c)) < 1e-12


def test_a_certain_map_gives_the_planner_the_track_060_plans_on():
    tab, c = table(), car()
    t = Track.from_swb_course(os.path.join(SWB, "data/courses/haruna"), tab, c)
    p = SurfaceMap(tab, c).planner_track(t)
    for k in ("mu", "sliding", "stiff_long", "stiff_lat", "peak_long", "peak_lat"):
        assert np.array_equal(getattr(p, k), getattr(t, k)), k


def test_declared_and_true_classes_differ_where_the_map_is_wrong():
    tab, c = table(), car()
    t, _, _ = bend(30.0, math.pi / 2)
    m = SurfaceMap(tab, c, [Zone(100.0, 200.0, "asphalt", 1.0, "snow_loose")])
    p, w = m.planner_track(t), m.true_track(t)
    i = int(np.argmin(np.abs(t.s - 150.0)))
    assert p.mu[i] == tab.mu("asphalt", c) and w.mu[i] == tab.mu("snow_loose", c)
    assert w.sliding[i] == tab.sliding("snow_loose") and p.mu[0] == w.mu[0]


# ---- the table ----

def sched(**kw):
    t, _, _ = bend(60.0, math.pi / 2)
    return Scheduler(t, car(), table(), grip=0.92, **kw)


def test_dry_open_asphalt_runs_g_alone_and_only_a_hairpin_whose_gain_clears_the_gate_adds_h_d():
    s = sched()
    assert s.corner_starts(corner(60.0, math.pi / 2, {"asphalt": 1.0}))[0] == set()
    assert s.corner_starts(corner(40.0, math.pi, {"asphalt": 1.0}))[0] == set()
    # R 9 pays 0.40 % (>= the 0.3 % switch threshold), R 15 0.23 %: not every hairpin
    assert s.corner_starts(corner(9.0, math.radians(170), {"asphalt": 1.0}))[0] == {"H", "D"}
    assert s.corner_starts(corner(9.4, math.radians(170), {"asphalt": 1.0}))[0] == {"H", "D"}
    # M4b: no stretch past the measured R 9 on a class that falls off (Haruna's R 9.9-11.2 m
    # hairpins lost 0.09 s with D on the plant)
    assert s.corner_starts(corner(9.9, math.radians(170), {"asphalt": 1.0}))[0] == set()
    assert s.corner_starts(corner(10.0, math.radians(170), {"asphalt": 1.0}))[0] == set()
    assert s.corner_starts(corner(12.0, math.radians(170), {"asphalt": 1.0}))[0] == set()
    assert s.corner_starts(corner(15.0, math.pi, {"asphalt": 1.0}))[0] == set()
    assert s.corner_starts(corner(10.0, math.radians(100), {"asphalt": 1.0}))[0] == set()
    # the old rule (gain_min on every class) seeded every asphalt hairpin up to R 31
    old = sched(rules={"gain_falloff": 0.05})
    assert old.corner_starts(corner(15.0, math.pi, {"asphalt": 1.0}))[0] == {"H", "D"}


def test_loose_ground_without_fall_off_adds_f_into_hairpins_ice_only_h():
    s = sched()
    assert s.corner_starts(corner(9.0, math.pi, {"mud_wet": 1.0}))[0] == {"H", "D", "F"}
    assert s.corner_starts(corner(15.0, math.pi / 2, {"mud_wet": 1.0}))[0] == {"H", "D"}
    assert s.corner_starts(corner(9.0, math.pi, {"gravel": 1.0}))[0] == {"H", "D"}    # falls off:
    assert s.corner_starts(corner(15.0, math.pi, {"gravel": 1.0}))[0] == set()      # 0.15 % < 0.3
    assert s.corner_starts(corner(25.0, math.pi / 2, {"mud_wet": 1.0}))[0] == {"H", "D"}  # 0.12 %
    assert s.corner_starts(corner(9.0, math.pi, {"ice": 1.0}))[0] == {"H"}
    # the stretch stays on a class without fall-off, goes on one with it
    assert s.corner_starts(corner(11.0, math.pi, {"mud_wet": 1.0}))[0] == {"H", "D", "F"}
    assert s.corner_starts(corner(11.0, math.pi, {"gravel": 1.0}))[0] == set()


def test_a_class_above_p_add_adds_its_starts_below_it_does_not():
    s = sched()
    assert s.corner_starts(corner(15.0, math.pi / 2, {"asphalt": 0.85, "mud_wet": 0.15}))[0] \
        == {"H", "D"}
    assert s.corner_starts(corner(15.0, math.pi / 2, {"asphalt": 0.9, "mud_wet": 0.1}))[0] \
        == set()


def test_a_steep_downhill_on_low_friction_adds_d():
    s = sched()
    flat = corner(60.0, math.pi / 2, {"snow_packed": 1.0})
    steep = corner(60.0, math.pi / 2, {"snow_packed": 1.0}, grade=-0.08)
    assert s.corner_starts(flat)[0] == {"H"}
    assert s.corner_starts(steep)[0] == {"H", "D"}
    dry = corner(60.0, math.pi / 2, {"asphalt": 1.0}, grade=-0.08)
    assert s.corner_starts(dry)[0] == set()


def test_no_new_drift_start_into_a_corner_under_25_m_ahead_but_one_running_keeps_on():
    t, a, b = bend(9.0, math.pi)
    tab, c = table(), car()
    m = SurfaceMap(tab, c, [Zone(0.0, 1e4, "mud_wet")])
    s = Scheduler(m.planner_track(t), c, tab, smap=m, grip=0.92)
    cn = s.corners[0]
    far = s.starts(s.scan(cn.s_in - 60.0, cn.s_in + 200.0), cn)[0]
    near = s.starts(s.scan(cn.s_in - 20.0, cn.s_in + 240.0), cn)[0]
    kept = s.starts(s.scan(cn.s_in - 20.0, cn.s_in + 240.0), cn, running=("D",))[0]
    assert far == {"G", "H", "D", "F"} and near == {"G", "H"} and kept == {"G", "H", "D"}


def test_width_from_the_arm_a_corner_reads_the_widest_measured_width_not_over_its_own():
    m = DriftMap({"mud_wet": [[15.0, math.pi / 2, 0.6, 6.0], [15.0, math.pi / 2, 0.6, 8.0]]},
                 widths={"mud_wet": [6.0, 8.0, 10.0, 15.0]})
    assert m.pays("mud_wet", 15.0, math.pi / 2, 8.0) and m.pays("mud_wet", 15.0, math.pi / 2, 9.0)
    assert not m.pays("mud_wet", 15.0, math.pi / 2, 10.0)
    assert m.pays("mud_wet", 15.0, math.pi / 2, None)            # unknown: the narrowest
    assert m.pays("unheard_of", 15.0, math.pi / 2, 12.0)          # unknown class: loose in


def test_always_runs_g_h_d_and_f_into_hairpins():
    s = sched(always=True)
    scn = s.scan(0.0, 200.0)
    hp = modes.Corner(0.0, 30.0, 1.0, 9.0, math.pi, 1.0, 1.0)
    assert s.starts(scn, hp)[0] == {"G", "H", "D", "F"}
    assert s.starts(scn, None)[0] == {"G", "H", "D"}


# ---- H ----

def test_h_seed_holds_the_rear_axle_within_its_bound():
    c = car()
    peak = modes.rear_peak(c, 0.2 * 0.92, 0.8)
    b = held_bound(peak)
    assert math.degrees(peak) < b <= math.degrees(peak) + 1.0 + 1e-9
    n = 30
    X = np.zeros((6, n))
    X[0], X[1], X[2] = 15.0, -3.0, 0.4           # a slide: rear slip about -20 deg
    U = np.zeros((7, n - 1))
    ms = types.SimpleNamespace(car=c)
    Xh, Uh = MultiStart._held_seed(ms, (X, U), b)
    rear = np.degrees(np.abs(modes.rear_slip(c, Xh)))
    assert rear[0] > b and np.all(rear[1:] <= b + 1e-6)
    assert np.array_equal(Xh[0], X[0]) and np.array_equal(Uh, U)


# ---- perception and the estimate ----

def test_perception_is_right_near_and_near_chance_far_and_seeded():
    tab, c = table(), car()
    m = SurfaceMap(tab, c, [Zone(0.0, 1e4, "snow_loose")])
    p = Perception(m, seed=3, reach=1e4)
    assert p.accuracy(10.0) == 0.95 and abs(p.accuracy(100.0) - 1 / len(m.classes)) < 1e-12
    assert np.all(np.diff(p.accuracy(np.linspace(0, 120, 50))) <= 1e-12)

    def hits(d, n=1500):
        return np.mean([p.read(0.0, [d])[0] == "snow_loose" for _ in range(n)])
    assert hits(20.0) > 0.92 and hits(150.0) < 0.2
    a = [Perception(m, seed=5, reach=1e4).read(0.0, [80.0])[0] for _ in range(3)]
    assert len(set(a)) == 1
    near = Perception(m, seed=0)
    assert near.read(0.0, [70.0, -5.0]) == [None, None]       # beyond reach, behind


def test_perception_only_lowers_the_planned_friction_near_the_car():
    tab, c = table(), car()
    m = SurfaceMap(tab, c, [Zone(0.0, 50.0, "asphalt", 1.0, "ice")])
    p = Perception(m, seed=0, acc_near=1.0)
    s = np.array([10.0, 20.0, 45.0, 80.0])
    sc = p.scale(0.0, s, np.full(4, tab.mu("asphalt", c)))
    assert sc[0] < 1.0 and sc[1] < 1.0 and sc[2] == 1.0 and sc[3] == 1.0
    up = p.scale(0.0, s, np.full(4, 0.1))                   # a map below the world: unchanged
    assert np.all(up == 1.0)


def test_the_estimate_only_lowers_holds_through_a_slide_and_is_released_slowly():
    e = FrictionEstimate(release=0.05, hold=1.0)
    assert e.update(0.0) == 1.0
    assert e.update(0.1, 0.6, sliding=True) == 0.6
    assert e.update(0.2, 0.9, sliding=True) == 0.6           # a higher reading: no rise
    for k in range(3, 30):                                   # mid-slide, no reading: held
        assert e.update(0.1 * k, None, sliding=True) == 0.6
    assert e.update(3.5, None, sliding=False) == 0.6         # inside the hold after the slide
    v = e.update(4.0, None, sliding=False)
    assert 0.6 < v <= 0.6 + 0.05 * 1.0 + 1e-12               # released at its rate
    assert e.update(4.0, 0.5) == 0.5                         # a lower reading at once


def test_after_a_pass_the_estimate_writes_a_lower_class_into_the_map():
    tab, c = table(), car()
    m = SurfaceMap(tab, c)
    log = [(s, tab.mu("snow_packed", c) + 0.01) for s in np.arange(100.0, 160.0, 2.0)]
    log += [(s, tab.mu("asphalt", c)) for s in np.arange(0.0, 100.0, 2.0)]
    z = m.correct(log)
    assert len(z) == 1 and z[0].declared == "snow_packed" and 98 <= z[0].s0 and z[0].s1 <= 160
    assert m.friction(np.array([130.0]))[2][0] < tab.mu("asphalt", c)
    assert m.correct([(s, 5.0) for s in np.arange(0.0, 100.0, 2.0)]) == []   # never raises


# ---- the driver ----

def follow(t, multistart, s_end, v0=12.0, **kw):
    from race_planner.live import Driver
    from race_planner.sim import NoTracker, PlanFollower
    d = Driver(car(), t, NoTracker(), budget=3.0, multistart=multistart, **kw)
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


def test_scheduled_on_an_open_asphalt_bend_runs_g_alone_and_plans_as_060():
    """Dry asphalt, R 60 m x 90 deg: the table asks for G alone at every re-plan, and G is solved
    in the driver's process as 060 solves it: the same plans, bit for bit."""
    t, _, _ = bend(60.0, math.pi / 2)
    one, _ = follow(t, None, 300.0, v0=20.0)
    sch, _ = follow(t, {"seed_budget": 3.0, "schedule": {"table": table()}}, 300.0, v0=20.0)
    assert all(r["slots"] == ["G"] for r in sch.multi.records)
    a = [(x["ok"], x["iters"], x["status"]) for x in one.solves]
    b = [(x["ok"], x["iters"], x["status"]) for x in sch.solves]
    assert a == b
    assert np.array_equal(one.plan["X"], sch.plan["X"]) and one.plan["time"] == sch.plan["time"]


def test_scheduled_on_a_mud_hairpin_runs_h_and_d_and_finishes():
    """mud_wet, R 9 m x 180 deg: H (held at the rear tyre's peak) and D run into it, H's plans
    keep the rear axle within the bound, and the car gets through."""
    t, a, b = bend(9.0, math.pi, before=120.0)
    tab, c = table(), car()
    m = SurfaceMap(tab, c, [Zone(0.0, 1e4, "mud_wet")])
    pt = m.planner_track(t)
    d, st = follow(pt, {"seed_budget": 3.0, "schedule": {"table": tab, "smap": m}}, b + 40.0)
    assert d.stopping is None and st["s"] >= b + 40.0
    recs = d.multi.records
    assert any({"H", "D"} <= set(r["slots"]) for r in recs)
    assert any(r["seeds"]["H"]["ok"] for r in recs if "H" in r["seeds"])


def test_h_is_the_fallback_adopted_only_when_no_free_start_has_a_plan():
    m = MultiStart.__new__(MultiStart)
    m.threshold, m.confirm, m.ahead, m.mode, m.count = 0.003, 2, 25.0, "grip", 0
    m.h_competes = False
    g_drift = {"ok": True, "time": 20.0, "mode": "drift", "onset": 40.0}
    h = {"ok": True, "time": 20.1, "mode": "grip", "onset": None}
    assert m.choose({"G": g_drift, "H": h}) == "G" and m.mode == "drift"   # as M3: G decides
    m.mode = "grip"
    failed = {"ok": False, "time": 0.0, "mode": None, "onset": None, "iters": 400}
    assert m.choose({"G": failed, "H": h}) == "H" and m.mode == "grip"
    m.h_competes, m.mode = True, "grip"
    assert m.choose({"G": g_drift, "H": h}) == "H"                          # weighed: held once
