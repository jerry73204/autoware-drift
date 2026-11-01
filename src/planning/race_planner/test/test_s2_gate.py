"""S2's gate (multistart.MultiStart._choose_s2, roadmap 065 M2): corners the lap budget bought
take their drift within its price; the lex-eps gate on classes without fall-off adopts the plan
that drifts most among those within (1 + eps) of the fastest, with hysteresis; other corners
are S1's; the account is debited once a corner by the predicted loss."""
import types

import pytest

from race_planner.budget import Account, Price, S2Lap
from race_planner.multistart import MultiStart
from race_planner.style import Style

CORNER = types.SimpleNamespace(s_in=100.0, s_out=140.0)
OTHER = types.SimpleNamespace(s_in=300.0, s_out=340.0)


def gate(eps_lap_s=0.2, eps_window=0.0, committed=None, lex=None, corner=CORNER):
    st = Style(name="drift_budget", eps_lap_s=eps_lap_s, eps_window=eps_window)
    m = MultiStart.__new__(MultiStart)
    m.threshold, m.confirm, m.ahead, m.mode, m.count = 0.003, 2, 25.0, "grip", 0
    m.style = st
    m.s2 = S2Lap(st, committed=committed, lex=lex)
    m._corner = corner
    return m


def res(grip=20.0, drifts=((20.1, 30.0),), onset=40.0):
    r = {"G": {"ok": True, "time": grip, "mode": "grip", "onset": None, "drift_m": 0.0}}
    for k, (t, dm) in enumerate(drifts):
        r["DF"[k]] = {"ok": True, "time": t, "mode": "drift", "onset": onset, "drift_m": dm}
    return r


def test_a_bought_corner_drifts_within_its_price_and_debits_once():
    m = gate(committed={100.0: Price(0.12, 0.6, ())})
    assert m.choose(res(20.0, ((20.1, 30.0),))) == "G"      # inside once: not yet
    assert m.choose(res(20.0, ((20.1, 30.0),))) == "D"      # twice
    assert m.mode == "drift"
    assert m.s2.account.spent == pytest.approx(0.12)
    assert m.choose(res(20.0, ((20.1, 30.0),))) == "D"
    assert m.s2.account.spent == pytest.approx(0.12)        # once a corner


def test_a_bought_corner_whose_drift_costs_more_than_its_price_stays_grip():
    m = gate(committed={100.0: Price(0.05, 0.6, ())})
    for _ in range(4):                                       # 0.2 s > 0.05 + 0.3 % of 20 s
        assert m.choose(res(20.0, ((20.2, 30.0),))) == "G"
    assert m.s2.account.spent == 0.0


def test_a_bought_corner_leaves_its_drift_only_outside_price_plus_hysteresis_twice():
    m = gate(committed={100.0: Price(0.1, 0.6, ())})
    m.choose(res(20.0, ((20.1, 30.0),)))
    m.choose(res(20.0, ((20.1, 30.0),)))
    assert m.mode == "drift"
    # allowed: 0.1 + 0.3 % (0.06) = 0.16 s; exit beyond 0.16 + 0.3 % (0.06) = 0.22 s
    assert m.choose(res(20.0, ((20.2, 30.0),))) == "D"      # outside entry, inside exit
    assert m.choose(res(20.0, ((20.3, 30.0),))) == "D"      # outside exit once
    assert m.choose(res(20.0, ((20.3, 30.0),))) == "G"      # twice
    assert m.mode == "grip"


def test_a_drift_carried_into_another_bought_corner_is_entered_and_paid_there():
    # issue 010: the mode spans corners; the next bought corner's own rule and debit apply
    m = gate(eps_lap_s=0.5, committed={100.0: Price(0.1, 0.6, ()), 300.0: Price(0.05, 0.6, ())})
    m.choose(res(20.0, ((20.1, 30.0),)))
    m.choose(res(20.0, ((20.1, 30.0),)))
    assert m.mode == "drift" and m.s2.account.spent == pytest.approx(0.1)
    m._corner = OTHER                                         # still drifting, the next corner
    assert m.choose(res(20.0, ((20.08, 30.0),))) == "D"      # 0.08 <= 0.05 + 0.06: entered
    assert m.s2.account.spent == pytest.approx(0.15)
    assert m.s2.account.debited(300.0)


def test_a_drift_carried_into_a_corner_it_does_not_fit_is_left_on_two_replans():
    m = gate(eps_lap_s=0.5, committed={100.0: Price(0.1, 0.6, ()), 300.0: Price(0.02, 0.6, ())})
    m.choose(res(20.0, ((20.1, 30.0),)))
    m.choose(res(20.0, ((20.1, 30.0),)))
    m._corner = OTHER
    # 0.1 s > 0.02 + 0.06: not this corner's drift, though inside the old corner's exit band
    assert m.choose(res(20.0, ((20.1, 30.0),))) == "D"       # once
    assert m.choose(res(20.0, ((20.1, 30.0),))) == "G"       # twice: left
    assert m.mode == "grip"
    assert not m.s2.account.debited(300.0)
    assert m.s2.account.spent == pytest.approx(0.1)


def test_a_corner_neither_bought_nor_lex_is_s1s():
    m = gate()
    for _ in range(3):                                        # slower: S1 stays grip
        assert m.choose(res(20.0, ((20.05, 30.0),))) == "G"
    assert m.choose(res(20.0, ((19.9, 30.0),))) == "G"       # 0.5 % faster: S1's two re-plans
    assert m.choose(res(20.0, ((19.9, 30.0),))) == "D"
    assert m.s2.account.spent == 0.0


def test_no_corner_ahead_is_s1s():
    m = gate(committed={100.0: Price(0.1, 0.6, ())}, corner=None)
    assert m.choose(res(20.0, ((20.05, 30.0),))) == "G"
    assert m.choose(res(20.0, ((20.05, 30.0),))) == "G"


def test_the_lex_gate_takes_the_most_drift_within_eps():
    m = gate(eps_lap_s=0.0, eps_window=0.01, lex={100.0: None})
    r = res(20.0, ((20.15, 20.0), (20.18, 35.0)))           # both within 1 %: F drifts more
    assert m.choose(r) == "G"
    assert m.choose(r) == "F"
    assert m.mode == "drift"
    r = res(20.0, ((20.15, 35.0), (20.18, 35.0)))           # a tie in drift: the faster
    assert m.choose(r) == "D"


def test_the_lex_gate_refuses_beyond_eps_and_near_onsets():
    m = gate(eps_lap_s=0.0, eps_window=0.01, lex={100.0: None})
    for _ in range(3):
        assert m.choose(res(20.0, ((20.25, 30.0),))) == "G"   # 1.25 % > 1 %
    for _ in range(3):
        assert m.choose(res(20.0, ((20.1, 30.0),), onset=12.0)) == "G"   # trail braking cut


def test_the_lex_gate_exits_outside_eps_plus_h_on_two_replans():
    m = gate(eps_lap_s=0.0, eps_window=0.01, lex={100.0: None})
    m.choose(res(20.0, ((20.1, 30.0),)))
    m.choose(res(20.0, ((20.1, 30.0),)))
    assert m.mode == "drift"
    assert m.choose(res(20.0, ((20.24, 30.0),))) == "D"      # 1.2 %: inside 1 + 0.3 %
    assert m.choose(res(20.0, ((20.3, 30.0),))) == "D"       # 1.5 %: outside once
    assert m.choose(res(20.24, ((20.25, 30.0),))) == "D"     # back inside: the run broken
    assert m.choose(res(20.0, ((20.3, 30.0),))) == "D"
    assert m.choose(res(20.0, ((20.3, 30.0),))) == "G"
    assert m.mode == "grip"


def test_the_lex_gate_stops_when_the_account_cannot_pay():
    st_price = Price(0.3, 0.5, ())
    m = gate(eps_lap_s=0.2, eps_window=0.01, lex={100.0: st_price})
    for _ in range(3):
        assert m.choose(res(20.0, ((20.1, 30.0),))) == "G"   # 0.3 s predicted > 0.2 s left
    m = gate(eps_lap_s=0.5, eps_window=0.01, lex={100.0: st_price})
    m.choose(res(20.0, ((20.1, 30.0),)))
    assert m.choose(res(20.0, ((20.1, 30.0),))) == "D"
    assert m.s2.account.spent == pytest.approx(0.3)


def test_the_lex_gate_with_no_map_debits_the_measured_loss_when_a_budget_is_set():
    m = gate(eps_lap_s=0.5, eps_window=0.01, lex={100.0: None})
    m.choose(res(20.0, ((20.1, 30.0),)))
    m.choose(res(20.0, ((20.1, 30.0),)))
    assert m.s2.account.spent == pytest.approx(0.1)


def test_one_mode_change_a_corner_when_the_loss_hovers_at_the_edge():
    m = gate(committed={100.0: Price(0.1, 0.6, ())})
    seq = [20.15, 20.17, 20.15, 20.17, 20.15, 20.17]          # around the 0.16 s allowance
    modes = []
    for t in seq:
        m.choose(res(20.0, ((t, 30.0),)))
        modes.append(m.mode)
    assert modes.count("drift") == 0 or len({*modes}) <= 2
    changes = sum(a != b for a, b in zip(["grip"] + modes, modes))
    assert changes <= 1


def test_s1_style_is_untouched_by_the_s2_gate():
    m = MultiStart.__new__(MultiStart)          # no style attribute: S1's chooser as before
    m.threshold, m.confirm, m.ahead, m.mode, m.count = 0.003, 2, 25.0, "grip", 0
    assert m.choose(res(20.0, ((19.9, 30.0),))) == "G"
    assert m.choose(res(20.0, ((19.9, 30.0),))) == "D"


def test_build_buys_the_cheapest_drift_per_second_and_lists_flat_corners():
    from race_planner.budget import LossMap
    import math
    lm = LossMap({"asphalt": [[9.0, math.pi, 8.0, 0.05, 0.6], [15.0, math.pi, 8.0, 0.4, 0.5]],
                  "mud_wet": [[9.0, math.pi, 8.0, 0.02, 0.8]]})
    cs = [types.SimpleNamespace(s_in=100.0, s_out=130.0, R=9.0, turn=math.pi),
          types.SimpleNamespace(s_in=300.0, s_out=340.0, R=15.0, turn=math.pi),
          types.SimpleNamespace(s_in=500.0, s_out=530.0, R=9.0, turn=math.pi)]
    cls = {100.0: "asphalt", 300.0: "asphalt", 500.0: "mud_wet"}
    st = Style(name="drift_budget", eps_lap_s=0.1, eps_window=0.01)
    lap = S2Lap.build(st, cs, lambda c: ({cls[c.s_in]: 1.0}, 8.0), lm,
                      lambda k: k == "mud_wet")
    assert set(lap.committed) == {100.0, 500.0}             # 0.07 s of 0.1; R 15 too dear
    assert set(lap.lex) == set()                            # the flat one is bought already
    assert lap.account.reserved == {100.0: 0.05, 500.0: 0.02}
    st = Style(name="drift_budget", eps_lap_s=0.03, eps_window=0.01)
    lap = S2Lap.build(st, cs, lambda c: ({cls[c.s_in]: 1.0}, 8.0), lm,
                      lambda k: k == "mud_wet")
    assert set(lap.committed) == {500.0} and set(lap.lex) == set()
    st = Style(name="drift_budget", eps_lap_s=0.0, eps_window=0.01)
    lap = S2Lap.build(st, cs, lambda c: ({cls[c.s_in]: 1.0}, 8.0), lm,
                      lambda k: k == "mud_wet")
    assert set(lap.committed) == set() and set(lap.lex) == {500.0}


# ---- the driver: S2's plumbing on the scheduled starts ----

def _every_class_map(cost):
    import math
    import os
    from race_planner.budget import LossMap
    from race_planner.track import SurfaceTable
    swb = os.environ.get("SWB", os.path.expanduser("~/repos/carla-scenario-bridge"))
    tab = SurfaceTable.from_yaml(os.path.join(
        swb, "src/swb_isaac_world/config/surface_classes.yaml"))
    return LossMap({c: [[40.0, math.pi / 2, 8.0, cost, 0.5]] for c in tab.mu_of})


def _s2_drive(eps_lap_s, cost):
    import math
    from test_multistart import FREE, drive, hairpin
    t = hairpin(40.0, 1.12, theta=math.pi / 2)
    st = {"name": "drift_budget", "eps_lap_s": eps_lap_s}
    import race_planner.live as live_mod
    from race_planner.car import Car
    from race_planner.sim import NoTracker, PlanFollower
    from test_ocp import SPEC
    c = Car.from_spec(SPEC)
    d = live_mod.Driver(c, t, NoTracker(), style=st, **FREE,
                        multistart={"cpus": None, "seed_budget": 3.0, "schedule": {},
                                    "loss_map": _every_class_map(cost)})
    f = PlanFollower(d, 10.0, 20.0)
    s = f.state()
    try:
        d.start(s)
        while s["s"] < 160.0 and s["t"] < 60.0 and d.stopping is None:
            d.step(s)
            s = f.step(0.05)
    finally:
        d.close()
    return d


def test_the_driver_buys_a_corner_within_the_budget_and_schedules_its_drift():
    d = _s2_drive(1.0, 0.5)
    lap = d.multi.s2
    assert set(lap.committed) and lap.account.budget == 1.0
    recs = [r for r in d.multi.records if r.get("s2")]
    assert recs and all(r["s2"]["kind"] == "budget" for r in recs)
    assert any("s2 budget" in r["why"] for r in d.multi.records)
    assert all("drift_m" in v for r in recs for v in r["seeds"].values())
    assert lap.account.spent in (0.0, 0.5)          # debited once, if the drift was entered
    # B, the bought corner's drift forced (forced.py): run, and a drift wherever it converged
    b = [r["seeds"]["B"] for r in recs if "B" in r["seeds"]]
    assert b and any(x["ok"] for x in b)
    assert all(x["mode"] == "drift" for x in b if x["ok"])
    assert any("s2 forced" in r["why"] for r in d.multi.records)


def test_the_driver_with_no_budget_buys_nothing_dear():
    d = _s2_drive(0.0, 0.5)
    assert not d.multi.s2.committed and not d.multi.s2.lex
    assert all(r.get("s2") is None for r in d.multi.records)
    assert not any("s2 budget" in r["why"] for r in d.multi.records)
    assert not any("B" in r["seeds"] for r in d.multi.records)


def test_the_forced_floor_is_past_the_cars_own_peak_not_the_planners():
    # roadmap 065 M3: the planner's peak (grip 0.92) is 1.25 deg earlier on asphalt; a floor
    # past it left the car 1.9 deg past its real one
    import math
    from race_planner import modes
    from race_planner.car import Car
    from test_ocp import SPEC
    car = Car.from_spec(SPEC)
    m = MultiStart.__new__(MultiStart)
    m.car, m.grip, m.style = car, 0.92, Style(name="drift_budget")
    c = types.SimpleNamespace(s_in=100.0, s_out=140.0, sign=1.0, mu=0.92 * 1.12, sliding=0.8,
                              surface=None)
    deg = m._floor(c)[0]
    real = math.degrees(modes.rear_peak(car, 1.12, 0.8))
    planner = math.degrees(modes.rear_peak(car, 0.92 * 1.12, 0.8))
    assert real > planner + 1.0
    assert deg == math.ceil(real + 3.0 - 1e-9)
