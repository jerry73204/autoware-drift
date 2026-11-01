"""S2's budget (budget.py, roadmap 065 M2): the signed map's conservative price, the knapsack,
the account that is never credited."""
import itertools
import math

import pytest

from race_planner.budget import Account, LossMap, knapsack

PI, HALF = math.pi, math.pi / 2


def _map():
    # asphalt-like: R 9 hairpin drift costs 0.05 s, R 15 0.20 s, R 25 none; 90s cost more
    cells = {"asphalt": [[9.0, PI, 8.0, 0.05, 0.6], [15.0, PI, 8.0, 0.20, 0.5],
                         [25.0, PI, 8.0, None, None],
                         [9.0, HALF, 8.0, 0.10, 0.4], [15.0, HALF, 8.0, 0.30, 0.3],
                         [25.0, HALF, 8.0, 0.50, 0.2]],
             "mud_wet": [[9.0, PI, 6.0, -0.30, 0.9], [9.0, PI, 15.0, 0.02, 0.7]]}
    return LossMap(cells, {"concrete": "asphalt"})


def test_a_measured_cell_is_its_own_price():
    p = _map().lookup("asphalt", 9.0, PI, 8.0)
    assert (p.loss_s, p.share) == (0.05, 0.6)


def test_between_cells_the_price_is_the_worst_bracketing_one():
    p = _map().lookup("asphalt", 11.0, PI, 8.0)
    assert p.loss_s == 0.20 and p.share == 0.5          # the larger loss, the smaller share
    p = _map().lookup("asphalt", 12.0, math.radians(120), 8.0)
    assert p.loss_s == 0.30 and p.share == 0.3          # radius and turn both bracketed


def test_no_price_where_a_bracketing_cell_has_no_drift_or_out_of_range():
    m = _map()
    assert m.lookup("asphalt", 20.0, PI, 8.0) is None    # R 25's hairpin has no drift valley
    assert m.lookup("asphalt", 40.0, HALF, 8.0) is None   # beyond 25 x 1.25
    assert m.lookup("asphalt", 30.0, HALF, 8.0).loss_s == 0.50   # within it
    assert m.lookup("ice", 9.0, PI, 8.0) is None         # a class not in the map
    assert m.lookup("asphalt", 9.0, math.radians(20), 8.0) is None   # not a corner


def test_alias_and_width():
    m = _map()
    assert m.lookup("concrete", 9.0, PI, 8.0).loss_s == 0.05
    assert m.lookup("mud_wet", 9.0, PI, 8.0).loss_s == -0.30    # widest measured <= 8: 6 m
    assert m.lookup("mud_wet", 9.0, PI, 20.0).loss_s == 0.02
    assert m.lookup("mud_wet", 9.0, PI, 4.0).loss_s == -0.30    # narrower than all: narrowest
    assert m.lookup("mud_wet", 9.0, PI, None).loss_s == -0.30


def test_save_and_load(tmp_path):
    p = str(tmp_path / "l.json")
    _map().save(p, note="t")
    m = LossMap.load(p)
    assert m.lookup("asphalt", 11.0, PI, 8.0).loss_s == 0.20
    assert m.lookup("asphalt", 20.0, PI, 8.0) is None


def _brute(items, budget):
    best = (0.0, 0.0, ())
    paid = [x for x in items if x[1] > 0]
    for n in range(len(paid) + 1):
        for c in itertools.combinations(paid, n):
            loss = sum(x[1] for x in c)
            if loss <= budget + 1e-12:
                cand = (sum(x[2] for x in c), -loss, tuple(sorted(x[0] for x in c)))
                if cand[:2] > best[:2]:
                    best = cand
    return set(best[2]), -best[1]


def test_knapsack_is_exact_and_takes_gains_free():
    items = [("a", 0.05, 10.0), ("b", 0.12, 30.0), ("c", 0.08, 12.0), ("d", -0.1, 5.0),
             ("e", 0.30, 50.0), ("f", 0.0, 1.0)]
    for budget in (0.0, 0.05, 0.1, 0.13, 0.2, 0.25, 0.5, 1.0):
        keys, spent = knapsack(items, budget)
        want, wspent = _brute(items, budget)
        assert {"d", "f"} <= set(keys)                   # gains and free ones: always
        assert set(keys) - {"d", "f"} == want, budget
        assert spent == pytest.approx(wspent) and spent <= budget + 1e-9


def test_knapsack_grows_with_the_budget():
    items = [(k, 0.03 * (k + 1), 1.0 + k) for k in range(8)]
    last = -1.0
    for b in (0.0, 0.1, 0.2, 0.5, 1.0, 2.0):
        keys, _ = knapsack(items, b)
        v = sum(1.0 + k for k in keys)
        assert v >= last
        last = v


def test_the_account_is_debited_once_and_never_credited():
    a = Account(0.2, reserve={"c1": 0.05, "c2": 0.1})
    assert a.remaining == 0.2 and a.free() == pytest.approx(0.05)
    assert a.debit("c1", 0.05) == 0.05
    assert a.debit("c1", 0.05) == 0.0                  # once a corner
    assert a.debit("g", -0.3) == 0.0                   # a gain credits nothing
    assert a.remaining == pytest.approx(0.15) and a.free() == pytest.approx(0.05)
    a.debit("c2", 0.1)
    assert a.remaining == pytest.approx(0.05) and a.free() == pytest.approx(0.05)
    assert [d["key"] for d in a.debits] == ["c1", "g", "c2"]


def test_a_course_price_takes_the_place_of_the_maps(tmp_path):
    # roadmap 065 M3: a corner the course priced uses its own (largest loss over the positions,
    # least share), one with no drift at some position is unpriced, one not priced the map's
    import json
    import types
    from race_planner.budget import CoursePrices, S2Lap
    from race_planner.style import Style
    pos = lambda *ls: [{"d": 45.0, "loss_s": x, "share": 0.4 - 0.1 * k, "drift_from": "B"}
                       if x is not None else {"d": 45.0} for k, x in enumerate(ls)]
    path = tmp_path / "prices.json"
    json.dump({"corners": [{"s_in": 100.0, "positions": pos(0.05, 0.09)},
                           {"s_in": 300.0, "positions": pos(-0.02, None)},
                           {"s_in": 500.0, "positions": []}]}, open(path, "w"))
    cp = CoursePrices.load(path)
    assert cp.lookup(101.0)[1].loss_s == pytest.approx(0.09)
    assert cp.lookup(100.0)[1].share == pytest.approx(0.3)
    assert cp.lookup(300.0) == (True, None)
    assert cp.lookup(500.0) == (False, None)
    cs = [types.SimpleNamespace(s_in=s, s_out=s + 40.0, R=25.0, turn=math.pi / 2)
          for s in (100.0, 300.0, 500.0)]
    lap = S2Lap.build(Style(name="drift_budget", eps_lap_s=0.2), cs,
                      lambda c: ({"asphalt": 1.0}, 8.0), _map(), lambda k: False, course=cp)
    assert lap.priced[100.0].loss_s == pytest.approx(0.09)
    assert lap.priced[300.0] is None
    assert lap.priced[500.0] == _map().lookup("asphalt", 25.0, math.pi / 2, 8.0)
