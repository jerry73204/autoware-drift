"""S2 `drift_budget`: what a drift costs, the lap's budget spent on it, and the account
(roadmap 065 M2; design docs/superpowers/specs/2026-10-31-drift-style-layer-design.md, 4.3 and
section 8, call 3).

- `LossMap`: the signed drift map (race_planner/drift_losses.json, built by
  studies/drift_optimality.py `drift-losses` from the `s2-signed` grid): per surface class, per
  cell (R, turn, width), the best drift plan's time against the best grip plan's over the
  study's corner section, signed (a gain negative), and the metres of the corner's zone the
  drift slides; or that no start stayed in the drift valley there (no drift to buy).
- `lookup` prices a corner conservatively: the cells that bracket its radius and turn, the
  largest loss among them and the least drift share; no price (None) where any bracketing cell
  has no drift, or the corner lies outside the measured radii by more than the tolerance.
- `knapsack`: the corners whose drift is bought with the lap budget `eps_lap_s`, the most
  drift metres for the seconds lost (0/1, exact over the loss in milliseconds). A corner whose
  drift gains is taken at no cost (and credits nothing).
- `Account`: debited by the map's predicted loss of each drift the car commits to (once a
  corner), never credited; on an open course the budget is per run.
"""
from dataclasses import dataclass
import json
import math
import os

LOSS_MAP = os.path.join(os.path.dirname(os.path.abspath(__file__)), "drift_losses.json")

TOL = {
    "r_tol": 1.25,           # a corner up to this beyond the measured radii (either way) is priced
    "turn_min": math.radians(30.0),   # turns below this are not corners (modes.CORNER_TURN)
}


@dataclass(frozen=True)
class Price:
    """A corner's drift, priced from the map: loss_s (s over the study's section; < 0 a gain),
    share (of the corner's zone the drift slides), and the cells it came from."""
    loss_s: float
    share: float
    cells: tuple
    forced: bool = False      # a bracketing cell's drift is forced (forced.py: no valley of its own)


class LossMap:
    """{class: [[R, turn_rad, width_m, loss_s or None, share or None(, forced)], ...]}: None
    where no drift was found (no valley, and the forced arm did not hold its floor); forced:
    the drift is forced.py's (the cell has no drift valley of its own). aliases: a class the
    grid did not run to one it did."""

    def __init__(self, cells, aliases=None, tol=None):
        self.cells = {c: [tuple(x) for x in v] for c, v in cells.items()}
        self.aliases = dict(aliases or {})
        self.tol = dict(TOL, **(tol or {}))

    @classmethod
    def load(cls, path=LOSS_MAP, tol=None):
        d = json.load(open(path))
        return cls(d["cells"], d.get("aliases"), tol)

    def save(self, path=LOSS_MAP, note=""):
        json.dump({"note": note, "cells": {c: [list(x) for x in v] for c, v in self.cells.items()},
                   "aliases": self.aliases}, open(path, "w"), indent=0)

    def classes(self):
        return set(self.cells) | set(self.aliases)

    def lookup(self, cls, R, turn, width=None):
        """The corner's Price (module docstring), or None: the class not in the map, a radius
        outside the measured ones by more than r_tol, or a bracketing cell with no drift."""
        c = self.aliases.get(cls, cls)
        if c not in self.cells or turn < self.tol["turn_min"]:
            return None
        rows = self.cells[c]
        ws = sorted({x[2] for x in rows})
        w = ws[0] if width is None or not math.isfinite(width) else max(
            [x for x in ws if x <= width + 1e-6] or [ws[0]])
        rows = [x for x in rows if x[2] == w]
        radii = sorted({x[0] for x in rows})
        turns = sorted({x[1] for x in rows})
        rs = _bracket(radii, R, self.tol["r_tol"])
        ts = _bracket(turns, turn, math.inf)
        if not rs or not ts:
            return None
        got = [x for x in rows if x[0] in rs and x[1] in ts]
        if len(got) < len(rs) * len(ts) or any(x[3] is None for x in got):
            return None
        return Price(max(x[3] for x in got), min(x[4] for x in got),
                     tuple((x[0], round(math.degrees(x[1])), x[2]) for x in got),
                     any(len(x) > 5 and bool(x[5]) for x in got))


def _bracket(vals, v, tol):
    """The measured values that bracket v (one if v is on one, or beyond the ends within the
    ratio tol); () if v lies further out."""
    if not vals:
        return ()
    for a in vals:
        if abs(a - v) <= 1e-6 * max(1.0, abs(a)):
            return (a,)
    if v < vals[0]:
        return (vals[0],) if v * tol >= vals[0] else ()
    if v > vals[-1]:
        return (vals[-1],) if v <= vals[-1] * tol else ()
    hi = next(a for a in vals if a > v)
    lo = max(a for a in vals if a < v)
    return (lo, hi)


def knapsack(items, budget_s, quantum=1e-3):
    """The items bought: items is [(key, loss_s, value)], budget_s the seconds to spend. An item
    whose loss is <= 0 is taken free (it credits nothing); the rest by an exact 0/1 knapsack
    over the loss in `quantum` steps (rounded up: never more than the budget), the most value,
    then the least loss. Returns (keys bought, seconds spent)."""
    free = [k for k, loss, _ in items if loss <= 0]
    paid = [(k, loss, v) for k, loss, v in items if loss > 0]
    cap = int(math.floor(budget_s / quantum + 1e-9))
    best = {0: (0.0, 0.0, ())}            # weight -> (value, -loss, keys)
    for k, loss, v in paid:
        wq = int(math.ceil(loss / quantum - 1e-9))
        if wq > cap:
            continue
        for w0, (val, nl, keys) in sorted(best.items(), reverse=True):
            w1 = w0 + wq
            if w1 > cap:
                continue
            cand = (val + v, nl - loss, keys + (k,))
            if w1 not in best or cand[:2] > best[w1][:2]:
                best[w1] = cand
    val, nl, keys = max(best.values(), key=lambda x: (x[0], x[1]))
    return free + list(keys), -nl


class Account:
    """The lap's drift account (design call 3): `budget_s` to spend; `reserve` the knapsack's
    corners (key -> predicted loss, held for them); each corner debited once, by its predicted
    loss (> 0 only: a gain credits nothing)."""

    def __init__(self, budget_s, reserve=None):
        self.budget = float(budget_s)
        self.reserved = {k: max(0.0, v) for k, v in (reserve or {}).items()}
        self.spent = 0.0
        self.debits = []

    @property
    def remaining(self):
        return self.budget - self.spent

    def free(self):
        """What is left beyond the corners still reserved."""
        return self.remaining - sum(self.reserved.values())

    def debited(self, key):
        return any(d["key"] == key for d in self.debits)

    def debit(self, key, loss_s, why=""):
        """The corner `key` committed to its drift at predicted loss_s; returns the debit (0 for
        a gain, or a corner already debited)."""
        if self.debited(key):
            return 0.0
        amt = max(0.0, float(loss_s))
        self.reserved.pop(key, None)
        self.spent += amt
        self.debits.append({"key": key, "loss_s": float(loss_s), "debit": amt, "why": why,
                            "remaining": self.remaining})
        return amt


ZONE_FROM = 30.0        # m before a corner's turn-in its zone starts (multistart.CORNER_FROM)


class CoursePrices:
    """A course's own corners priced offline at the live settings (roadmap 065 M3; studies/
    mode_choice_m4.py `course-prices`): per corner (its turn-in s, m) the largest live loss over
    the gate's window positions that have a drift and the least share, or None where fewer
    than MIN_PRICED (and fewer than all) positions have one. The
    cell map's synthetic corners are flat and stand alone; a course's grade, its corners' run
    and its speeds change the price (Haruna: its downhill hairpins gain, its R 37 kink costs
    0.15-0.93 s against the map's 0.03-0.13), so where a course has been priced its own price
    is used, and the map only for corners it has none for."""

    TOL = 2.0               # m: a corner's turn-in matched to a priced one within this
    # positions with a drift a corner needs to be priced (roadmap 065 M4): the gate enters a
    # drift seen on two re-plans in a row, so two of the three approach positions are enough
    # (packed snow R 15 x 90 had none at 105 m back, drifts at 75 and 45 m); fewer, unpriced
    MIN_PRICED = 2

    def __init__(self, prices):
        self.prices = {float(k): v for k, v in prices.items()}

    @classmethod
    def load(cls, path):
        d = json.load(open(path))
        out = {}
        for c in d["corners"]:
            pos = c.get("positions") or []
            if not pos:
                continue                    # not priced (too near the start): the map's
            pr = [p for p in pos if "loss_s" in p]
            out[c["s_in"]] = (None if len(pr) < min(len(pos), cls.MIN_PRICED) else
                              Price(max(p["loss_s"] for p in pr), min(p["share"] for p in pr),
                                    ("course", c["s_in"]),
                                    all(p.get("drift_from") == "B" for p in pr)))
        return cls(out)

    def lookup(self, s_in):
        """(True, its Price or None) where the course priced the corner at s_in; (False, None)
        where it did not."""
        for k, v in self.prices.items():
            if abs(k - s_in) <= self.TOL:
                return True, v
        return False, None


class S2Lap:
    """S2's plan for a run (design 4.3; call 3): which corners the lap budget buys
    (`committed`: corner s_in -> Price, the knapsack's), which the live lex-eps gate may take
    (`lex`: corner s_in -> Price or None, on classes without fall-off, with eps_window), and the
    account. Built by `build`; the gate (multistart.MultiStart._choose_s2) reads it."""

    def __init__(self, style, committed=None, lex=None, account=None, priced=None):
        self.style = style
        self.eps, self.h = float(style.eps_window), float(style.exit_hyst)
        self.committed = dict(committed or {})
        self.lex = dict(lex or {})
        self.account = account or Account(style.eps_lap_s, reserve={
            k: p.loss_s for k, p in self.committed.items()})
        self.priced = dict(priced or {})          # every corner's price (or None), for the record

    def kind(self, c):
        """'budget' (bought offline), 'lex' (the live gate's), or None (S1's gate) for corner c."""
        if c is None:
            return None
        if c.s_in in self.committed:
            return "budget"
        if c.s_in in self.lex:
            return "lex"
        return None

    def price(self, c):
        return self.committed.get(c.s_in) or self.lex.get(c.s_in)

    @classmethod
    def build(cls, style, corners, corner_classes, lossmap, flat, lap=None, course=None):
        """corners: modes.Corner list; corner_classes(c) -> ({class: probability}, width m);
        lossmap: LossMap; flat(cls) -> True where the class has no fall-off past its peak.
        A corner's price is its worst probable class's (None if any has none); the knapsack
        spends style.eps_lap_s on the priced ones, the drift metres (share x zone length) its
        value. With eps_window set, every corner whose classes are all flat and not bought is
        the lex gate's. lap: the budget per lap of this length (m; corners grouped by the lap
        their turn-in is on, each lap its own knapsack, the account eps_lap_s a lap), None: per
        run. course: CoursePrices, a corner's own price where it has one (the map's otherwise).
        A course price is per lap position, so it applies to the lap it was made on: on laps
        after the first the corner is matched by its turn-in less the lap's length."""
        priced, items, flat_corners = {}, [], set()
        for c in corners:
            probs, width = corner_classes(c)
            has, p = (False, None) if course is None else course.lookup(
                c.s_in if not lap else c.s_in % lap)
            if not has:
                ps = [lossmap.lookup(k, c.R, c.turn, width) for k in probs]
                p = (None if not ps or any(x is None for x in ps)
                     else max(ps, key=lambda x: x.loss_s))
            priced[c.s_in] = p
            if p is not None:
                items.append((c.s_in, p.loss_s, p.share * (c.s_out - c.s_in + ZONE_FROM)))
            if probs and all(flat(k) for k in probs):
                flat_corners.add(c.s_in)
        groups = {}
        for it in items:
            groups.setdefault(0 if not lap else int(it[0] // lap), []).append(it)
        keys = []
        for g in groups.values():
            keys += knapsack(g, style.eps_lap_s)[0]
        committed = {k: priced[k] for k in keys}
        lex = ({c.s_in: priced[c.s_in] for c in corners
                if c.s_in in flat_corners and c.s_in not in committed}
               if style.eps_window > 0 else {})
        n = 1 if not lap or not corners else int(max(c.s_in for c in corners) // lap) + 1
        account = Account(style.eps_lap_s * n,
                          reserve={k: p.loss_s for k, p in committed.items()})
        return cls(style, committed, lex, account=account, priced=priced)

    def summary(self):
        return {"eps_lap_s": self.style.eps_lap_s, "eps_window": self.eps,
                "committed": {round(k, 1): round(p.loss_s, 4) for k, p in self.committed.items()},
                "predicted_s": round(sum(max(0.0, p.loss_s) for p in self.committed.values()), 4),
                "lex": sorted(round(k, 1) for k in self.lex),
                "priced": {round(k, 1): (None if p is None else round(p.loss_s, 4))
                           for k, p in self.priced.items()}}
