"""Which of L1's starts run at a re-plan, from what lies ahead (roadmap 061 M3b; the live
planner's design, section 4.4a; docs/mode-choice-m3b.md).

Grip and drift are separate valleys, and a start in the drift valley pays only where a drift can
win or a grip plan may fail: on dry asphalt almost nowhere (at most 0.4 %, tight hairpins), on
loose ground and snow in most corners (M2). So before each re-plan a cheap rule reads the window
ahead and picks the starts; the solver refines each. G (the grip plan, 060's one start) always
runs.

- `scan` reads the window: its corners (modes.corners: turn, least radius, distance ahead), each
  node's class probabilities and friction (surface.SurfaceMap; a track without a map is one
  certain class per node, matched from its friction and tyre), the grade and the width.
- `DriftMap` holds where a drift was measured to pay, per surface class (the drift-optimality
  study after M2: race_planner/drift_map.json, built by studies/drift_optimality.py
  `drift-map`): cells (R, turn, gain). A corner pays for a class if a cell that pays is no
  tighter and turns no further, within tolerances (a tighter, longer corner pays too).
- `Scheduler.starts` applies the table of design 4.4a to the corner the choice is about (the
  first whose turn-in is SEED_LEAD or more ahead: the corner the seeds go into), the union over
  every class at least p_add probable (loose in):
    D  where a drift pays (the map: above gain_min on a class without fall-off; on one that falls
       off, asphalt among them, only above gain_falloff, the switch threshold, and only up to
       the measured cell's radius), or on a steep
       downhill on low friction;
    H  with D, and wherever the window holds low friction (the fallback when G fails);
    F  with D into a hairpin on a surface without fall-off (tighter than F_R_DRY on asphalt-like
       friction, HAIRPIN_R elsewhere);
  no new D or F when the corner's turn-in is under AHEAD m (its trail braking is already cut);
  a D or F already sliding in it keeps its plan (multistart.MultiStart).
The thresholds are data (RULES), none per track.
"""
from dataclasses import dataclass, field
import json
import math
import os

import numpy as np

from race_planner import modes
from race_planner.track import TYRE_FIELDS

DRIFT_MAP = os.path.join(os.path.dirname(os.path.abspath(__file__)), "drift_map.json")

RULES = {
    "p_add": 0.15,          # a class this probable adds its row's starts (design: ~0.1-0.2)
    "gain_min": 0.05,       # %: a cell pays if its drift wins by this much (above the study's noise)
    "gain_falloff": 0.3,    # %: on a class that falls off past the peak (asphalt, gravel, packed
                            #   snow), only a cell whose drift clears the switch threshold pays
    "r_tol": 1.25,          # a corner up to this times a paying cell's radius pays
    "r_tol_falloff": 1.05,  # on a class that falls off: only the measured cell's radius (and the
                            #   corner finder's discretisation), not the stretch (roadmap 061
                            #   M4b: D into asphalt R 9.9-11.2 m hairpins cost 0.09 s on the plant)
    "turn_tol": math.radians(30.0),   # and turning down to this less than it
    "ahead": 25.0,          # m: no new D or F into a corner whose turn-in is nearer
    "mu_fallback": 0.45,    # a class's mu (swb's, dry asphalt 1.12) this low or lower: H runs
    "mu_dry": 0.9,          # and this high: asphalt-like (F only into the tightest hairpins)
    "flat": 0.95,           # a class's sliding share this high: no fall-off past the peak
    "f_r_dry": 9.0,         # m: F into a hairpin this tight on asphalt-like friction
    "downhill": -0.05,      # grade (+ climbs): steeper down than this on low friction adds D
    "approach": 60.0,       # m before the turn-in the corner's surface and grade are read from
}


class DriftMap:
    """Where a drift pays, per class: {class: [[R, turn_rad, gain_pct, width_m], ...]} of the
    cells where the free plan's rear axle ran 3 deg or more past its peak and beat the held plan
    beyond the noise, on a road `width_m` wide (the study's 8 m where the width arm did not run);
    `widths`: {class: the widths it was measured at}; `aliases` maps a class the study did not
    run to one it did; a class in neither is seeded as the most drift-friendly (unknown fall-off
    seeded as none: design 4.4a).

    Width (M3b's arm): a corner reads its class's cells at the widest measured width no wider
    than its road (a narrower road drifts more: loose in), the narrowest if its road is narrower
    than all or its width unknown."""

    def __init__(self, cells, aliases=None, rules=None, widths=None):
        self.cells = {c: [tuple(map(float, x)) + ((8.0,) if len(x) == 3 else ())
                          for x in v] for c, v in cells.items()}
        self.widths = {c: sorted(map(float, w)) for c, w in (widths or {}).items()}
        for c, v in self.cells.items():
            self.widths.setdefault(c, sorted({x[3] for x in v}) or [8.0])
        self.aliases = dict(aliases or {})
        self.rules = dict(RULES, **(rules or {}))

    @classmethod
    def load(cls, path=DRIFT_MAP, rules=None):
        d = json.load(open(path))
        return cls(d["cells"], d.get("aliases"), rules, d.get("widths"))

    def save(self, path=DRIFT_MAP, note=""):
        json.dump({"note": note, "cells": self.cells, "widths": self.widths,
                   "aliases": self.aliases}, open(path, "w"))

    def _cells(self, c, width=None):
        c = self.aliases.get(c, c)
        if c not in self.cells:                              # unknown: every class's, narrowest
            return [x for k in self.cells for x in self._cells(k, None)]
        ws = self.widths[c]
        w = ws[0] if width is None or not math.isfinite(width) else max(
            [x for x in ws if x <= width + 1e-6] or [ws[0]])
        return [x for x in self.cells[c] if x[3] == w]

    def pays(self, c, R, turn, width=None, falloff=False):
        """Whether a drift pays in a corner of radius R (m), turning `turn` (rad), on class c, on a
        road `width` m wide (None: unknown, the narrowest measured). falloff: the class falls off
        past its peak, so its cell must clear gain_falloff (the switch threshold: a D start whose
        gain the gate would never take is a core spent for nothing where G does not fail), not
        gain_min (roadmap 061 M4's tightened asphalt rule), and the corner must be no wider in
        radius than the cell itself (r_tol_falloff, M4b: between the measured R 9 and R 15 the
        gain is the model's interpolation, and on the plant D into R 10-11 m asphalt hairpins
        lost time), not up to r_tol x it."""
        r = self.rules
        g_min = r["gain_falloff"] if falloff else r["gain_min"]
        r_tol = r["r_tol_falloff"] if falloff else r["r_tol"]
        return any(g >= g_min and R <= Rm * r_tol and turn >= tm - r["turn_tol"]
                   for Rm, tm, g, _ in self._cells(c, width))


@dataclass
class CornerAhead:
    corner: object            # modes.Corner
    dist: float               # m from the window's start to its turn-in
    probs: dict               # class -> its largest probability over the corner and approach
    grade: float              # the steepest descent over the approach and the corner (< 0 down)
    width: float              # the road's least width there, m


@dataclass
class Scan:
    s0: float
    corners: list = field(default_factory=list)   # CornerAhead, in order
    probs: dict = field(default_factory=dict)     # class -> largest probability in the window
    mu_mean: np.ndarray = None
    mu_sigma: np.ndarray = None
    mu_q: np.ndarray = None


def node_classes(track, table, car):
    """Each node's class for a track with no map: the class whose friction, sliding share and
    tyre are nearest the node's (a track built from swb's classes matches exactly)."""
    names = list(table.mu_of)
    ref = np.array([[table.mu(c, car), table.sliding(c), *table.tyre(c, car)] for c in names])
    have = np.vstack([track.mu, track.sliding, *(getattr(track, k) for k in TYRE_FIELDS)]).T
    scale = np.maximum(np.abs(ref).max(axis=0), 1e-9)
    d = (((have[:, None, :] - ref[None, :, :]) / scale) ** 2).sum(axis=2)
    return [names[i] for i in np.argmin(d, axis=1)]


def scan(track, s0, s1, corners, probs, friction=None):
    """The window [s0, s1] of `track` (the planner's): the corners whose exit is ahead of s0 and
    turn-in before s1, with their class probabilities (probs(s) -> {class: p per node}), grade
    and width; the window's class probabilities; friction (mean, sigma, quantile per node) if
    given (SurfaceMap.friction)."""
    sel = (track.s >= s0) & (track.s <= s1)
    sw = track.s[sel]
    pw = probs(sw)
    out = Scan(s0, probs={c: float(np.max(v)) for c, v in pw.items()})
    if friction is not None:
        out.mu_mean, out.mu_sigma, out.mu_q = friction(sw)
    a = RULES["approach"]
    for c in corners:
        if c.s_out < s0 or c.s_in > s1:
            continue
        m = (track.s >= c.s_in - a) & (track.s <= c.s_out)
        pc = probs(track.s[m])
        out.corners.append(CornerAhead(
            c, float(c.s_in - s0), {k: float(np.max(v)) for k, v in pc.items()},
            float(np.min(track.grade[m])) if m.any() else 0.0,
            float(np.min(track.w_left[m] + track.w_right[m])) if m.any() else math.inf))
    return out


class Scheduler:
    """The starts for each re-plan (module docstring). table: track.SurfaceTable (each class's
    mu and sliding share); dmap: DriftMap; smap: surface.SurfaceMap, or None (each node one
    class, matched from the track); always: every start every re-plan (the always-on arm, F into
    hairpins), for comparison."""

    def __init__(self, track, car, table, dmap=None, smap=None, grip=1.0, always=False,
                 rules=None):
        self.track, self.table, self.smap, self.always = track, table, smap, always
        self.dmap = dmap or DriftMap.load()
        self.rules = dict(self.dmap.rules, **(rules or {}))
        self.dmap.rules = self.rules
        self.corners = modes.corners(track, grip)
        if smap is None:
            cls = node_classes(track, table, car)
            names = list(table.mu_of)
            onehot = {c: (np.array(cls) == c).astype(float) for c in names}
            self._probs = lambda s: {c: np.interp(s, track.s, v) for c, v in onehot.items()}
            self.friction = None
        else:
            self._probs = lambda s: dict(zip(smap.classes, smap.probs(s).T))
            self.friction = smap.friction

    def scan(self, s0, s1):
        return scan(self.track, s0, s1, self.corners, self._probs, self.friction)

    def low(self, c):
        return self.table.mu_of[c] <= self.rules["mu_fallback"]

    def corner_starts(self, ca):
        """The starts a corner ahead asks for (a set of 'H', 'D', 'F') and why."""
        r, out, why = self.rules, set(), []
        hair = ca.corner.turn >= modes.HAIRPIN_TURN
        for c, p in ca.probs.items():
            if p < r["p_add"]:
                continue
            flat = self.table.sliding(c) >= r["flat"]
            d = self.dmap.pays(c, ca.corner.R, ca.corner.turn, ca.width, falloff=not flat)
            if not d and self.low(c) and ca.grade < r["downhill"]:
                d = True
                why.append(f"{c} downhill")
            if d:
                out |= {"H", "D"}
                why.append(f"{c} pays")
                r_f = r["f_r_dry"] if self.table.mu_of[c] >= r["mu_dry"] else modes.HAIRPIN_R
                if flat and hair and ca.corner.R <= r_f:
                    out.add("F")
            if self.low(c):
                out.add("H")
        return out, why

    def starts(self, scn, corner=None, running=()):
        """The starts for this re-plan, from scan `scn` and the corner the choice is about
        (multistart's corner_ahead; None: none). running: D or F starts whose own plan still
        slides in that corner (kept whatever the table says). Returns (set of slots, reasons)."""
        slots, why = {"G"}, []
        if self.always:
            slots |= {"H", "D"}
            if corner is not None and corner.hairpin:
                slots.add("F")
            return slots, ["always"]
        if any(p >= self.rules["p_add"] and self.low(c) for c, p in scn.probs.items()):
            slots.add("H")
            why.append("low friction in the window")
        if corner is not None:
            ca = next((x for x in scn.corners if x.corner.s_in == corner.s_in), None)
            if ca is not None:
                add, w = self.corner_starts(ca)
                if ca.dist < self.rules["ahead"]:
                    add -= {"D", "F"}
                    w.append("too near for a new drift start")
                slots |= add
                why += w
        slots |= set(running)
        return slots, why
