"""What the planner knows of the surface ahead, and how wrong it may be (roadmap 061 M3b; the live
planner's design, section 4.4a "Where the surface comes from"; docs/preconditions.md).

No sensor on the car sees friction 200-260 m ahead and none senses the fall-off past the peak, so
over L1's window the surface is the map's: zones of swb's surface classes, each declared with a
confidence. In simulation the world can disagree with the map on purpose, so each zone also has
a true class, the one the plant (sim.Plant) drives on:

- `SurfaceMap`: the zones (declared class, confidence, true class) over a base class. It gives
  each node's class probabilities, the friction L1 plans with (the mixture's lower quantile,
  mean - k sigma, never below the lowest probable class's), the planner's Track (that friction,
  the most probable class's tyre: its peak and fall-off) and the true Track (the plant's);
- `Perception`: a simulated camera classifier: it reads the true class of each patch within its
  reach through a confusion matrix whose accuracy falls with distance (about 95 % within 30 m,
  chance by 100 m); seeded. It may confirm or veto a drift commit and lower the planned friction
  near the car; it never adds a start (it sees too late for that);
- `FrictionEstimate`: the car's own estimate as the planner may use it: it only lowers, holds
  low through a slide (no rise mid-slide: 060 M6's rose and made a x0.7 patch a crash), and is
  released slowly after; its log writes corrections into the map after a pass.

None of this is on by default: a Track without a map is planned as 060 plans it.
"""
from dataclasses import dataclass, replace
import math

import numpy as np

from race_planner.track import TYRE_FIELDS

K_SIGMA = 1.0         # the planned friction: the mixture's mean less this many sigmas
P_FLOOR = 0.15        # classes this probable or more bound the planned friction from below
CLASS_SD = 0.0        # each class's own spread (Track mu units); 0: a certain map plans as 060


@dataclass
class Zone:
    s0: float
    s1: float
    declared: str                 # the class the map says
    confidence: float = 1.0       # its probability; the rest spread over `alt`
    true: str = None              # the class the world has (None: the declared one)
    alt: tuple = None             # the other classes it may be (None: every other class)


class SurfaceMap:
    """Zones of `table`'s classes (track.SurfaceTable) along s over `base` (the table's
    reference by default), for `car` (its tyre friction scales every class's mu)."""

    def __init__(self, table, car, zones=(), base=None, k=K_SIGMA, sd=CLASS_SD, p_floor=P_FLOOR):
        self.table, self.car = table, car
        self.zones = [z if isinstance(z, Zone) else Zone(*z) for z in zones]
        self.base = base or table.reference
        self.k, self.sd, self.p_floor = k, sd, p_floor
        self.classes = list(table.mu_of)
        self.index = {c: i for i, c in enumerate(self.classes)}
        self.mu_c = np.array([table.mu(c, car) for c in self.classes])     # Track's mu units

    def zone_at(self, s):
        """The last zone listed that holds s (later zones overwrite earlier ones), or None."""
        for z in reversed(self.zones):
            if z.s0 <= s <= z.s1:
                return z
        return None

    def _row(self, z):
        p = np.zeros(len(self.classes))
        if z is None:
            p[self.index[self.base]] = 1.0
            return p
        others = [c for c in (z.alt or self.classes) if c != z.declared]
        p[self.index[z.declared]] = z.confidence
        if others and z.confidence < 1.0:
            for c in others:
                p[self.index[c]] += (1.0 - z.confidence) / len(others)
        else:
            p[self.index[z.declared]] = 1.0
        return p

    def probs(self, s):
        """Each node's class probabilities (n x classes, rows summing to 1)."""
        return np.array([self._row(self.zone_at(float(x))) for x in np.atleast_1d(s)])

    def true_class(self, s):
        """The class the world has at each node."""
        out = []
        for x in np.atleast_1d(s):
            z = self.zone_at(float(x))
            out.append(self.base if z is None else (z.true or z.declared))
        return out

    def friction(self, s):
        """At each node: the mixture's mean and sigma (Track mu units, each class's own spread
        `sd` included) and the friction planned with, mean - k sigma, at least the lowest
        class of probability p_floor or more less k sd (so an unlikely ice never sets it)."""
        P = self.probs(s)
        m = self.mu_c
        mean = P @ m
        var = P @ (self.sd ** 2 + m ** 2) - mean ** 2
        sigma = np.sqrt(np.maximum(var, 0.0))
        floor = np.array([np.min(m[p >= self.p_floor]) for p in P]) - self.k * self.sd
        q = np.where(sigma > 0, np.maximum(mean - self.k * sigma, floor), mean)
        return mean, sigma, q

    def planner_track(self, track):
        """`track` as L1 sees it through the map: each node's friction the planned quantile, its
        sliding share and tyre the most probable class's (the tyre at that friction, swb's rule:
        track.SurfaceTable.tyre's friction_scale). A node of one certain class gets exactly what
        Track.from_swb_course gives it."""
        P = self.probs(track.s)
        _, _, q = self.friction(track.s)
        top = [self.classes[int(np.argmax(p))] for p in P]
        return self._track(track, top, q)

    def true_track(self, track):
        """`track` as the world has it: each node's true class at its own friction (the
        plant's)."""
        cls = self.true_class(track.s)
        return self._track(track, cls, np.array([self.table.mu(c, self.car) for c in cls]))

    def _track(self, track, cls, mu):
        tab, car = self.table, self.car
        tyres, cache = [], {}
        for c, m in zip(cls, mu):
            key = (c, float(m))
            if key not in cache:
                cache[key] = tab.tyre(c, car, float(m) / tab.mu(c, car))
            tyres.append(cache[key])
        T = np.array(tyres).T
        return replace(track, mu=np.asarray(mu, float).copy(),
                       sliding=np.array([tab.sliding(c) for c in cls]),
                       **{k: T[i].copy() for i, k in enumerate(TYRE_FIELDS)})

    def nearest_class(self, mu):
        """The class whose friction (Track units) is nearest mu."""
        return self.classes[int(np.argmin(np.abs(self.mu_c - mu)))]

    def correct(self, log, tol=0.05, confidence=0.9, min_len=10.0):
        """After a pass: where the car's estimate (`log`: (s, friction in Track units) pairs,
        FrictionEstimate.log) stayed more than `tol` below what the map plans with, over at
        least min_len metres, a zone declaring the class nearest the lowest estimate there
        (the world's class kept as it was). It only lowers: an estimate above the map changes
        nothing. Returns the zones added."""
        if not log:
            return []
        s, mu = (np.array(v, float) for v in zip(*sorted(log)))
        _, _, q = self.friction(s)
        low = mu < q - tol
        added, i = [], 0
        while i < len(s):
            if not low[i]:
                i += 1
                continue
            j = i
            while j + 1 < len(s) and low[j + 1]:
                j += 1
            if s[j] - s[i] >= min_len:
                lo = float(np.min(mu[i:j + 1]))
                truth = self.true_class([0.5 * (s[i] + s[j])])[0]
                z = Zone(float(s[i]), float(s[j]), self.nearest_class(lo), confidence, truth)
                self.zones.append(z)
                added.append(z)
            i = j + 1
        return added


class Perception:
    """A simulated surface classifier (a camera's): the true class of each patch (a run of one
    true class) within `reach` of the car, read through a confusion matrix: right with
    probability accuracy(d) at distance d to the patch's nearest point (acc_near within `near`
    m, falling linearly to chance, one in the number of classes, at `far`), otherwise one of the
    other classes at random. One read per patch per call; seeded."""

    def __init__(self, smap, seed=0, near=30.0, far=100.0, acc_near=0.95, reach=60.0):
        self.map, self.near, self.far, self.acc_near, self.reach = smap, near, far, acc_near, reach
        self.rng = np.random.default_rng(seed)
        self.chance = 1.0 / len(smap.classes)

    def accuracy(self, d):
        d = np.asarray(d, float)
        f = np.clip((d - self.near) / (self.far - self.near), 0.0, 1.0)
        return self.acc_near + (self.chance - self.acc_near) * f

    def read(self, s_car, s):
        """The class read at each node s (None beyond reach or behind the car)."""
        s = np.atleast_1d(np.asarray(s, float))
        truth = self.map.true_class(s)
        out = [None] * len(s)
        i = 0
        while i < len(s):
            j = i
            while j + 1 < len(s) and truth[j + 1] == truth[i]:
                j += 1
            ahead = [k for k in range(i, j + 1) if 0.0 <= s[k] - s_car <= self.reach]
            if ahead:
                d = s[ahead[0]] - s_car
                c = truth[i]
                if self.rng.random() >= float(self.accuracy(d)):
                    others = [x for x in self.map.classes if x != c]
                    c = others[int(self.rng.integers(len(others)))]
                for k in ahead:
                    out[k] = c
            i = j + 1
        return out

    def scale(self, s_car, s, planned, near=None):
        """The planned friction's scale per node from what is read within `near` m (the
        sensor's own near range by default): a class read with less friction than planned
        lowers it there; nothing raises it."""
        near = self.near if near is None else near
        reads = self.read(s_car, s)
        out = np.ones(len(reads))
        for k, (c, x) in enumerate(zip(reads, np.atleast_1d(s))):
            if c is not None and x - s_car <= near:
                m = self.map.table.mu(c, self.map.car)
                if m < planned[k]:
                    out[k] = m / planned[k]
        return out


class FrictionEstimate:
    """The car's friction estimate as the planner uses it (a scale on the map's friction): it
    only lowers on a reading, holds while the car slides and `hold` s after, then rises back
    toward 1 at `release` per second. `log` keeps (s, friction in Track units) for
    SurfaceMap.correct after a pass."""

    def __init__(self, release=0.05, hold=1.0):
        self.release, self.hold = release, hold
        self.value, self.t, self.slide_t = 1.0, None, -math.inf
        self.log = []

    def update(self, t, raw=None, sliding=False, s=None, mu_map=None):
        """At time t: raw, a reading of the scale (None: none this tick); sliding, whether the car
        is past its tyres' peak (no rise then). Returns the scale. Called again at the same t it
        returns the same value."""
        if self.t is not None and t <= self.t:
            if raw is not None:
                self.value = min(self.value, raw)
            return self.value
        dt = 0.0 if self.t is None else t - self.t
        self.t = t
        if sliding:
            self.slide_t = t
        if raw is not None:
            self.value = min(self.value, raw)
        elif t - self.slide_t >= self.hold:
            self.value = min(1.0, self.value + self.release * dt)
        if s is not None and mu_map is not None:
            self.log.append((float(s), float(mu_map) * self.value))
        return self.value
