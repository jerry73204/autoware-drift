"""Grip or drift: the mode seeds L1 is started from, a plan's mode, a track's corners and the
library of corner archetypes (roadmap 061; docs/mode-choice-methods.md, mode-choice-m1.md,
mode-choice-m3.md).

Grip and drift are separate valleys of L1's problem, and a derivative solver stays in the valley
it starts in. So the live loop (multistart.py) solves each window from a start in each: the grip
plan, a rear-out seed for the corner ahead, and for a tight hairpin a flick. A seed is a plan
with its body turned so that the rear axle runs past its tyre's peak through the corner
(`reshape_slip`): the path and the speed kept, the heading giving way to the sideslip. Where it
slides comes from one of two places:

- the library (`Library`): each corner archetype of the drift study (radius x turn x friction x
  fall-off) solved offline by fall-off continuation (no fall-off first, then the surface's, each
  step from the last: studies/drift_optimality.py's Dc), its rear axle's slip, its line and its
  request kept along the corner and mapped onto the corner ahead (aligned at the turn-in and the
  exit, the corner's length stretched, mirrored for a right turn);
- a synthetic profile (`seed_rear_out`) where no archetype is near enough: the rear SEED_PAST
  past its peak from D_FROM before the turn-in to the apex.

A plan is a drift if its rear axle runs REAR_PAST or more past its tyre's peak anywhere (the
study's definition; the live loop asks it of the corner ahead only, multistart.MultiStart.classify).
"""
from dataclasses import dataclass
import math
import os

import numpy as np

from race_planner.track import TYRE_FIELDS

REAR_PAST = math.radians(3.0)      # a plan's rear axle this far past its tyre's peak slides
SEED_PAST = math.radians(7.5)      # the rear-out seeds' rear axle past its tyre's peak
FLICK_PAST = math.radians(5.0)     # the flick's first slide, the other way
D_FROM = 25.0                      # m before turn-in the rear-out seed starts sliding
D_BRAKE = -0.3                     # m/s^2: its request on low friction (mu < 0.7); 0 above
CORNER_R = 150.0                   # m: a stretch tighter than this is a corner
CORNER_TURN = math.radians(30.0)   # and turning at least this much
CORNER_GAP = 12.0                  # m: two stretches of the same hand closer than this are one
HAIRPIN_R, HAIRPIN_TURN = 15.0, math.radians(150.0)   # a flick is tried only in these
LIBRARY = os.path.join(os.path.dirname(os.path.abspath(__file__)), "corner_library.npz")


_PEAKS = {}


def _rear_peak(car, mu, sliding, surface=None):
    key = (repr(car), mu, sliding, surface)
    if key in _PEAKS:
        return _PEAKS[key]
    from race_planner import model
    fz0 = model.rest_loads(car)[2]
    fz = (1.0 + 0.6 * mu) * fz0
    a = np.radians(np.linspace(0.05, 40.0, 800))
    fy = np.array([-float(model.tyre(car, 0.0, math.tan(x), fz, fz0, mu, sliding, surface)[1])
                   for x in a])
    _PEAKS[key] = float(a[int(np.argmax(fy >= 0.995 * fy.max()))])
    return _PEAKS[key]


def rear_peak(car, mu, sliding, surface=None):
    """The rear tyre's peak slip angle (rad) in the planner's tyre (model.tyre, no longitudinal
    slip) at the outer rear wheel's load at the limit (1 + 0.6 mu rest loads: the lateral
    transfer at mu g): the smallest angle at which its lateral force is within 0.5 % of its
    largest. mu is the planner's (grip times the road's); surface the road's tyre (model.tyre's;
    None the rig's). Cached on mu to 0.01, sliding to 0.05, the surface's shares to 0.001."""
    sf = None if surface is None else tuple(round(float(v), 3) for v in surface)
    if sf == (1.0, 1.0, 1.0, 1.0):
        sf = None                              # the rig's: one entry, today's numbers
    return _rear_peak(car, round(float(mu), 2), round(float(sliding) * 20) / 20, sf)


def rear_slip(car, X):
    """The rear axle's slip angle, atan((vy - lr r) / vx), at every node of X (6 x n)."""
    return np.arctan2(X[1] - car.lr * X[2], np.maximum(X[0], 0.1))


def past_peak(car, X, mu, sliding, surface=None):
    """How far the rear axle runs past its tyre's peak at every node (rad; negative within it),
    mu and sliding per node (the planner's mu: grip times the road's), surface the road's tyre
    per node (4 x n, model.tyre's; None the rig's)."""
    n = X.shape[1]
    mu, sliding = np.broadcast_to(mu, n)[:n], np.broadcast_to(sliding, n)[:n]
    sfs = [None] * n if surface is None else np.asarray(surface, float)[:, :n].T
    peak = np.array([rear_peak(car, m, sl, sf) for m, sl, sf in zip(mu, sliding, sfs)])
    return np.abs(rear_slip(car, X)) - peak


# ---- the seeds (moved here from studies/drift_optimality.py, M1's, unchanged) ----

def _zone(s, a, b, ramp_in, ramp_out):
    """1 on [a, b], ramped linearly to 0 over ramp_in before and ramp_out after."""
    return np.clip(np.minimum((s - (a - ramp_in)) / ramp_in, ((b + ramp_out) - s) / ramp_out),
                   0.0, 1.0)


def _consistent_u(X, U, t, car, w=None, a_req=None):
    """U (7 x M) made to match states X: the steering rate from delta, the turn's acceleration,
    and where w > 0 the request blended to a_req and the wheels' speeds for it (wheel_speeds)."""
    from race_planner.window import wheel_speeds
    M = X.shape[1] - 1
    s = t.s[:X.shape[1]]
    U = np.array(U, float).copy()
    rate = car.steer_rate_max or 1.0
    U[0] = np.clip(np.gradient(X[5], s)[:M] * X[0, :M], -rate, rate)
    U[3] = X[0, :M] * X[2, :M]
    if w is not None:
        wm = w[:M]
        if a_req is not None:
            a_req = np.broadcast_to(a_req, X.shape[1])[:M] if np.ndim(a_req) else a_req
            U[1] = (1 - wm) * U[1] + wm * a_req
            U[2] = (1 - wm) * U[2] + wm * a_req
        ws = wheel_speeds(car, X[:, :M], U[1])
        U[4:7] = (1 - wm) * U[4:7] + wm * ws
    return U


def reshape_slip(X, U, t, car, target, w, a_req=None):
    """A seed from plan (X, U): the body turned so that the rear axle's slip angle is `target`
    (rad, per node) where w is 1, blended where 0 < w < 1, untouched where w is 0. The path and
    the speed are kept (the velocity's heading to the track, xi + beta, and |v|); the heading xi
    gives way to the new sideslip, the yaw rate follows from it (sdot (dxi/ds + kappa)), and the
    steering keeps the front axle's slip angle the plan had. The request goes to a_req in the
    zone (None: kept)."""
    N = X.shape[1]
    s, kap = t.s[:N], t.kappa[:N]
    vx, vy, r, n, xi, delta = (np.array(X[i], float) for i in range(6))
    v = np.hypot(vx, vy)
    beta0 = np.arctan2(vy, vx)
    chi = xi + beta0
    front0 = np.arctan2(vy + car.lf * r, vx) - delta
    ar0 = np.arctan2(vy - car.lr * r, vx)
    goal = ar0 + w * (target - ar0)
    beta = beta0.copy()
    on = w > 0
    for _ in range(60):
        xin = chi - beta
        sdot = v * np.cos(chi) / (1 - n * kap)
        rn = sdot * (np.gradient(xin, s) + kap)
        ar = np.arctan2(v * np.sin(beta) - car.lr * rn, v * np.cos(beta))
        beta = np.where(on, np.clip(beta + 0.4 * (goal - ar), -0.9, 0.9), beta0)
    xin = chi - beta
    rn = np.where(on, v * np.cos(chi) / (1 - n * kap) * (np.gradient(xin, s) + kap), r)
    vxn, vyn = v * np.cos(beta), v * np.sin(beta)
    dn = np.clip(np.arctan2(vyn + car.lf * rn, vxn) - front0, -car.steer_max, car.steer_max)
    dn = np.where(on, dn, delta)
    Xn = np.vstack([vxn, vyn, rn, n, np.where(on, xin, xi), dn])
    Xn[:, 0] = X[:, 0]
    return Xn, _consistent_u(Xn, U, t, car, w, a_req)


def seed_rear_out(X, U, t, s_in, s_out, car, peak, mu, sgn=None):
    """Ds: the rear SEED_PAST past its peak from D_FROM before turn-in to the apex (ramped in
    over 5 m, out over 10), braking on low friction, coasting on asphalt. sgn: the corner's hand
    (1 left); by default the whole section's."""
    s = t.s[:X.shape[1]]
    sgn = sgn or np.sign(t.kappa.sum()) or 1.0
    apex = 0.5 * (s_in + s_out)
    w = _zone(s, s_in - D_FROM, apex, 5.0, 10.0)
    target = -sgn * (peak + SEED_PAST) * np.ones_like(s)
    return reshape_slip(X, U, t, car, target, w, D_BRAKE if mu < 0.7 else 0.0)


def seed_flick(X, U, t, s_in, s_out, car, peak, mu, sgn=None):
    """F: the rear out the other way first (FLICK_PAST past the peak, 35 to 18 m before
    turn-in), then as Ds from 10 m before turn-in to the apex."""
    s = t.s[:X.shape[1]]
    sgn = sgn or np.sign(t.kappa.sum()) or 1.0
    apex = 0.5 * (s_in + s_out)
    w1 = _zone(s, s_in - 35.0, s_in - 18.0, 4.0, 4.0)
    w2 = _zone(s, s_in - 10.0, apex, 4.0, 10.0)
    target = np.where(w1 > w2, sgn * (peak + FLICK_PAST), -sgn * (peak + SEED_PAST))
    return reshape_slip(X, U, t, car, target, np.maximum(w1, w2),
                        D_BRAKE if mu < 0.7 else 0.0)


# ---- corners ----

@dataclass
class Corner:
    s_in: float          # where the curvature passes 1 / CORNER_R going in
    s_out: float         # and coming out
    sign: float          # 1 left, -1 right
    R: float             # its tightest radius, m
    turn: float          # how far it turns, rad
    mu: float            # the planner's friction in it (grip times the road's, its lowest)
    sliding: float
    surface: tuple = None  # the road's tyre at its lowest friction (model.tyre's; None the rig's)

    @property
    def hairpin(self):
        return self.R <= HAIRPIN_R and self.turn >= HAIRPIN_TURN


def corners(track, grip=1.0):
    """The track's corners: stretches tighter than CORNER_R (the curvature smoothed over 12 m),
    those of one hand less than CORNER_GAP apart joined, each turning at least CORNER_TURN."""
    s, k = track.s, track.kappa
    ds = float(s[1] - s[0])
    m = max(1, int(round(12.0 / ds)))
    ks = np.convolve(k, np.ones(m) / m, "same")
    on = np.abs(ks) > 1.0 / CORNER_R
    runs, i = [], 0
    while i < len(s):
        if not on[i]:
            i += 1
            continue
        j = i
        while j + 1 < len(s) and on[j + 1] and np.sign(ks[j + 1]) == np.sign(ks[i]):
            j += 1
        runs.append([i, j, float(np.sign(ks[i]))])
        i = j + 1
    joined = []
    for r in runs:
        if joined and r[2] == joined[-1][2] and s[r[0]] - s[joined[-1][1]] < CORNER_GAP:
            joined[-1][1] = r[1]
        else:
            joined.append(r)
    out = []
    for i, j, sg in joined:
        sl = slice(i, j + 1)
        turn = abs(float(np.trapezoid(k[sl], s[sl]))) if j > i else 0.0
        if turn < CORNER_TURN:
            continue
        lo = i + int(np.argmin(track.mu[sl]))
        sf = tuple(float(getattr(track, f)[lo]) for f in TYRE_FIELDS)
        out.append(Corner(float(s[i]), float(s[j]), sg, float(1.0 / np.max(np.abs(k[sl]))), turn,
                          grip * float(np.min(track.mu[sl])), float(np.min(track.sliding[sl])),
                          None if sf == (1.0, 1.0, 1.0, 1.0) else sf))
    return out


# ---- the library ----

class Library:
    """Corner archetypes, each a drift plan of the study's (a left turn of radius R through
    `theta` on friction mu with fall-off sliding, entered on a straight at the study's speed):
    along s from the turn-in (s_rel) its rear axle's slip angle, its offset n and its request a,
    and the rear tyre's peak it was planned against. Built by `build_library` from M1's study-
    setting grid (play_log/race_planner/mode_choice/m1/multistart.*): the Dc plan of each case
    (fall-off continuation), kept only where it does slide."""

    def __init__(self, path=LIBRARY):
        d = np.load(path)
        self.meta = d["meta"]                  # R theta mu sliding s_out_rel peak, one row each
        self.s_rel, self.alpha, self.n, self.a = d["s_rel"], d["alpha"], d["n"], d["a"]

    def nearest(self, c, tol=1.2):
        """The archetype nearest corner c (radius in log steps of ~1.8, the turn in quarter
        turns, mu in 0.2, fall-off in 0.4), or None if none is within tol."""
        R, th, mu, sl = (self.meta[:, i] for i in range(4))
        d = (np.abs(np.log(c.R / R)) / 0.6 + np.abs(c.turn - th) / (math.pi / 2)
             + np.abs(c.mu - mu) / 0.2 + np.abs(c.sliding - sl) / 0.4)
        i = int(np.argmin(d))
        return i if d[i] <= tol else None

    def seed(self, i, X, U, t, c, car):
        """Dc: plan (X, U) on section t with archetype i's slide put into corner c: its rear
        axle's slip (its excess past the peak kept, its peak the corner's), its line (within the
        road) and its request (scaled by the friction), from 40 m before the turn-in to 20 m past
        the exit (ramped over 10 m), mapped on the turn-in and the exit."""
        R, th, mu_a, sl_a, out_a, peak_a = self.meta[i]
        s = t.s[:X.shape[1]]
        L = max(c.s_out - c.s_in, 1.0)
        sa = np.where(s < c.s_in, s - c.s_in,
                      np.where(s <= c.s_out, (s - c.s_in) * out_a / L, out_a + (s - c.s_out)))
        sr = self.s_rel[i]
        alpha = np.interp(sa, sr, self.alpha[i])          # a left turn's: negative in a slide
        n_a = np.interp(sa, sr, self.n[i])
        a_a = np.interp(sa, sr, self.a[i])
        peak = rear_peak(car, c.mu, c.sliding, c.surface)
        excess = np.abs(alpha) - peak_a
        mag = np.where(excess > 0, peak + excess, np.abs(alpha) * peak / peak_a)
        target = c.sign * np.sign(alpha) * mag
        w = _zone(s, c.s_in - 40.0, c.s_out + 20.0, 10.0, 10.0)
        w = np.where((sa >= sr[0]) & (sa <= sr[-1]), w, 0.0)
        half = 0.5 * car.width + 0.3
        Xn = np.array(X, float).copy()
        n_l = np.clip(c.sign * n_a, -(t.w_right[:len(s)] - half), t.w_left[:len(s)] - half)
        Xn[3] = (1 - w) * Xn[3] + w * n_l
        Xn[:, 0] = X[:, 0]
        return reshape_slip(Xn, U, t, car, target, w, a_a * c.mu / mu_a)


def build_library(m1_dir, car, section_bounds, path=LIBRARY):
    """The library from M1's study-setting grid (`multistart`.jsonl and .npz under m1_dir): each
    case's Dc plan (the fall-off continuation; on no fall-off the free plan from the cold start,
    which is what continuation starts from), where it slides (its rear REAR_PAST past the peak in
    the corner). section_bounds(R, theta): the study section's turn-in and exit. Stored along s
    from the turn-in on the study's 2 m grid, float32. Returns the number of archetypes kept."""
    import json
    recs = [json.loads(x) for x in open(os.path.join(m1_dir, "multistart.jsonl"))]
    data = np.load(os.path.join(m1_dir, "multistart.npz"))
    meta, s_rel, alpha, n, a = [], [], [], [], []
    for i, r in enumerate(recs):
        if "error" in r or f"{i}/Dc_X" not in data:
            continue
        s, X, U = data[f"{i}/s"], data[f"{i}/Dc_X"], data[f"{i}/Dc_U"]
        s_in, s_out = section_bounds(r["R"], r["theta"])
        sel = (s >= s_in - D_FROM - 10) & (s <= s_out + 20)
        peak = math.radians(r["peak"])
        ar = rear_slip(car, X)
        if np.abs(ar[sel]).max() - peak < REAR_PAST:
            continue
        meta.append([r["R"], r["theta"], r["mu"], r["sliding"], s_out - s_in, peak])
        s_rel.append(s - s_in)
        alpha.append(ar)
        n.append(X[3])
        a.append(np.concatenate([U[1], U[1][-1:]]))
    f32 = lambda v: np.array(v, np.float32)
    np.savez_compressed(path, meta=np.array(meta), s_rel=f32(s_rel), alpha=f32(alpha), n=f32(n),
                        a=f32(a))
    return len(meta)
