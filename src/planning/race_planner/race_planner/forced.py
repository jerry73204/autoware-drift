"""A forced drift: the rear axle held past its tyre's peak over a stretch of the window, so that a
drift can be priced where it is not the window's own optimum (roadmap 065 M2, S2's signed map;
and S2's bought corners live).

Grip and drift are separate valleys, but where a drift loses time its valley is usually not a
local minimum of the free window at all: from a drift seed IPOPT slides back to grip (065 E2,
docs/drift-style-m2.md). To price such a drift, and to drive one the budget bought, the window
needs a floor: on the stretch, sign * rear slip angle <= -angle (the rear out, the corner's way).

No new window: a Window built with `rear_alpha_max = -angle` (`floor_kwargs`) has, per stage,
the two linear rows |vy - lr r| <= tan(rear_alpha_max) vx, which with a negative bound read

    row 1:  (vy - lr r) + tan(angle) vx <= 0     the rear out to the right of the velocity:
                                                 a left turn's floor
    row 2:  (vy - lr r) - tan(angle) vx >= 0     a right turn's floor

and contradict each other. `set_floor` sets their bounds per stage before a solve: on the
stretch the turn's row is active and the other open; off it both are open. The window's slip
caps, edge margins and every other row are untouched (a floor only adds a requirement; nothing
of design 065 section 5 is loosened), and the first `free_stages` stages stay free as the held
rows' do. A window carries one angle (compiled once); `angle_for` rounds a peak plus the past up
to whole degrees, so a few windows serve every surface.
"""
import math

import numpy as np


def floor_kwargs(angle):
    """Window kwargs for a floor at `angle` (rad > 0)."""
    if not angle > 0:
        raise ValueError(f"'angle': a floor past the peak is > 0, not {angle!r}")
    return {"rear_alpha_max": -float(angle)}


def angle_for(peak, past_deg):
    """The floor (rad) for a rear tyre peaking at `peak` (rad), `past_deg` past it, rounded up
    to a whole degree."""
    return math.radians(math.ceil(math.degrees(peak) + past_deg - 1e-9))


def set_floor(w, s, a, b, sign):
    """Bounds of window w's floor rows (w built with floor_kwargs and no beta_max) for a solve:
    active on the stages whose node s[k] lies in [a, b] (and k >= w.free_stages), the turn's
    row (sign 1 left, -1 right); open everywhere else. a > b opens every stage (no floor)."""
    if w.beta_max is not None or w.rear_alpha_max is None or w.rear_alpha_max >= 0:
        raise ValueError("'window': set_floor needs a window built with floor_kwargs only")
    M = w.M
    per = (len(w.lbg_all) - 2) // M
    fs = getattr(w, "free_stages", 0)
    for k in range(M):
        i = k * per + per - 2
        on = a <= float(s[k]) <= b and k >= fs
        w.lbg_all[i], w.ubg_all[i] = -np.inf, (0.0 if on and sign > 0 else np.inf)
        w.lbg_all[i + 1], w.ubg_all[i + 1] = (0.0 if on and sign < 0 else -np.inf), np.inf
    return w


def floor_met(car, X, s, a, b, sign, angle, tol=1e-3):
    """Whether plan X (over nodes s) holds the floor on [a, b]: sign * rear slip angle <=
    -angle there (within tol rad)."""
    from race_planner.modes import rear_slip
    on = (np.asarray(s) >= a) & (np.asarray(s) <= b)
    return bool(np.all(sign * rear_slip(car, X)[on] <= -angle + tol))
