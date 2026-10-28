"""The minimum-curvature line: the path through the road's free width (its width less the car's
half width and a margin) with the least integral of squared curvature. A geometric line, not the
fastest one, but close to it on most corners; it is the quasi-steady-state baseline's line and the
minimum-time optimiser's warm start.

The line is the reference line offset along its left normal, p_i = r_i + a_i n_i, with a_i boxed
by the free width. The objective is the line's own: each point's Menger curvature (the circle
through it and its neighbours) squared, times the line's spacing there, so a line moved outward is
both straighter and longer, as it is. IPOPT (through CasADi) solves it with the exact, banded
Hessian. Two cheaper forms fail: with the spacing frozen (a quadratic), moving inward shrinks the
second differences and looks straighter, which on a circle takes the inside edge; Gauss-Newton on
the residuals drops the second-order term that makes the circle's outer edge best and stalls. A tiny pull to the
reference breaks ties on straights. Closed tracks wrap.
"""
from dataclasses import replace

import numpy as np
import casadi as ca

TIE = 1e-9


def _spacing(p, closed):
    d = np.diff(np.vstack([p, p[:1]]) if closed else p, axis=0)
    return np.hypot(d[:, 0], d[:, 1])


def _curvature(p, closed):
    """Signed Menger curvature at each point (> 0 turning left); exact on a circle."""
    n = len(p)
    idx = np.arange(n)
    im, ip = (idx - 1) % n, (idx + 1) % n
    a, b = p[idx] - p[im], p[ip] - p[idx]
    cross = a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]
    la, lb, lc = np.hypot(*a.T), np.hypot(*b.T), np.hypot(*(p[ip] - p[im]).T)
    k = 2.0 * cross / np.maximum(la * lb * lc, 1e-12)
    if not closed:
        k[0], k[-1] = k[1], k[-2]
    return k


def min_curvature_line(track, half_width, margin=0.3):
    """The line on `track` (as a Track, its widths measured from the line) and its offsets a (m,
    left of the reference line > 0), with the car's `half_width` and `margin` kept from either
    edge."""
    closed = track.closed
    r = np.column_stack([track.x, track.y])
    nrm = np.column_stack([-np.sin(track.psi), np.cos(track.psi)])
    lo = -(track.w_right - half_width - margin)
    hi = track.w_left - half_width - margin
    if np.any(lo > hi):
        raise ValueError("the road is narrower than the car and its margin somewhere")
    n = len(r)
    a = ca.MX.sym("a", n)
    px, py = r[:, 0] + a * nrm[:, 0], r[:, 1] + a * nrm[:, 1]
    idx = np.arange(n)
    im, ip = (idx - 1) % n, (idx + 1) % n
    mid = idx if closed else idx[1:-1]
    ax, ay = px[mid] - px[im[mid]], py[mid] - py[im[mid]]
    bx, by = px[ip[mid]] - px[mid], py[ip[mid]] - py[mid]
    cx, cy = ax + bx, ay + by
    la2, lb2, lc2 = ax * ax + ay * ay, bx * bx + by * by, cx * cx + cy * cy
    cross = ax * by - ay * bx
    # kappa^2 ds with ds = (|a| + |b|) / 2: 4 cross^2 / (|a|^2 |b|^2 |c|^2) (|a| + |b|) / 2
    cost = ca.sum1(2.0 * cross * cross / (la2 * lb2 * lc2) * (ca.sqrt(la2) + ca.sqrt(lb2)))
    cost += TIE * ca.sumsqr(a)
    solver = ca.nlpsol("line", "ipopt", {"x": a, "f": cost},
                       {"ipopt.print_level": 0, "ipopt.sb": "yes", "print_time": False,
                        "ipopt.tol": 1e-10})
    sol = solver(x0=np.zeros(n), lbx=lo, ubx=hi)
    if not solver.stats()["success"]:
        raise RuntimeError(f"the line did not converge: {solver.stats()['return_status']}")
    a = np.asarray(sol["x"]).ravel()
    p = r + a[:, None] * nrm
    h = _spacing(p, closed)
    s = track.s[0] + np.concatenate([[0.0], np.cumsum(h[: n - 1])])
    t = (np.roll(p, -1, 0) - np.roll(p, 1, 0)) if closed else np.gradient(p, axis=0)
    psi = np.unwrap(np.arctan2(t[:, 1], t[:, 0]))
    line = replace(track, s=s, x=p[:, 0], y=p[:, 1], psi=psi, kappa=_curvature(p, closed),
                   w_left=track.w_left - a, w_right=track.w_right + a,
                   grade=np.gradient(track.z, s))
    return line, a
