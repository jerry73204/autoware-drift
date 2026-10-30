"""When is a drift the fastest way through a corner? (docs/drift-optimality.md)

A corner is a synthetic section: a straight entry, a constant-radius arc through an angle with
linear curvature ramps either side, a straight exit, the road 8 m wide (Haruna's). The car starts
at the section's start on its centre line, straight, at a speed it can just brake from to the
corner's steady speed over most of the entry (so braking into it is part of the problem). Each
case is solved twice by L1's window (window.py: the live planner's problem, the car's own tyres,
its ECU and the stable-branch rows): free, and with the body's sideslip held within BETA_HOLD
(`Window(beta_max=...)`). A drift is the optimal solution iff the free plan's sideslip reaches
BETA_DRIFT and the held plan is slower by more than the solver's noise (the spread over warm
starts).

Arms:
- `l1`: the live window, unchanged (locked rear axle only: L1 models the spool's one wheel speed);
- `l1_relaxed`: the same with the stable-branch rows off and the slip ratio bound at 1.0
  (window.SLOPE_MIN, KAPPA_MAX patched before the window is built): how much the rows that keep
  the plan off the tyre's falling branch cost;
- `l0`: ocp.py's two-track car (every wheel's speed an input, no stable-branch row, slips within
  [-1, 3]), locked and open: the differential's arm, which L1 cannot model.

Holds: `beta` (|beta| <= BETA_HOLD, the question's definition) and `rear` (the rear axle's slip
angle within its tyre's peak, rear_peak; the one the answer uses).

Run: env -u PYTHONPATH .venv/bin/python studies/drift_optimality.py <grid> [--jobs N]
(grids: coarse (beta hold), rear, relaxed, l0, regen, power, grade, deep; results under OUT as
csv and npz), then `report` (the figures beside docs/drift-optimality.md) and `deep-report`
(the 170 deg corners' speeds and energy, deep_speeds.csv).

Multistart (roadmap 061 M1, docs/mode-choice-m1.md): grids multistart (the study's settings;
first), multistart-live, multistart-live-warm (the live window, one budgeted solve a start)
and e0b (the live grid on MUMPS and on SPRAL), results under MS_OUT as jsonl and npz; then
`multistart-report`; `library` builds race_planner/corner_library.npz (the live loop's corner
archetypes, roadmap 061 M3) from the multistart grid.
"""
import argparse
import csv
import dataclasses
import itertools
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from race_planner.car import Car  # noqa: E402
from race_planner.track import Track  # noqa: E402
from race_planner.modes import (D_BRAKE, D_FROM, FLICK_PAST, SEED_PAST,  # noqa: E402,F401
                                _consistent_u, _zone, reshape_slip, seed_flick,
                                seed_rear_out)

SWB = os.environ.get("SWB", os.path.expanduser("~/repos/carla-scenario-bridge"))
SPEC = os.path.join(SWB, "src/sim_vehicle_spec/vehicles/drift_mustang.yaml")
OUT = os.path.join(SWB, "play_log/race_planner/drift_study")

DS = 2.0             # m, the window's grid
LENGTH = 360.0       # m, every section (one compiled window per arm)
L_IN = 60.0          # m of entry straight
RAMP = 8.0           # m, each curvature ramp
EXIT_TIMED = 50.0    # m past the corner the section's time is read at
HALF_WIDTH = 4.0     # m, Haruna's
BETA_DRIFT = math.radians(5.0)
BETA_HOLD = math.radians(2.0)
REAR_PAST = 3.0      # deg: a free plan's rear axle this far past its tyre's peak slides


def section(R, theta, mu, sliding, grade=0.0, ds=DS):
    """The track: L_IN straight, a ramp up to 1 / R, the arc, a ramp down, the rest straight
    (left turn), on a grid of ds. The arc's length is what makes the whole turn theta. Returns
    (track, s_in, s_out): where the corner's ramps start and end."""
    s = np.arange(0.0, LENGTH + ds / 2, ds)
    arc = theta * R - RAMP
    assert arc > 0, (R, theta)
    s1, s2, s3, s4 = L_IN, L_IN + RAMP, L_IN + RAMP + arc, L_IN + 2 * RAMP + arc
    assert s4 + EXIT_TIMED + 20 < LENGTH, (R, theta)
    k = np.interp(s, [0, s1, s2, s3, s4, LENGTH], [0, 0, 1 / R, 1 / R, 0, 0])
    psi = np.concatenate([[0.0], np.cumsum(0.5 * (k[1:] + k[:-1]) * ds)])
    x = np.concatenate([[0.0], np.cumsum(0.5 * (np.cos(psi[1:]) + np.cos(psi[:-1])) * ds)])
    y = np.concatenate([[0.0], np.cumsum(0.5 * (np.sin(psi[1:]) + np.sin(psi[:-1])) * ds)])
    one = np.ones_like(s)
    t = Track(s=s, x=x, y=y, z=-grade * s, psi=psi, kappa=k, w_left=HALF_WIDTH * one,
              w_right=HALF_WIDTH * one, mu=mu * one, closed=False, sliding=sliding * one,
              grade=-grade * one)
    return t, s1, s4


def v_start(R, mu):
    """The entry speed: the corner's point-mass speed sqrt(mu g R) plus what 0.6 mu g brakes over
    the entry straight, at most 30 m/s."""
    g = 9.81
    return min(30.0, math.sqrt(mu * g * R + 2 * 0.6 * mu * g * (L_IN - 10.0)))


def make_car(diff="locked", power=1.0, regen=1.0):
    """The car, its motor's torque and power times `power`, its regeneration's times `regen`."""
    c = Car.from_spec(SPEC)
    return dataclasses.replace(c, diff=diff, axle_torque_max=c.axle_torque_max * power,
                               power_max=c.power_max * power,
                               regen_torque_max=c.regen_torque_max * regen,
                               regen_power_max=c.regen_power_max * regen)


# ---- L1 (window.py) ----

_WINDOWS = {}


def rear_peak(mu, sliding, car=None):
    """The rear tyre's peak slip angle (rad) in the planner's tyre (model.tyre, no longitudinal
    slip) at the outer rear wheel's load at the limit (1 + 0.6 mu rest loads: the lateral
    transfer at mu g): the smallest angle at which its lateral force is within 0.5 % of its
    largest. PhysX's brush saturates at tan = 3 mu Fz / C, so it scales with mu (2.9 deg at 0.3,
    15 deg at 1.12 with no fall-off); swb's fall-off starts at the rig's 8.6 deg."""
    from race_planner import modes
    return modes.rear_peak(car or make_car(), mu, sliding)


def hold_kwargs(hold, mu, sliding):
    """The held solve's bound: `beta`, the body's sideslip within BETA_HOLD (the question's own
    definition); `rear`, the rear axle's slip angle within its tyre's peak (rear_peak) plus
    REAR_MARGIN: the rear tyres never past their peak, which is what a drift is."""
    if hold == "beta":
        return {"beta_max": BETA_HOLD}
    return {"rear_alpha_max": round(rear_peak(mu, sliding) + REAR_MARGIN, 4)}


REAR_MARGIN = math.radians(1.0)   # PhysX's lateral slip is over |vx| + 1, the bound's over vx


def window(arm, held=None, power=1.0, regen=1.0):
    """The arm's window (built once per process), held by `held` (Window's kwargs) or free."""
    key = (arm, tuple(sorted((held or {}).items())), power, regen)
    if key not in _WINDOWS:
        from race_planner import window as wmod
        if arm == "l1_relaxed":
            wmod.SLOPE_MIN, wmod.KAPPA_MAX = -1e3, 1.0
        else:
            wmod.SLOPE_MIN, wmod.KAPPA_MAX = 0.01, 0.6
        _WINDOWS[key] = wmod.Window(make_car("locked", power, regen), ds=DS, length=LENGTH, grip=1.0,
                                    warm=False, tol=1e-6, max_iter=3000, **(held or {}))
    return _WINDOWS[key]


def _l1_solve(w, t, v0, X, U):
    x0 = np.array([v0, 0.0, 0.0, 0.0, 0.0, 0.0])
    X = np.array(X, float)
    X[:, 0] = x0
    w.lam = w.lam_x = None
    return w.solve(t, 0, x0, 1e3, X, U)


def _clip_slip(X, held, car):
    """X with its sideslip (or rear axle's slip) clipped into 0.9 of the hold: a warm start for
    the held solve from the free one's result."""
    X = X.copy()
    if "beta_max" in held:
        tb = 0.9 * math.tan(held["beta_max"])
        X[1] = np.clip(X[1], -tb * X[0], tb * X[0])
    else:
        tb = 0.9 * math.tan(held["rear_alpha_max"])
        X[1] = car.lr * X[2] + np.clip(X[1] - car.lr * X[2], -tb * X[0], tb * X[0])
    return X


def l1_case(arm, R, theta, mu, sliding, grade=0.0, power=1.0, hold="beta", regen=1.0):
    """Free and held solves of one case on L1, each from two warm starts: the cold one (the
    minimum-curvature line, window.initial) and the other solve's result; the best of each kept,
    the spread of each over its starts reported as its noise."""
    from race_planner.window import initial
    t, s_in, s_out = section(R, theta, mu, sliding, grade)
    v0 = v_start(R, mu)
    held = hold_kwargs(hold, mu, sliding)
    wf, wh = window(arm, None, power, regen), window(arm, held, power, regen)
    X0, U0 = initial(t, wf.car, wf.margin, v0)
    free = [_l1_solve(wf, t, v0, X0, U0)]
    hp = [_l1_solve(wh, t, v0, X0, U0)]
    if good(free[0]):
        hp.append(_l1_solve(wh, t, v0, _clip_slip(free[0]["X"], held, wf.car), free[0]["U"]))
    if good(hp[0]):
        free.append(_l1_solve(wf, t, v0, hp[0]["X"], hp[0]["U"]))
    bound = math.degrees(held.get("beta_max", held.get("rear_alpha_max")))
    return summarise(t, s_in, s_out, free, hp, arm=arm, R=R, theta=theta, mu=mu,
                     sliding=sliding, grade=grade, power=power, diff="locked", v0=v0,
                     hold=hold, bound=bound, regen=regen,
                     rear_peak=math.degrees(rear_peak(mu, sliding)))


def good(p):
    return (p["status"] in ("Solve_Succeeded", "Solved_To_Acceptable_Level")
            and p["defect"] < 1e-3 and p["violation"] < 1e-3 and p.get("edge", 0.0) < 0.05)


# ---- L0 (ocp.py) ----

def l0_case(diff, R, theta, mu, sliding, grade=0.0, power=1.0, hold="rear"):
    from race_planner.ocp import solve_min_time
    t, s_in, s_out = section(R, theta, mu, sliding, grade)
    v0 = v_start(R, mu)
    c = make_car(diff, power)
    held = hold_kwargs(hold, mu, sliding)
    runs = {}
    for name, kw in (("free", {}), ("held", held)):
        r = solve_min_time(t, c, v_start=v0, grip=1.0, max_iter=3000, **kw)
        runs[name] = [_from_l0(r)]
    if good(runs["free"][0]):           # each from the other's result too
        X = _clip_slip(runs["free"][0]["X"], held, c)
        r = solve_min_time(t, c, v_start=v0, grip=1.0, warm=(X, runs["free"][0]["U"]),
                           max_iter=3000, **held)
        runs["held"].append(_from_l0(r))
    if good(runs["held"][0]):
        r = solve_min_time(t, c, v_start=v0, grip=1.0, warm=(runs["held"][0]["X"],
                                                             runs["held"][0]["U"]), max_iter=3000)
        runs["free"].append(_from_l0(r))
    bound = math.degrees(held.get("beta_max", held.get("rear_alpha_max")))
    return summarise(t, s_in, s_out, runs["free"], runs["held"], arm="l0", R=R, theta=theta,
                     mu=mu, sliding=sliding, grade=grade, power=power, diff=diff, v0=v0,
                     hold=hold, bound=bound, rear_peak=math.degrees(rear_peak(mu, sliding)))


def _from_l0(r):
    worst = max(v[0] for k, v in r["violation"].items() if k != "defect")
    return {"X": r["X"], "U": r["U"], "s": r["s"], "t": r["t"], "time": r["time"],
            "status": r["status"] if r["ok"] else "failed:" + r["status"],
            "defect": r["violation"]["defect"][0], "violation": worst,
            "alpha": r["alpha"], "kappa": r["kappa"]}


# ---- what a plan does in the corner ----

def corner_metrics(p, t, s_in, s_out, held_car=None):
    """In the corner (its ramps' start to 20 m past their end): the largest |sideslip| (deg),
    the rear axle's largest |slip angle| (deg) and its sign against the turn's, whether the
    front wheels countersteer (steer against the turn's sense while turning), the slowest
    speed, the time at s_out + EXIT_TIMED, and how far past the road's edge the plan goes."""
    X, s = p["X"], p["s"]
    sel = (s >= s_in - 10) & (s <= s_out + 20)
    vx, vy, r, n, xi, delta = X
    beta = np.degrees(np.arctan2(vy, vx))
    from race_planner import model
    from race_planner.car import Car  # noqa: F401
    car = held_car
    a_r = np.degrees(np.arctan2(vy - r * car.lr, np.maximum(vx, 0.1)))   # rear axle's slip
    sgn = np.sign(t.kappa[sel].sum()) or 1.0
    counter = np.any((delta[sel] * sgn < -math.radians(0.5)) & (r[sel] * sgn > 0))
    i_b = int(np.argmax(np.abs(beta[sel])))
    edge = max(0.0, float(np.max(np.abs(n) - (HALF_WIDTH - 0.5 * car.width - 0.3))))
    return {"beta_max": float(np.abs(beta[sel]).max()), "beta_at": float(s[sel][i_b]),
            "beta_sign": float(np.sign(beta[sel][i_b]) * sgn),
            "rear_alpha_max": float(np.abs(a_r[sel]).max()),
            "delta_min": float(np.degrees((delta[sel] * sgn).min())),
            "countersteer": bool(counter), "v_min": float(np.hypot(vx, vy)[sel].min()),
            "t_timed": float(np.interp(s_out + EXIT_TIMED, s, p["t"])), "edge": edge}


def summarise(t, s_in, s_out, free, held, **case):
    car = make_car(case["diff"], case["power"], case.get("regen", 1.0))
    for p in free + held:
        p.update(corner_metrics(p, t, s_in, s_out, car))
    gf, gh = [p for p in free if good(p)], [p for p in held if good(p)]
    row = dict(case)
    row.update(free_ok=len(gf), free_n=len(free), held_ok=len(gh), held_n=len(held),
               free_status=free[0]["status"], held_status=held[0]["status"])
    if gf:
        bf = min(gf, key=lambda p: p["t_timed"])
        row.update(t_free=bf["t_timed"], beta_free=bf["beta_max"], beta_sign=bf["beta_sign"],
                   rear_alpha_free=bf["rear_alpha_max"], counter_free=bf["countersteer"],
                   delta_min_free=bf["delta_min"], vmin_free=bf["v_min"],
                   noise_free=max(p["t_timed"] for p in gf) - bf["t_timed"])
    if gh:
        bh = min(gh, key=lambda p: p["t_timed"])
        row.update(t_held=bh["t_timed"], beta_held=bh["beta_max"], vmin_held=bh["v_min"],
                   noise_held=max(p["t_timed"] for p in gh) - bh["t_timed"])
    if gf and gh:
        row["dt"] = row["t_held"] - row["t_free"]
        row["dt_pct"] = 100 * row["dt"] / row["t_free"]
        noise = max(row["noise_free"], row["noise_held"], 2e-3)
        row["noise"] = noise
        row["drift_optimal"] = bool(row["beta_free"] >= math.degrees(BETA_DRIFT)
                                    and row["dt"] > noise)
        if "rear_peak" in case:          # the rear past its tyre's peak by REAR_PAST, and faster
            row["rear_past"] = row["rear_alpha_free"] - case["rear_peak"]
            row["drift_rear"] = bool(row["rear_past"] >= REAR_PAST and row["dt"] > noise)
    keep = {f"free{i}_{k}": p[k] for i, p in enumerate(free) for k in ("X", "U", "t")}
    keep.update({f"held{i}_{k}": p[k] for i, p in enumerate(held) for k in ("X", "U", "t")})
    keep["s"], keep["kappa"] = t.s, t.kappa
    return row, keep


# ---- grids ----

MUS = (1.12, 0.8, 0.6, 0.45, 0.3)
SLIDINGS = (1.0, 0.8, 0.6)
RADII = (9.0, 15.0, 25.0, 40.0, 60.0)
THETAS = (math.pi / 2, math.pi)


def grid(name):
    if name == "coarse":
        for arm, mu, sl, R, th in itertools.product(("l1",), MUS, SLIDINGS, RADII, THETAS):
            yield ("l1", (arm, R, th, mu, sl))
    elif name == "rear":
        for mu, sl, R, th in itertools.product(MUS, SLIDINGS, RADII, THETAS):
            yield ("l1", ("l1", R, th, mu, sl, 0.0, 1.0, "rear"))
    elif name == "relaxed":
        for mu, sl, R, th in itertools.product((1.12, 0.6, 0.3), (1.0, 0.6), (9.0, 15.0, 40.0),
                                               THETAS):
            yield ("l1", ("l1_relaxed", R, th, mu, sl, 0.0, 1.0, "rear"))
    elif name == "l0":
        for diff, mu in itertools.product(("locked", "open"), (1.12, 0.6, 0.3)):
            yield ("l0", (diff, 15.0, math.pi, mu, 1.0, 0.0, 1.0, "rear"))
    elif name == "power":
        for p, mu, sl, R in itertools.product((0.5, 2.0), (1.12, 0.6, 0.3), (1.0, 0.6),
                                              (9.0, 15.0)):
            yield ("l1", ("l1", R, math.pi, mu, sl, 0.0, p, "rear"))
    elif name == "deep":            # a 170 deg turn: does a slide keep the speed up?
        for (mu, sl), R in itertools.product(((1.12, 0.8), (1.12, 1.0), (0.3, 1.0)),
                                             (9.0, 15.0, 25.0)):
            yield ("l1", ("l1", R, math.radians(170.0), mu, sl, 0.0, 1.0, "rear"))
    elif name == "regen":           # the ECU's regeneration off: the rear no longer brakes first
        for mu, sl, R, th in itertools.product((0.3, 0.45), (1.0, 0.6), (9.0, 15.0), THETAS):
            yield ("l1", ("l1", R, th, mu, sl, 0.0, 1.0, "rear", 0.0))
    elif name == "grade":
        for mu, sl, R in itertools.product((1.12, 0.6, 0.3), (1.0, 0.6), (9.0, 15.0)):
            yield ("l1", ("l1", R, math.pi, mu, sl, 0.08, 1.0, "rear"))
    else:
        raise SystemExit(f"no grid {name}")


# ---- multistart: a start in each valley (roadmap 061 M1, docs/mode-choice-m1.md) ----
#
# Each case is solved free from about eight starts, at the study's settings and at the live
# planner's, and the best feasible time over all of them is the case's oracle. The starts:
#
# - C, cold: window.initial (the minimum-curvature line at the quasi-steady speed), the live
#   loop's own first plan and the study's free0;
# - G, grip: the plan held to the rear tyres' peak (the held window, cold), the bound then
#   removed: what a live planner that defaults to grip warm-starts from, its last plan shifted
#   (offline there is no last plan; the held plan is the grip valley's own). This is also the
#   brief's held->free start: the two are one here;
# - Ds, rear out, synthetic: G with the body turned so that the rear axle runs SEED_PAST past
#   its tyre's peak from D_FROM before turn-in to the apex (the path and speed kept, the yaw rate
#   and the steering made to match, the front axle's slip kept), the request braking on low
#   friction and coasting on asphalt there;
# - Dc, rear out by continuation: the free plan with no fall-off (sliding 1.0, where a drift
#   wins easily), then the fall-off stepped to the surface's (CONT_STEPS), each step from the
#   last; on a surface with no fall-off it is C itself;
# - F, flick: as Ds, the rear first out the other way (FLICK_PAST, ahead of the turn-in);
# - P1-P3: G with smooth noise on every state.
#
# A held plan is solved twice (cold, and from the oracle's plan clipped into the bound), the
# better kept: the grip time a drift has to beat.

MS_OUT = os.path.join(SWB, "play_log/race_planner/mode_choice/m1")
SETTINGS = {
    # the study's: one window over the whole section, tight tolerance, no budget
    "study": {"ds": 2.0, "length": 360.0, "tol": 1e-6, "max_iter": 3000, "budget": None,
              "warm": False},
    # the live planner's (live.Driver's warm window): ds 4, its tolerance, iterations and
    # budget; 260 m, the longest window the Driver builds for these speeds. IPOPT's own barrier
    # start (warm False): a seed has no multipliers of its own (window.solve: rows' without the
    # bounds' cost more than none), and mu_init 1e-6 from a point that is not near optimal
    # stalls (measured: docs/mode-choice-m1.md)
    "live": {"ds": 4.0, "length": 260.0, "tol": 1e-3, "max_iter": 400, "budget": 1.0,
             "warm": False},
    "live_warm": {"ds": 4.0, "length": 260.0, "tol": 1e-3, "max_iter": 400, "budget": 1.0,
                  "warm": True},
}
LIVE_TIMED = 220.0                 # m: a live window's time read at most here, 40 m short of its end
CONT_STEPS = (1.0, 0.9, 0.8, 0.7, 0.6)
SEEDS = ("C", "G", "Ds", "Dc", "F", "P1", "P2", "P3")
_MS_WINDOWS = {}


def ms_window(setting, held=None, linsol="mumps", budget=True):
    """The window for a setting (SETTINGS), held by `held` or free, on IPOPT's linear solver
    `linsol`, under the setting's budget or (budget False) none (built once per process)."""
    key = (setting, tuple(sorted((held or {}).items())), linsol, budget)
    if key not in _MS_WINDOWS:
        from race_planner import window as wmod
        wmod.SLOPE_MIN, wmod.KAPPA_MAX = 0.01, 0.6
        o = dict(SETTINGS[setting])
        ipopt = dict(o.get("ipopt", {}))
        if linsol != "mumps":
            ipopt["linear_solver"] = linsol
        # without the budget (the first plan, the held ones): live.Driver's cold window
        _MS_WINDOWS[key] = wmod.Window(make_car(), ds=o["ds"], length=o["length"], grip=1.0,
                                       warm=o["warm"] and budget, tol=o["tol"], max_iter=o["max_iter"],
                                       budget=o["budget"] if budget else None, ipopt=ipopt,
                                       **(held or {}))
    return _MS_WINDOWS[key]


def seed_perturbed(X, U, t, car, k):
    """P1-P3: smooth noise (a 12 m running mean of Gaussian noise, the window's start kept) on
    every state: 3 % of the speed, 0.3 m/s sideways, 0.03 rad/s of yaw, 0.3 m of offset, 0.02
    rad of heading and 0.01 of steer."""
    rng = np.random.default_rng(1000 + k)
    N = X.shape[1]
    ds = float(t.s[1] - t.s[0])
    m = max(1, int(round(12.0 / ds)))
    sm = lambda: np.convolve(rng.standard_normal(N + m), np.ones(m) / m * math.sqrt(m),
                             "same")[:N]
    Xp = np.array(X, float).copy()
    Xp[0] *= 1 + 0.03 * sm()
    for i, a in ((1, 0.3), (2, 0.03), (3, 0.3), (4, 0.02), (5, 0.01)):
        Xp[i] += a * sm()
    Xp[5] = np.clip(Xp[5], -car.steer_max, car.steer_max)
    Xp[:, 0] = X[:, 0]
    w = np.ones(N)
    return Xp, _consistent_u(Xp, U, t, car, w, None)


def _ms_metrics(p, t, s_in, s_out, s_time, car, peak):
    """What a solve did: its time at s_time, the rear axle's largest slip in the corner past the
    tyre's peak (deg), the largest sideslip (deg), how far past the road's edge it went."""
    X, s = p["X"], p["s"]
    sel = (s >= s_in - D_FROM - 10) & (s <= s_out + 20)
    a_r = np.degrees(np.arctan2(X[1] - car.lr * X[2], np.maximum(X[0], 0.1)))
    edge = max(0.0, float(np.max(np.abs(X[3]) - (HALF_WIDTH - 0.5 * car.width - 0.3))))
    return {"t": float(np.interp(s_time, s, p["t"])), "status": p["status"],
            "good": bool(good(dict(p, edge=edge))), "ok": bool(p["ok"]) and edge < 0.05,
            "iters": int(p["iters"]), "solve_s": float(p["solve_s"]),
            "past": float(np.abs(a_r[sel]).max()) - math.degrees(peak),
            "beta": float(np.degrees(np.abs(np.arctan2(X[1], X[0])[sel]).max())),
            "edge": edge, "defect": float(p["defect"]), "violation": float(p["violation"])}


def _ms_solve(w, t, x0, v_end, X, U, keep_lam=False):
    X = np.array(X, float)
    X[:, 0] = x0
    if not keep_lam:
        w.lam = w.lam_x = None
    return w.solve(t, 0, x0, v_end, X, U)


def _stored(setting_from, mu, sliding, R, theta, name):
    """A plan of the study-setting grid (`multistart`): the start `name`'s for this case, as
    (s, X, U); None if that grid has not been run or has no such plan."""
    import json
    key = ("_stored", setting_from)
    if key not in _MS_WINDOWS:
        idx, data = {}, None
        try:
            with open(os.path.join(MS_OUT, f"{setting_from}.jsonl")) as f:
                for i, line in enumerate(f):
                    r = json.loads(line)
                    if "error" not in r:
                        idx[(r["mu"], r["sliding"], r["R"], round(r["theta"], 4))] = i
            data = np.load(os.path.join(MS_OUT, f"{setting_from}.npz"))
        except OSError:
            pass
        _MS_WINDOWS[key] = (idx, data)
    idx, data = _MS_WINDOWS[key]
    i = idx.get((mu, sliding, R, round(theta, 4)))
    if i is None or f"{i}/{name}_X" not in data:
        return None
    return data[f"{i}/s"], data[f"{i}/{name}_X"], data[f"{i}/{name}_U"]


def _resample(s_from, X, U, s_to):
    """Plan (X over s_from, U over its intervals) on the grid s_to."""
    Xn = np.array([np.interp(s_to, s_from, x) for x in X])
    Un = np.array([np.interp(s_to[:-1], s_from[:-1], u) for u in U])
    return Xn, Un


def ms_case(setting, R, theta, mu, sliding, linsol="mumps"):
    """One case from every start (SEEDS) at a setting; returns (record, arrays).

    At the live setting every start is one solve under the budget, as a re-plan would be; the
    grip plan G starts from (the held plan) is solved without one, as live.Driver's first plan
    is (its re-plans then keep it shifted), and so are the held plans and Cnb (C without a
    budget, for the oracle only). Dc is the study setting's continuation plan resampled onto the
    live grid: a continuation chain is offline work (each step a solve), so live it can only be
    a stored plan, here the case's own (an archetype that matches exactly: the best a library of
    corners could do)."""
    from race_planner.window import initial
    from race_planner.line import min_curvature_line
    from race_planner.qss import speed_profile
    o = SETTINGS[setting]
    live = setting != "study"
    t, s_in, s_out = section(R, theta, mu, sliding, ds=o["ds"])
    car = make_car()
    v0 = v_start(R, mu)
    x0 = np.array([v0, 0.0, 0.0, 0.0, 0.0, 0.0])
    peak = rear_peak(mu, sliding)
    held = {"rear_alpha_max": round(peak + REAR_MARGIN, 4)}
    wf = ms_window(setting, None, linsol)
    wh = ms_window(setting, held, linsol, budget=False)
    if not live:
        v_end, s_time = 1e3, s_out + EXIT_TIMED
    else:                              # the live loop's: the quasi-steady ceiling at the end
        line, _ = min_curvature_line(t, 0.5 * car.width, 0.3)
        v_end = float(np.interp(o["length"], t.s, speed_profile(line, car)))
        s_time = min(s_out + EXIT_TIMED, LIVE_TIMED)
    sec = wf.section(t, 0)
    X0, U0 = initial(sec, car, wf.margin, v0)
    rec = {"setting": setting, "linsol": linsol, "R": R, "theta": theta, "mu": mu,
           "sliding": sliding, "v0": v0, "peak": math.degrees(peak), "s_time": s_time,
           "seeds": {}, "held": {}}
    arrays = {"s": sec.s}
    m = lambda p: _ms_metrics(p, t, s_in, s_out, s_time, car, peak)

    def keep(name, p):
        arrays[f"{name}_X"], arrays[f"{name}_U"], arrays[f"{name}_t"] = p["X"], p["U"], p["t"]

    h0 = _ms_solve(wh, t, x0, v_end, X0, U0)
    rec["held"]["H0"] = m(h0)
    keep("H0", h0)
    starts = {"C": (X0, U0)}
    if h0["ok"]:
        Xb, Ub = h0["X"], h0["U"]
        starts["G"] = (Xb, Ub)
        starts["Ds"] = seed_rear_out(Xb, Ub, sec, s_in, s_out, car, peak, mu)
        starts["F"] = seed_flick(Xb, Ub, sec, s_in, s_out, car, peak, mu)
        for k in (1, 2, 3):
            starts[f"P{k}"] = seed_perturbed(Xb, Ub, sec, car, k)
    if live:
        st = _stored("multistart", mu, sliding, R, theta, "Dc")
        if st is None and sliding >= 1.0:
            st = _stored("multistart", mu, sliding, R, theta, "C")
        if st is not None:
            starts["Dc"] = _resample(st[0], st[1], st[2], sec.s)
    for name in ("C", "G", "Ds", "Dc", "F", "P1", "P2", "P3"):
        if name not in starts or (name == "Dc" and not live):
            continue
        p = _ms_solve(wf, t, x0, v_end, *starts[name])
        rec["seeds"][name] = m(p)
        keep(name, p)
        if name in ("Ds", "F"):
            arrays[f"{name}_seed_X"] = starts[name][0]
    if live:                           # the oracle's own: C without the budget
        wn = ms_window(setting, None, linsol, budget=False)
        p = _ms_solve(wn, t, x0, v_end, X0, U0)
        rec["seeds"]["Cnb"] = m(p)
        keep("Cnb", p)
        # and what the mode seeds need without one (their time is the budget's question)
        rec["nobudget"] = {}
        for name in ("G", "Ds", "Dc", "F"):
            if name in starts:
                rec["nobudget"][name] = m(_ms_solve(wn, t, x0, v_end, *starts[name]))
    elif sliding >= 1.0:               # Dc: on no fall-off it is C
        rec["seeds"]["Dc"] = dict(rec["seeds"]["C"], steps=1, degenerate=True)
        keep("Dc", {"X": arrays["C_X"], "U": arrays["C_U"], "t": arrays["C_t"]})
    else:                              # no fall-off first, then the surface's, each from the last
        X, U, its, wall = X0, U0, 0, 0.0
        steps = [x for x in CONT_STEPS if x > sliding + 1e-9] + [sliding]
        for j, sl in enumerate(steps):
            tk = dataclasses.replace(t, sliding=sl * np.ones_like(t.sliding))
            p = _ms_solve(wf, tk, x0, v_end, X, U, keep_lam=j > 0)
            its, wall = its + p["iters"], wall + p["solve_s"]
            if not p["ok"] and j < len(steps) - 1:
                break
            X, U = p["X"], p["U"]
        r = m(p)
        r.update(iters=its, solve_s=wall, steps=j + 1, degenerate=False,
                 reached=bool(abs(steps[j] - sliding) < 1e-9))
        if not r["reached"]:
            r.update(good=False, ok=False)
        rec["seeds"]["Dc"] = r
        keep("Dc", p)
    # the held plan again, from the best free plan clipped into the bound
    ok = {k: v for k, v in rec["seeds"].items() if v["good"]}
    if ok:
        best = min(ok, key=lambda k: ok[k]["t"])
        h1 = _ms_solve(wh, t, x0, v_end, _clip_slip(arrays[f"{best}_X"], held, car),
                       arrays[f"{best}_U"])
        rec["held"]["H1"] = m(h1)
        keep("H1", h1)
    return rec, arrays


def ms_grid(name):
    """The cases: every mu 0.45 and mu 0.3 corner of the study (3 slidings, 5 radii, 90 and 180
    deg), and every 180 deg asphalt hairpin (mu 1.12, sliding 0.8 and 1.0, 5 radii); grouped by
    surface so a worker's held windows are shared."""
    setting, linsol = {"multistart": ("study", "mumps"), "multistart-live": ("live", "mumps"),
                       "multistart-live-warm": ("live_warm", "mumps"),
                       "e0b": ("live", "pair")}[name]
    cases = [(mu, sl, R, th) for mu in (0.45, 0.3) for sl in SLIDINGS for R in RADII
             for th in THETAS]
    cases += [(1.12, sl, R, math.pi) for sl in (0.8, 1.0) for R in RADII]
    only = os.environ.get("MS_ONLY")              # e.g. "0.45" or "0.45,1.0": a subset
    if only:
        f = [float(x) for x in only.split(",")]
        cases = [c for c in cases if c[0] == f[0] and (len(f) < 2 or c[1] == f[1])]
    return [(setting, R, th, mu, sl, linsol) for mu, sl, R, th in cases]


def ms_run(args):
    """One case; with linsol "pair" (E0b) twice in the same worker, on MUMPS and on SPRAL, in
    an order that alternates by case (the host's load drifts): SPRAL's record, MUMPS's in it as
    "pair"."""
    c = time.time()
    try:
        if args[-1] == "pair":
            first = ("mumps", "spral") if hash(args[1:5]) % 2 else ("spral", "mumps")
            got = {ls: ms_case(*args[:-1], ls) for ls in first}
            rec, arrays = got["spral"]
            rec["pair"] = got["mumps"][0]
            rec["order"] = first
        else:
            rec, arrays = ms_case(*args)
    except Exception as e:
        import traceback
        return {"args": repr(args), "error": repr(e), "trace": traceback.format_exc()}, None
    rec["wall_s"] = time.time() - c
    return rec, arrays


def ms_main(name, jobs):
    """Run a multistart grid: <name>.jsonl (one record a case, every start's solve in it) and
    <name>.npz (every plan) under MS_OUT."""
    import json
    os.makedirs(MS_OUT, exist_ok=True)
    cases = ms_grid(name)
    arrays = {}
    with open(os.path.join(MS_OUT, f"{name}.jsonl"), "w") as f, ProcessPoolExecutor(jobs) as ex:
        for i, (rec, keep) in enumerate(ex.map(ms_run, cases, chunksize=5)):
            f.write(json.dumps(rec) + "\n")
            f.flush()
            if keep is None:
                print(i, rec, flush=True)
                continue
            sd = rec["seeds"]
            print(i, rec["mu"], rec["sliding"], rec["R"], round(math.degrees(rec["theta"])),
                  {k: (round(v["t"], 3), v["good"], v["iters"], round(v["solve_s"], 2),
                       round(v["past"], 1)) for k, v in sd.items()},
                  {k: (round(v["t"], 3), v["good"]) for k, v in rec["held"].items()},
                  round(rec["wall_s"]), flush=True)
            arrays.update({f"{i}/{k}": v for k, v in keep.items()})
    np.savez_compressed(os.path.join(MS_OUT, f"{name}.npz"), **arrays)


SEED_SETS = (("C",), ("G",), ("G", "Ds"), ("G", "Dc"), ("G", "Ds", "Dc"), ("G", "Ds", "F"),
             ("G", "Dc", "F"), ("G", "Ds", "Dc", "F"))


def _ms_load(name):
    import json
    path = os.path.join(MS_OUT, f"{name}.jsonl")
    if not os.path.exists(path):
        return []
    return [r for r in map(json.loads, open(path)) if "error" not in r]


def _usable(v, live):
    """A start's plan counts: converged and feasible (good) at the study's settings; what the
    live loop would adopt (feasible within the budget, on the road) at the live ones."""
    return v["ok"] if live else v["good"]


def ms_noise(recs, live):
    """Each case's spread over G and its perturbed starts that stay in G's valley (the rear
    within 2 deg of G's past its peak): the solver's own noise, without a change of valley."""
    out = []
    for r in recs:
        sd = r["seeds"]
        if "G" not in sd or not _usable(sd["G"], live):
            continue
        ts = [sd["G"]["t"]] + [sd[k]["t"] for k in ("P1", "P2", "P3")
                               if k in sd and _usable(sd[k], live)
                               and abs(sd[k]["past"] - sd["G"]["past"]) < 2.0]
        if len(ts) > 1:
            out.append((max(ts) - min(ts), 100 * (max(ts) - min(ts)) / min(ts)))
    return np.array(out)


def ms_classify(r, live, thr_pct, seeds=None):
    """The case against its oracle (the best usable plan of all starts, or of `seeds`): oracle
    time, its rear past its peak, the grip (held) time, the gain, and the label: drift (the best
    plan's rear >= REAR_PAST past its peak, and faster than grip by more than the noise), grip,
    or undecided (rear past its peak but the gain within the noise)."""
    sd = r["seeds"]
    names = [k for k in sd if (seeds is None or k in seeds) and _usable(sd[k], live)]
    held = [v["t"] for v in r["held"].values() if v["good"]]
    if not names:
        return None
    best = min(names, key=lambda k: sd[k]["t"])
    t_o, past = sd[best]["t"], sd[best]["past"]
    out = {"best": best, "t": t_o, "past": past}
    if held:
        th = min(held)
        thr = max(2e-3, thr_pct / 100 * t_o)
        gain = th - t_o
        out.update(t_held=th, gain=gain, gain_pct=100 * gain / t_o, thr=thr,
                   label=("drift" if past >= REAR_PAST and gain > thr else
                          "undecided" if past >= REAR_PAST and gain > -thr else "grip"))
    return out


def _pct(a, q):
    return float(np.percentile(a, q)) if len(a) else float("nan")


NOISE_Q = (50, 75, 90)              # the noise floor's quantiles reported; M3's: NOISE_Q_USE
NOISE_Q_USE = 75


def ms_floor(recs, live, q):
    """The noise floor per friction (mu -> % of the section's time): the q-th percentile of the
    perturbed spread (ms_noise) over that friction's cases."""
    out = {}
    for mu in sorted({r["mu"] for r in recs}):
        nz = ms_noise([r for r in recs if r["mu"] == mu], live)
        out[mu] = _pct(nz[:, 1], q) if len(nz) else 0.0
    return out


IT_BUDGETS = (150, 225, 300)       # iterations: 0.6, 0.9, 1.2 s at the quiet core's 4 ms each


def ms_iter_coverage(recs, floor, say):
    """Live coverage by iterations, not by this host's wall clock: each mode seed's solve
    without the budget (rec["nobudget"]) counts if it converged within an iteration budget
    (IT_BUDGETS; the wall time a quiet core takes for them is ms_report's calibration); the
    oracle is the best converged plan of every start, with and without the budget. `floor`:
    the noise floor per friction (the study setting's: the live perturbed solves are cut by the
    budget, their spread is the cut's, not the solver's)."""
    for nit in IT_BUDGETS:
        say(f"\n#### live coverage within {nit} iterations (study noise floor p{NOISE_Q_USE})\n")
        say("| seed set | reaches the oracle (drift cases) | share | mu 0.45 undecided |"
            "\n|---|---|---|---|")
        sets = (("G",), ("G", "Ds"), ("G", "Dc"), ("G", "F"), ("G", "Ds", "Dc"), ("G", "Ds", "F"),
                ("G", "Dc", "F"), ("G", "Ds", "Dc", "F"))
        pseudo = []
        for r in recs:
            sd = {}
            for k, v in r.get("nobudget", {}).items():
                sd[k] = dict(v, ok=v["good"] and v["iters"] <= nit)
            sd["all"] = dict(min((v for v in list(r.get("nobudget", {}).values())
                                  + list(r["seeds"].values()) if v["ok"]),
                                 key=lambda v: v["t"], default={"t": 1e9, "ok": False,
                                                                "past": 0.0}))
            pseudo.append(dict(r, seeds=sd))
        lab = {id(r): ms_classify(r, True, floor[r["mu"]], ("all",)) for r in pseudo}
        drift = [r for r in pseudo if (lab[id(r)] or {}).get("label") == "drift"]
        for ss in sets:
            def hit(r):
                c, o = ms_classify(r, True, floor[r["mu"]], ss), lab[id(r)]
                return bool(c and o and "thr" in o and c["t"] - o["t"] <= o["thr"])
            h = sum(hit(r) for r in drift)
            und = sum(1 for r in pseudo if r["mu"] == 0.45 and (ms_classify(
                r, True, floor[r["mu"]], ss) or {}).get("label") == "undecided")
            say(f"| {{{','.join(ss)}}} | {h}/{len(drift)} | {100 * h / max(1, len(drift)):.0f} % "
                f"| {und}/30 |")
        und_o = sum(1 for r in pseudo if r["mu"] == 0.45
                    and (lab[id(r)] or {}).get("label") == "undecided")
        say(f"\noracle's labels: drift {len(drift)}, mu 0.45 undecided {und_o}/30")


def ms_report():
    """The tables of docs/mode-choice-m1.md, from the multistart grids under MS_OUT: the noise
    floor, the oracle's labels, each seed set's coverage of the drift-optimal cases, the mu 0.45
    cases the study left undecided, every start's iterations and wall time at the live setting,
    and the linear solvers against each other (E0b). Printed, and written to MS_OUT/report.md.

    A case's threshold is its friction's noise floor (ms_floor) times its oracle time, at least
    2 ms (the study's floor)."""
    lines = []
    say = lambda *a: lines.append(" ".join(str(x) for x in a))
    study_rows = {}
    try:                               # the study's own labels (grid rear)
        for r in _rows("rear"):
            study_rows[(float(r["mu"]), float(r["sliding"]), float(r["R"]),
                        round(float(r["theta"]), 4))] = r
    except OSError:
        pass
    grids = [g for g in ("multistart", "multistart-live", "multistart-live-warm")
             if _ms_load(g)]
    groups = (("mu 0.45", 0.45), ("mu 0.3", 0.3), ("asphalt hairpins", 1.12))
    for g in grids:
        recs, live = _ms_load(g), g != "multistart"
        say(f"\n## {g}: {len(recs)} cases\n")
        say("noise floor: the spread of G and its perturbed starts that stay in G's valley, % of "
            "the section's time (ms)\n\n| friction | cases | p50 | p75 | p90 | max |\n"
            "|---|---|---|---|---|---|")
        for gname, mu in groups:
            nz = ms_noise([r for r in recs if r["mu"] == mu], live)
            if len(nz):
                say(f"| {gname} | {len(nz)} | " + " | ".join(
                    f"{_pct(nz[:, 1], q):.3f} ({_pct(nz[:, 0], q) * 1e3:.0f})"
                    for q in (50, 75, 90, 100)) + " |")
        for q in NOISE_Q:
            fl = ms_floor(recs, live, q)
            lab = {id(r): ms_classify(r, live, fl[r["mu"]]) for r in recs}
            drift = [r for r in recs if (lab[id(r)] or {}).get("label") == "drift"]
            say(f"\n### threshold: the noise floor's p{q} ("
                + ", ".join(f"mu {mu} {fl[mu]:.3f} %" for mu in fl) + ")\n")
            say("| friction | cases | drift | grip | undecided | no plan |\n|---|---|---|---|---|---|")
            for gname, mu in groups:
                ls = [lab[id(r)] for r in recs if r["mu"] == mu]
                cnt = lambda x: sum(1 for l in ls if l and l.get("label") == x)
                say(f"| {gname} | {len(ls)} | {cnt('drift')} | {cnt('grip')} | "
                    f"{cnt('undecided')} | {sum(1 for l in ls if not l or 'label' not in l)} |")
            say(f"\n| seed set | reaches the oracle, {len(drift)} drift cases | share | all cases "
                "| mu 0.45 undecided |\n|---|---|---|---|---|")
            for ss in SEED_SETS:
                def hit(r):
                    c, o = ms_classify(r, live, fl[r["mu"]], ss), lab[id(r)]
                    return bool(c and o and "thr" in o and c["t"] - o["t"] <= o["thr"])
                h = sum(hit(r) for r in drift)
                und = sum(1 for r in recs if r["mu"] == 0.45 and (ms_classify(
                    r, live, fl[r["mu"]], ss) or {}).get("label") == "undecided")
                say(f"| {{{','.join(ss)}}} | {h}/{len(drift)} | {100 * h / max(1, len(drift)):.0f}"
                    f" % | {sum(hit(r) for r in recs)}/{len(recs)} | {und}/30 |")
            if q != NOISE_Q_USE:
                continue
            say("\nwhich start is the oracle: " + ", ".join(
                f"{k} {sum(1 for r in recs if lab[id(r)] and lab[id(r)]['best'] == k)}"
                for k in SEEDS + ("Cnb",)))
            say("\n| mu 0.45 case | study | study gain/noise (s) | oracle start | past (deg) | "
                "gain (s) | thr (s) | label |\n|---|---|---|---|---|---|---|---|")
            for r in sorted((r for r in recs if r["mu"] == 0.45),
                            key=lambda r: (-r["sliding"], r["theta"], r["R"])):
                o = lab[id(r)] or {}
                sr = study_rows.get((r["mu"], r["sliding"], r["R"], round(r["theta"], 4)), {})
                st = ("drift" if sr.get("drift_rear") == "True" else
                      "undecided" if _num(sr, "rear_past") >= REAR_PAST and _num(sr, "dt") <=
                      _num(sr, "noise") else "grip") if sr else "-"
                say(f"| sl {r['sliding']} R {r['R']:.0f} {math.degrees(r['theta']):.0f} deg | "
                    f"{st} | {_num(sr, 'dt'):.3f}/{_num(sr, 'noise'):.3f} | {o.get('best')} | "
                    f"{o.get('past', float('nan')):.1f} | {o.get('gain', float('nan')):.3f} | "
                    f"{o.get('thr', float('nan')):.3f} | {o.get('label')} |")
        if live and any(r.get("nobudget") for r in recs):
            ms_iter_coverage(recs, ms_floor(_ms_load("multistart"), False, NOISE_Q_USE)
                             if _ms_load("multistart") else ms_floor(recs, live, NOISE_Q_USE), say)
        # per start: iterations and wall time
        say("\n| start | usable | iters p50/p95 | wall p50 | p95 | p99 | max | <= 0.9 s | ms/iter "
            "p50 |\n|---|---|---|---|---|---|---|---|---|")
        rowsets = [(k, [r["seeds"][k] for r in recs if k in r["seeds"]
                        and not r["seeds"][k].get("degenerate")], False) for k in SEEDS + ("Cnb",)]
        rowsets += [(k + ", no budget", [r["nobudget"][k] for r in recs
                                         if k in r.get("nobudget", {})], True)
                    for k in ("G", "Ds", "Dc", "F")]
        for k, v, nb in rowsets:
            if not v:
                continue
            w = np.array([x["solve_s"] for x in v])
            it = np.array([x["iters"] for x in v])
            use = sum((x["good"] if nb else _usable(x, live)) for x in v)
            say(f"| {k} | {use}/{len(v)} | {_pct(it, 50):.0f}/{_pct(it, 95):.0f} | "
                f"{_pct(w, 50):.2f} | {_pct(w, 95):.2f} | {_pct(w, 99):.2f} | {w.max():.2f} | "
                f"{100 * np.mean(w <= 0.9):.0f} % | {1e3 * _pct(w / np.maximum(it, 1), 50):.1f} |")
    # the linear solvers: the same live solves
    b = _ms_load("e0b")                 # the pairs: SPRAL's records, MUMPS's in them
    a = [r["pair"] for r in b]
    if a and b:
        bk = {(r["mu"], r["sliding"], r["R"], round(r["theta"], 4)): r for r in b}
        say("\n## linear solver: MUMPS against SPRAL, the same live solves\n")
        say("| start | MUMPS p50/p95/p99 (s) | SPRAL p50/p95/p99 (s) | iters MUMPS/SPRAL p50 | "
            "same plan (|dt| < 1 ms) |\n|---|---|---|---|---|")
        for k in SEEDS + ("Cnb",):
            pa, pb, same, n = [], [], 0, 0
            for r in a:
                q = bk.get((r["mu"], r["sliding"], r["R"], round(r["theta"], 4)))
                if not q or k not in r["seeds"] or k not in q["seeds"]:
                    continue
                pa.append((r["seeds"][k]["solve_s"], r["seeds"][k]["iters"]))
                pb.append((q["seeds"][k]["solve_s"], q["seeds"][k]["iters"]))
                if _usable(r["seeds"][k], True) and _usable(q["seeds"][k], True):
                    n += 1
                    same += abs(r["seeds"][k]["t"] - q["seeds"][k]["t"]) < 1e-3
            if not pa:
                continue
            pa, pb = np.array(pa), np.array(pb)
            say(f"| {k} | " + " | ".join("/".join(f"{_pct(x[:, 0], q):.2f}" for q in (50, 95, 99))
                                       for x in (pa, pb))
                + f" | {_pct(pa[:, 1], 50):.0f}/{_pct(pb[:, 1], 50):.0f} | {same}/{n} |")
        allw = lambda recs: np.array([v["solve_s"] for r in recs for k, v in r["seeds"].items()
                                      if k != "Cnb" and not v.get("degenerate")])
        wa, wb = allw(a), allw(b)
        say(f"\nall budgeted starts: MUMPS p50 {_pct(wa, 50):.3f} p95 {_pct(wa, 95):.3f} p99 "
            f"{_pct(wa, 99):.3f}; SPRAL p50 {_pct(wb, 50):.3f} p95 {_pct(wb, 95):.3f} p99 "
            f"{_pct(wb, 99):.3f}; p95 change {100 * (_pct(wb, 95) / _pct(wa, 95) - 1):+.0f} %")
    text = "\n".join(lines)
    print(text)
    with open(os.path.join(MS_OUT, "report.md"), "w") as f:
        f.write(text + "\n")


def run(job):
    kind, args = job
    c = time.time()
    try:
        row, keep = (l1_case if kind == "l1" else l0_case)(*args)
    except Exception as e:                       # a case that breaks is a row that says so
        return {"kind": kind, "args": repr(args), "error": repr(e)}, None
    row["wall_s"] = time.time() - c
    return row, keep


FIELDS = ("arm", "hold", "regen", "bound", "rear_peak", "rear_past", "drift_rear", "noise",
          "diff", "R", "theta", "mu", "sliding", "grade", "power", "v0", "free_ok",
          "free_n", "held_ok", "held_n", "free_status", "held_status", "t_free", "t_held", "dt",
          "dt_pct", "noise_free", "noise_held", "beta_free", "beta_sign", "rear_alpha_free",
          "counter_free", "delta_min_free", "vmin_free", "beta_held", "vmin_held",
          "drift_optimal", "wall_s", "error", "kind", "args")


DOCS = os.path.normpath(os.path.join(HERE, "../../../../docs"))


def _rows(name):
    with open(os.path.join(OUT, f"{name}.csv")) as f:
        return list(csv.DictReader(f))


def _num(r, k):
    try:
        return float(r[k])
    except (KeyError, ValueError):
        return float("nan")


def report():
    """The map's figures, beside docs/drift-optimality.md: per sliding share and turn angle, the
    time the rear-held plan loses (colour, %) over radius and friction, the cases where the free
    plan's rear axle is past its peak marked; and the hairpin's free and held plans on snow."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rows = _rows("rear")
    fig, axs = plt.subplots(2, 3, figsize=(13, 7.5), sharex=True, sharey=True,
                            constrained_layout=True)
    radii, mus = sorted({float(r["R"]) for r in rows}), sorted({float(r["mu"]) for r in rows})
    for i, th in enumerate(THETAS):
        for j, sl in enumerate(SLIDINGS):
            ax = axs[i, j]
            Z = np.full((len(mus), len(radii)), np.nan)
            for r in rows:
                if abs(float(r["theta"]) - th) < 1e-3 and float(r["sliding"]) == sl:
                    a, b = mus.index(float(r["mu"])), radii.index(float(r["R"]))
                    Z[a, b] = _num(r, "dt_pct")
                    past, drift = _num(r, "rear_past"), r.get("drift_rear") == "True"
                    ax.text(b, a, f"{_num(r, 'dt_pct'):.1f}%\n{past:+.0f}\u00b0",
                            ha="center", va="center", fontsize=8,
                            fontweight="bold" if drift else "normal",
                            color="black" if drift else "0.35")
            im = ax.imshow(Z, origin="lower", cmap="Oranges", vmin=0, vmax=5, aspect="auto")
            ax.set_title(f"{math.degrees(th):.0f}\u00b0 turn, sliding {sl}")
            ax.set_xticks(range(len(radii)), [f"{x:.0f}" for x in radii])
            ax.set_yticks(range(len(mus)), [f"{x:g}" for x in mus])
            if i == 1:
                ax.set_xlabel("radius R (m)")
            if j == 0:
                ax.set_ylabel("peak friction mu")
    fig.colorbar(im, ax=axs, shrink=0.8, label="time lost held to the rear tyres' peak (%)")
    fig.suptitle("L1 (live window), locked rear: what drifting buys. Bold: a drift is optimal "
                 "(free plan's rear axle >= 3\u00b0 past its peak, held plan slower than noise); "
                 "second line: rear axle past its peak, deg", fontsize=10)
    fig.savefig(os.path.join(DOCS, "drift-optimality-map.png"), dpi=110)
    # the hairpins on low friction: free against held, along s
    data = np.load(os.path.join(OUT, "rear.npz"))
    fig, axs = plt.subplots(4, 1, figsize=(9, 9), sharex=True, constrained_layout=True)
    car = make_car()
    for sl, ls in ((1.0, "-"), (0.6, "--")):
        idx = [k for k, r in enumerate(rows) if float(r["R"]) == 9.0
               and abs(float(r["theta"]) - math.pi) < 1e-3 and float(r["mu"]) == 0.3
               and float(r["sliding"]) == sl]
        if not idx:
            continue
        k = idx[0]
        s = data[f"{k}/s"]
        for tag, col in (("free0", "C3"), ("held0", "C0")):
            if f"{k}/{tag}_X" not in data:
                continue
            X = data[f"{k}/{tag}_X"]
            vx, vy, r, _, _, delta = X
            lab = f"{tag[:-1]}, sliding {sl}"
            axs[0].plot(s, np.hypot(vx, vy), ls, color=col, label=lab)
            axs[1].plot(s, np.degrees(np.arctan2(vy, vx)), ls, color=col)
            axs[2].plot(s, np.degrees(np.arctan2(vy - car.lr * r, vx)), ls, color=col)
            axs[3].plot(s, np.degrees(delta), ls, color=col)
    for ax, lab in zip(axs, ("speed (m/s)", "sideslip beta (deg)", "rear axle slip (deg)",
                             "steer delta (deg)")):
        ax.set_ylabel(lab)
        ax.grid(alpha=0.3)
        ax.axvspan(L_IN, L_IN + 2 * RAMP + math.pi * 9.0 - RAMP, color="0.9")
    axs[0].legend(fontsize=8)
    axs[-1].set_xlabel("s (m); grey: the hairpin, R 9 m, 180 deg, mu 0.3")
    axs[-1].set_xlim(20, 160)
    fig.savefig(os.path.join(DOCS, "drift-optimality-hairpin.png"), dpi=110)


def dissipation(car, X, U, P):
    """Power the four tyres turn into heat by sliding (W, per interval): each wheel's force times
    its slip velocity, Fx (omega R - lx) - Fy ly, in its own frame (window.node_functions' out
    for the forces, the same kinematics for the velocities)."""
    from race_planner import window as wmod
    _, _, _, _, out = wmod.node_functions(car, tyres=True)
    vx, vy, r, _, _, delta = X[:, :-1]
    tn, L, tr = np.tan(delta), car.wheel_base, car.track
    steer = [np.arctan2(L * tn, L - 0.5 * tr * tn), np.arctan2(L * tn, L + 0.5 * tr * tn),
             0 * tn, 0 * tn]
    w = [U[5], U[6], U[4], U[4]]
    from race_planner import model
    pw = np.zeros(X.shape[1] - 1)
    for k in range(len(pw)):
        fx, fy = (np.asarray(v).ravel() for v in out(X[:, k], U[:, k], P[:, k])[:2])
        for i, (px, py) in enumerate(model.wheels(car)):
            ux, uy = vx[k] - r[k] * py, vy[k] + r[k] * px
            c, sn = np.cos(steer[i][k]), np.sin(steer[i][k])
            lx, ly = c * ux + sn * uy, -sn * ux + c * uy
            pw[k] += fx[i] * (w[i][k] - lx) - fy[i] * ly
    return pw


def deep_report():
    """The 170 deg corners (grid `deep`): for the free and the rear-held plan, the speeds at
    turn-in, apex (the least), the corner's end and 50 m on, the kinetic energy the corner costs,
    where the rear is past its peak and what the request does there, when the car has turned
    half the corner, when it is back on the power, the tyres' sliding loss, and the time."""
    rows = _rows("deep")
    data = np.load(os.path.join(OUT, "deep.npz"))
    car = make_car()
    out = []
    for k, r in enumerate(rows):
        R, th, mu, sl = (float(r[x]) for x in ("R", "theta", "mu", "sliding"))
        t, s_in, s_out = section(R, th, mu, sl)
        P = np.vstack([t.kappa, mu * np.ones_like(t.s), t.grade, t.sliding])[:, :-1]
        pk = float(r["rear_peak"])
        best = {}
        for tag in ("free", "held"):        # the faster of the starts, as in the map
            cands = [j for j in range(2) if f"{k}/{tag}{j}_X" in data]
            tt = [float(np.interp(s_out + EXIT_TIMED, t.s, data[f"{k}/{tag}{j}_t"])) for j in cands]
            best[tag] = cands[int(np.argmin(tt))]
        for tag in ("free", "held"):
            j = best[tag]
            X, U, tm = data[f"{k}/{tag}{j}_X"], data[f"{k}/{tag}{j}_U"], data[f"{k}/{tag}{j}_t"]
            v = np.hypot(X[0], X[1])
            at = lambda q, x=None: float(np.interp(q, t.s, v if x is None else x))
            sel = (t.s >= s_in) & (t.s <= s_out)
            i_min = int(np.argmin(np.where(sel, v, np.inf)))
            ar = np.degrees(np.abs(np.arctan2(X[1] - car.lr * X[2], X[0])))
            past = np.where(ar[:-1] > pk + 1.0)[0]
            a = U[1]
            heading = t.psi + X[4]
            half = float(np.interp(0.5 * th, heading - heading[0], t.s))
            after = np.where((t.s[:-1] > t.s[i_min]) & (a > 0.5))[0]
            pw = dissipation(car, X, U, P)
            dt_seg = np.diff(tm)
            corner = (t.s[:-1] >= s_in - 30) & (t.s[:-1] <= s_out)
            out.append({
                "mu": mu, "sliding": sl, "R": R, "plan": tag,
                "v_turn_in": at(s_in), "v_min": float(v[i_min]), "s_min": float(t.s[i_min]),
                "v_exit": at(s_out), "v_exit50": at(s_out + EXIT_TIMED),
                "ke_lost_kJ": 0.5 * car.m * (at(s_in) ** 2 - at(s_out) ** 2) / 1e3,
                "slide_s": (f"{t.s[past[0]]:.0f}-{t.s[past[-1]]:.0f}" if len(past) else "-"),
                "slide_a": float(np.mean(a[past])) if len(past) else float("nan"),
                "rear_alpha_max": float(ar.max()),
                "s_half_turn": half, "t_half_turn": float(np.interp(half, t.s, tm)),
                "s_power_on": float(t.s[after[0]]) if len(after) else float("nan"),
                "slide_heat_kJ": float(np.sum((pw * dt_seg)[corner])) / 1e3,
                "t_corner": float(np.interp(s_out, t.s, tm)),
                "t_section": float(np.interp(s_out + EXIT_TIMED, t.s, tm)),
                "s_in": s_in, "s_out": s_out})
    path = os.path.join(OUT, "deep_speeds.csv")
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, list(out[0]))
        w.writeheader()
        for o in out:
            w.writerow({k: (round(v, 3) if isinstance(v, float) else v) for k, v in o.items()})
    for o in out:
        print({k: (round(v, 2) if isinstance(v, float) else v) for k, v in o.items()})


def main():
    if sys.argv[1:] == ["report"]:
        return report()
    if sys.argv[1:] == ["deep-report"]:
        return deep_report()
    if sys.argv[1:] == ["multistart-report"]:
        return ms_report()
    if sys.argv[1:] == ["library"]:                  # race_planner/corner_library.npz (M3)
        from race_planner import modes
        bounds = lambda R, th: (L_IN, L_IN + RAMP + th * R)
        return print(modes.build_library(MS_OUT, make_car(), bounds), "archetypes")
    ap = argparse.ArgumentParser()
    ap.add_argument("grid")
    ap.add_argument("--jobs", type=int, default=6)
    a = ap.parse_args()
    if a.grid.startswith("multistart") or a.grid == "e0b":
        # one thread a solve (MUMPS, SPRAL): the wall times are one core's, as live
        os.environ.setdefault("OMP_NUM_THREADS", "1")
        os.environ.setdefault("OMP_CANCELLATION", "TRUE")      # SPRAL asks for it
        return ms_main(a.grid, a.jobs)
    os.makedirs(OUT, exist_ok=True)
    jobs = list(grid(a.grid))
    path = os.path.join(OUT, f"{a.grid}.csv")
    arrays = {}
    with open(path, "w", newline="") as f, ProcessPoolExecutor(a.jobs) as ex:
        w = csv.DictWriter(f, FIELDS, extrasaction="ignore")
        w.writeheader()
        # chunks of ten share a held window (one surface's radii and angles: one compile)
        for i, (row, keep) in enumerate(ex.map(run, jobs, chunksize=10 if a.grid == "rear" else 1)):
            row = {k: (round(v, 5) if isinstance(v, float) else v) for k, v in row.items()}
            w.writerow(row)
            f.flush()
            print(i, {k: row.get(k) for k in ("arm", "hold", "diff", "R", "theta", "mu", "sliding",
                                               "beta_free", "rear_past", "dt", "dt_pct",
                                               "drift_optimal", "drift_rear",
                                               "free_status", "held_status", "error")},
                  flush=True)
            if keep:
                arrays.update({f"{i}/{k}": v for k, v in keep.items()})
    np.savez_compressed(os.path.join(OUT, f"{a.grid}.npz"), **arrays)


if __name__ == "__main__":
    main()
