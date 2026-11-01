"""Grip or drift chosen live, model in the loop (roadmap 061 M4, experiment E2;
docs/mode-choice-m4.md).

live.Driver re-plans (L1) and the NMPC tracks (L2, tracker.Tracker) while sim.Plant drives: the
two-track car with each wheel's spin behind sim_vehicle_ecu (the ECU Isaac's car runs), on the
world's surface (the map's true classes). So unlike M3/M3b's chains the plan is not the car:
what is measured is whether the car keeps going, how far it leaves its plans, and its time.

Arms (`--arm`):
- `single`: 060's one start;
- `always`: G, H (the fallback), D every re-plan, F into hairpins (schedule always=True);
- `sched`: M3b's scheduler (design 4.4a, the asphalt rule tightened in M4);
- each on a map `--err` % of whose zones are misdeclared (`--seed` picks which and as what),
  with `--perc` (the simulated camera: veto, lowers friction near the car) and `--est` (the
  car's estimate used, held low through a slide: surface.FrictionEstimate).

Courses:
- `c:<class>:<R>:<turn deg>:<width m>[:<entry m>]`: one corner of the drift study's geometry (60 m
  entry straight, or <entry>; 8 m ramps, the exit straight), the whole road one swb class;
- `zones`: M3b's class-zone chain (E1's ten corners, each in a zone of a class, one uncertain);
- `haruna`, `tsukuba`: swb's courses, dry, one lap;
- `<course>@<class>`: haruna, tsukuba or zones (E1's ten-corner chain) with the whole road one
  swb class, e.g. `haruna@snow_packed` (a class the table does not have is refused by name).

The oracle (`oracle <course>`): the corner's best-of-starts plan offline at the loop's settings
(grip 0.92, ds 4, from the loop's start state), tightly solved: free from cold, from H's result,
from D (library and synthetic) and F; held at H's bound from cold and from the free result.
Its drift gain (held - best) is the ceiling a loop can keep.

Run:  env -u PYTHONPATH .venv/bin/python studies/mode_choice_m4.py run <course> --arm sched
          [--err 10 --seed 1 --perc --est --rep 1 --cpus 8,9,10,11]
          [--style <style.yaml or JSON>] [--digest] [--jit 1e-9] [--traj] [--prices <json>]
      --traj: the run tick by tick as well, <key>.traj.json (the car, each wheel's slip against
      its peak, every adopted plan with its seed and mode, the road), for tools/arm_video.py;
      --style: the driving style record (roadmap 065; live.Driver's style), none by default;
      --digest: a sha256 of every plan the driver made (X, U, s, t) and of the plant's states,
      for bit-for-bit comparisons (roadmap 065 E1). With RP_DET=1 in the environment the solvers
      run without a clock (IPOPT uncut by wall time, every solve timed 0 s, so a plan is adopted
      at the fixed latency), which makes a run repeatable bit for bit on any load; with
      RP_ITERCAP=<n> too, a budgeted solve is cut at n iterations a second of its budget instead
      (the budget, load-free; 170 is about 1 s at 5.8 ms an iteration); RP_ROOT puts
      another race_planner tree first (another commit's, for the same comparison);
      --jit: the start speed times (1 + jit), a last-bit start: ice's corners are chaotic (an
      outcome flips with the last bit), so a repeatable run is one draw, and --jit 1e-9, 2e-9 ...
      draw others (every run's plant digest differs; name each with --tag; M4e).
      ... oracle <course>;  ... report;  ... pareto <course>[,<course>...] (065 M2, E2)
Results: swb's play_log/race_planner/mode_choice/m4/.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.environ.get("RP_ROOT") or os.path.dirname(HERE))
sys.path.insert(0, HERE)

from race_planner import axle, live, modes  # noqa: E402
from race_planner.car import Car  # noqa: E402
from race_planner.live import Driver  # noqa: E402
from race_planner.multistart import held_bound  # noqa: E402
from race_planner.sim import Plant  # noqa: E402
from race_planner.surface import FrictionEstimate, Perception, SurfaceMap, Zone  # noqa: E402
from race_planner.track import SurfaceTable, Track  # noqa: E402
from race_planner.tracker import Tracker  # noqa: E402
import drift_optimality as do  # noqa: E402
import mode_choice_e1 as e1  # noqa: E402
import mode_choice_m3b as m3b  # noqa: E402

SWB = e1.SWB


def _deterministic():
    """RP_DET=1: no clock in the planner (module docstring). Applied at import, so the seed
    processes (spawned: they import this script again) run the same way."""
    import types
    from race_planner import multistart, tracker, window
    clock = types.SimpleNamespace(time=lambda: 0.0, perf_counter=lambda: 0.0)
    window.time = tracker.time = multistart.time = clock
    init = window.Window.__init__

    cap = os.environ.get("RP_ITERCAP")

    def uncut(self, *a, **kw):
        # RP_ITERCAP: the budget as IPOPT iterations a second of it (a budgeted window's
        # max_iter its budget times this), so a run is repeatable and still budgeted (M4d)
        if cap and kw.get("budget") is not None:
            kw["max_iter"] = max(1, int(round(float(cap) * kw["budget"])))
        kw["budget"] = None
        init(self, *a, **kw)
    window.Window.__init__ = uncut


if os.environ.get("RP_DET") == "1":
    _deterministic()
OUT = os.environ.get("M4_OUT", os.path.join(SWB, "play_log/race_planner/mode_choice/m4"))
GRIP, DS = e1.GRIP, e1.DS
RATE = 50.0
CLASSES = ("asphalt", "mud_wet", "snow_loose", "gravel", "snow_packed", "ice")
ORACLE_LEN = 300.0


def table():
    return SurfaceTable.from_yaml(e1.CLASSES)


def misdeclare(zones, err, seed, classes=CLASSES):
    """`err` % of the zones (at least one if err > 0) declared as another class, picked and
    drawn with `seed`; the world (true) keeps its own."""
    if not err:
        return zones, []
    rng = np.random.default_rng(seed)
    k = max(1, int(round(err / 100.0 * len(zones))))
    pick = sorted(rng.choice(len(zones), k, replace=False).tolist())
    out = list(zones)
    for i in pick:
        z = zones[i]
        others = [c for c in classes if c != z.declared]
        out[i] = Zone(z.s0, z.s1, others[int(rng.integers(len(others)))], 1.0, z.declared)
    return out, pick


def course(name, car, err=0, seed=0):
    """(base track at 1 m, surface map or None, start s, start speed, timed span (s0, s1) or
    None, corner info). The planner's track is smap.planner_track(base), the plant's
    smap.true_track(base)."""
    tab = table()
    if "@" in name:                      # a course with its whole road one class
        where, cls = name.split("@", 1)
        if cls not in tab.mu_of:
            raise ValueError(f"course {name!r}: unknown surface class {cls!r} (the table's: "
                             f"{', '.join(tab.mu_of)})")
        if where in ("haruna", "tsukuba"):
            t = Track.from_swb_course(os.path.join(SWB, "data/courses", where), tab, car)
            if t.closed:
                t = t.unrolled(1)
            base = t.resampled(1.0)
        elif where == "zones":
            base = e1.chain(1.0)
        else:
            raise ValueError(f"course {name!r}: no course {where!r} to put {cls!r} on "
                             "(haruna, tsukuba, zones)")
        zones, pick = misdeclare([Zone(0.0, 1e5, cls)], err, seed)
        return base, SurfaceMap(tab, car, zones), 10.0, 12.0, None, {"class": cls,
                                                                     "wrong": pick}
    if name in ("haruna", "tsukuba"):
        t = Track.from_swb_course(os.path.join(SWB, "data/courses", name), tab, car)
        if t.closed:
            t = t.unrolled(1)
        return t.resampled(1.0), None, 10.0, 12.0, None, {}
    if name == "zones":
        base = e1.chain(1.0)
        b4 = base.resampled(DS)
        zones = []
        for c, cls in zip(modes.corners(b4, GRIP), m3b.ZONES):
            if cls.endswith("?"):
                zones.append(Zone(c.s_in - m3b.APPROACH, c.s_out + 20.0, cls[:-1], 0.85, None,
                                  ("mud_wet",)))
            else:
                zones.append(Zone(c.s_in - m3b.APPROACH, c.s_out + 20.0, cls))
        zones, pick = misdeclare(zones, err, seed)
        return base, SurfaceMap(tab, car, zones), 10.0, 12.0, None, {"wrong": pick}
    _, cls, R, turn, width, *entry = name.split(":")
    R, turn, width = float(R), math.radians(float(turn)), float(width)
    # an entry straight longer than the study's 60 m (roadmap 065 M3): the gate's re-plans need
    # room before the turn-in (two in a row with the drift's onset AHEAD into the window)
    l_in = float(entry[0]) if entry else do.L_IN
    length = max(do.LENGTH, l_in + turn * R + 2 * do.RAMP + do.EXIT_TIMED + 60.0)
    base, s_in, s_out = do.section(R, turn, 1.12, 0.8, ds=1.0, half_width=width / 2,
                                   l_in=l_in, length=length)
    zones, pick = misdeclare([Zone(0.0, 1e5, cls)], err, seed)
    smap = SurfaceMap(tab, car, zones)
    v0 = do.v_start(R, tab.mu(cls, car) * GRIP)
    return (base, smap, 8.0, v0, (8.0, s_out + do.EXIT_TIMED),
            {"class": cls, "R": R, "turn": round(math.degrees(turn)), "width": width,
             "s_in": s_in, "s_out": s_out, "wrong": pick})


def tracks(name, car, err=0, seed=0):
    base, smap, s0, v0, span, info = course(name, car, err, seed)
    if smap is None:
        return base.resampled(DS), base, None, s0, v0, span, info
    return (smap.planner_track(base).resampled(DS), smap.true_track(base), smap, s0, v0, span,
            info)


def _load():
    return [round(x, 2) for x in os.getloadavg()]


def run(name, arm="single", err=0, seed=0, perc=False, est=False, rep=1, cpus=None, rules=None,
        sd=None, tag="", beta_max=None, front=None, rear=None, rear_low=None, style=None,
        digest=False, jit=0.0, traj=False, prices=None, ecu=None, ecu_obs=False,
        force_drift=False, b_iter=None, b_budget=None):
    car = Car.from_spec(e1.SPEC)
    if sd is not None:
        import race_planner.surface as sf
        sf.CLASS_SD = sd
    planner, world, smap, s0, v0, span, info = tracks(name, car, err, seed)
    v0 *= 1.0 + jit                     # a last-bit start (--jit)
    if sd is not None and smap is not None:
        smap.sd = sd
        base, *_ = course(name, car, err, seed)
        planner = smap.planner_track(base).resampled(DS)
    ms = None
    if arm != "single":
        ms = {"threshold": 0.003, "seed_budget": e1.SEED_BUDGET, "cpus": cpus,
              "schedule": {"table": table(), "smap": smap, "always": arm == "always",
                           **({"rules": rules} if rules else {})},
              "perception": Perception(smap, seed=1000 * rep + seed) if perc and smap else None,
              **({"course_prices": prices} if prices else {}),
              **({"b_max_iter": b_iter} if b_iter else {}),
              **({"b_budget": b_budget} if b_budget else {})}
    plant = Plant(car, e1.SPEC, world, ecu_overrides=ecu)
    # a car on ice has braked on it before the corner: its ECU has found the rear's grip and
    # holds regen within its share of it (issue 009), as the planner's model assumes there
    # (axle.regen_hold_car, axle.held); found here by braking on the course's start, then the
    # run starts afresh with only that kept
    primed = None
    if axle.regen_hold_car(car, world.mu, world.sliding).regen_grip_share:
        primed = plant.regen_grip_found(s=s0, speed=v0)
    plant.reset(s=s0, speed=v0)
    if primed is not None:
        plant.ecu.powertrain.axle_mu = primed
    d = Driver(car, planner, Tracker(car, planner, grip=GRIP), ds=DS, grip=GRIP, multistart=ms,
               use_estimate=est, estimator=FrictionEstimate() if est else None,
               beta_max=None if beta_max is None else math.radians(beta_max),
               **({} if front is None else {"front_slip_max": None if front <= 0 else front}),
               **({} if rear is None else {"rear_slip_max": None if rear <= 0 else rear}),
               **({} if rear_low is None else {"rear_slip_low": None if rear_low <= 0 else rear_low}),
               **({} if style is None else {"style": style}))
    if force_drift:
        _force_drift(d.multi)
    dig = _digest(d, plant) if digest else None
    rec = _Traj(d, car, world) if traj else None
    obs = _DriftObs(car, plant, world, (style.drift_past_deg if style is not None
                                        and style.name != "fastest" else 3.0))
    eobs = _EcuObs(d, car, plant, obs, 3.0) if ecu is not None or ecu_obs else None
    s_end = (span[1] + 10.0) if span else float(planner.s[-1]) - 10.0
    load0, c0 = _load(), time.time()
    v_used = v0
    for k in range(4):          # a cold first plan that fails: the car starts 10 % slower
        try:
            out = live.run(d, plant, s_end, rate=RATE, t_max=400.0 if span is None else 120.0)
            break
        except RuntimeError as ex:
            out = {"result": f"error: {ex}"[:200]}
            if "no first plan" not in str(ex):
                break
            v_used = v0 * 0.9 ** (k + 1)
            plant.reset(s=s0, speed=v_used)
            d.log.clear()
            d.solves.clear()
        except Exception as ex:              # a solver crash
            out = {"result": f"error: {type(ex).__name__}: {ex}"[:200]}
            break
    wall = time.time() - c0
    d.close()
    log = d.log
    s = np.array([r["s"] for r in log]) if log else np.zeros(1)
    t = np.array([r["t"] for r in log]) if log else np.zeros(1)
    lat = np.array([abs(r["n"] - r["n_plan"]) for r in log]) if log else np.zeros(1)
    beta = np.array([abs(r["beta_deg"]) for r in log]) if log else np.zeros(1)
    bplan = np.array([abs(r["beta_plan_deg"]) for r in log]) if log else np.zeros(1)
    berr = np.array([abs(r["beta_deg"] - r["beta_plan_deg"]) for r in log]) if log else np.zeros(1)
    fb = sum(1 for r in log if r["fallback"] and not r["stopping"])
    solves = d.solves[1:]
    recs = d.multi.records if d.multi is not None else []
    t_timed = None
    if span and len(s) > 1 and s.max() >= span[1]:
        t_timed = float(np.interp(span[1], s, t) - np.interp(span[0], s, t))
    t_course = None
    if out.get("result") == "done":
        t_course = float(out["t"])
    res = {"course": name, "arm": arm, "err": err, "seed": seed, "perc": perc, "est": est,
           "rep": rep, "tag": tag, "rules": rules, "sd": sd, "beta_bound": beta_max,
           "front_slip_max": d.front_slip_max, "rear_slip_max": d.rear_slip_max,
           "rear_slip_low": d.rear_slip_low, "info": info,
           "v0": v0,
           "v_start_used": v_used,
           "result": out.get("result"), "t": out.get("t"), "s": out.get("s"),
           "t_timed": t_timed, "t_course": t_course, "wall": round(wall, 1), "load0": load0,
           "load1": _load(),
           "replans": len(solves), "failed": sum(1 for x in solves if not x["ok"]),
           "overruns": sum(1 for x in solves if x["solve_s"] > 1.0),
           "solve_p50": float(np.median([x["solve_s"] for x in solves])) if solves else None,
           "solve_p95": float(np.percentile([x["solve_s"] for x in solves], 95)) if solves else None,
           "solve_max": float(max([x["solve_s"] for x in solves])) if solves else None,
           # timed runs (065 M3 left, E3): L1's wall time a re-plan, and its adoption (at
           # max(latency, solve): live.Driver), tails and every value, for R6
           "solve_p99": float(np.percentile([x["solve_s"] for x in solves], 99)) if solves else None,
           "solve_all": [round(float(x["solve_s"]), 4) for x in solves],
           "adopt_p99": (float(np.percentile([max(d.budget, x["solve_s"]) for x in solves], 99))
                         if solves else None),
           "slot_solve_p95": _slot_p95(recs, "solve_s"),
           "replan_seeds": [[round(x["s0"], 1), round(x["wall"], 4),
                             {k: [round(v["solve_s"], 4), v["iters"], v["start"]]
                              for k, v in x["seeds"].items()}] for x in recs],
           "slot_iters_p95": _slot_p95(recs, "iters"),
           "per_replan": (float(np.mean([len(x["slots"]) for x in recs])) if recs else 1.0),
           "g_only": (sum(1 for x in recs if x["slots"] == ["G"]) / len(recs)) if recs else 1.0,
           # a re-plan's IPOPT iterations, its slowest start's (the starts run at once): with no
           # clock (RP_DET) the load-free stand-in for L1's time (roadmap 065 M3, R6)
           "iters_p95": (float(np.percentile([max(v["iters"] for v in x["seeds"].values())
                                              for x in recs], 95)) if recs else None),
           "iters_max": (int(max(max(v["iters"] for v in x["seeds"].values()) for x in recs))
                         if recs else None),
           "fallback_ticks": fb, "ticks": len(log),
           "lat_max": float(lat.max()), "lat_rms": float(np.sqrt(np.mean(lat ** 2))),
           "beta_max": float(beta.max()), "beta_plan_max": float(bplan.max()),
           "beta_err_max": float(berr.max()), "beta_err_rms": float(np.sqrt(np.mean(berr ** 2))),
           "off_plan": sum(1 for r in log if r["replan"] == "off_plan"),
           "adopted": _adopted(recs),
           "mode_changes": [(round(x["s0"]), x["mode_before"], x["mode"]) for x in recs
                            if x["mode"] != x["mode_before"]],
           "estimate_min": float(min([r["mu_scale"] for r in log] or [1.0])),
           "style": None if style is None else getattr(d, "style", None).to_dict(),
           "det": os.environ.get("RP_DET") == "1", "rp_root": os.environ.get("RP_ROOT"),
           "itercap": os.environ.get("RP_ITERCAP"), "jit": jit, "prices": prices,
           **obs.result(d),
           **({} if eobs is None else eobs.result()), "ecu_overrides": ecu,
           "force_drift": force_drift, "b_iter": b_iter, "b_budget": b_budget,
           # the car (issue 009): the plant's ECU, its regen slip control, the planner's hold
           "ecu": os.path.dirname(os.path.dirname(os.path.abspath(plant.ecu_mod.__file__))),
           "ecu_regen_slip": getattr(plant.ecu.c, "regen_slip_control", None),
           "regen_hold": d.car.regen_grip_share, "ecu_primed": primed,
           **({} if dig is None else dig.result())}
    os.makedirs(OUT, exist_ok=True)
    key = run_key(name, arm, err, seed, perc, est, rep, tag)
    with open(os.path.join(OUT, key + ".json"), "w") as fh:
        json.dump(res, fh, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    if rec is not None:
        with open(os.path.join(OUT, key + ".traj.json"), "w") as fh:
            json.dump(dict(rec.result(), run=key, course=name, arm=arm), fh,
                      default=lambda o: o.item() if hasattr(o, "item") else str(o))
    print(key, res["result"], res["t_timed"] or res["t_course"], "wall", round(wall),
          "load", load0, flush=True)
    return res


class _DriftObs:
    """The plant's drift (roadmap 065 M2): at every plant step the rear axle's slip angle
    against its tyre's peak on the true surface there (modes.rear_peak at the world's friction);
    the metres driven past_deg or more past it, in all and per corner of the planner's map.
    Observes only (the plant's states are read, never changed)."""

    def __init__(self, car, plant, world, past_deg):
        self.car, self.world, self.past = car, world, math.radians(past_deg)
        self.rows, self._pk = [], {}
        step = plant.step

        def step_(*a, **kw):
            st = step(*a, **kw)
            self.rows.append((st["s"], st["vx"], st["vy"], st["r"], st["i"]))
            return st
        plant.step = step_

    def _peak(self, i, grip=1.0):
        if (i, grip) not in self._pk:
            from race_planner.track import TYRE_FIELDS
            w = self.world
            self._pk[i, grip] = modes.rear_peak(
                self.car, grip * float(w.mu[i]), float(w.sliding[i]),
                tuple(float(getattr(w, k)[i]) for k in TYRE_FIELDS))
        return self._pk[i, grip]

    def result(self, d):
        if len(self.rows) < 2:
            return {"plant_drift_m": 0.0, "plant_drift_share": 0.0}
        a = np.array(self.rows)
        s, vx, vy, r, i = a.T
        past = np.abs(np.arctan2(vy - self.car.lr * r, np.maximum(vx, 0.1))) - np.array(
            [self._peak(int(k)) for k in i])
        ds = np.diff(s, prepend=s[0]).clip(min=0.0)
        on = past >= self.past
        # and against the planner's peak (grip times the true friction), the one its plans are
        # called a drift by (multistart.classify; roadmap 065 M3): 1.25 deg earlier on asphalt
        rear = np.abs(np.arctan2(vy - self.car.lr * r, np.maximum(vx, 0.1)))
        on_g = rear - np.array([self._peak(int(k), GRIP) for k in i]) >= self.past
        out = {"plant_drift_m": round(float(ds[on].sum()), 1),
               "plant_drift_share": round(float(ds[on].sum() / max(ds.sum(), 1e-9)), 4),
               "plant_past_max_deg": round(float(np.degrees(past.max())), 2),
               "plant_drift_m_planner": round(float(ds[on_g].sum()), 1)}
        if d.multi is not None:
            per = {}
            for c in d.multi.corners:
                z = (s >= c.s_in - 30.0) & (s <= c.s_out)
                if z.any():
                    per[round(c.s_in, 1)] = round(float(ds[z & on].sum()), 1)
            out["plant_drift_corners"] = per
            out["plant_drift_corners_planner"] = {
                round(c.s_in, 1): round(float(ds[(s >= c.s_in - 30.0) & (s <= c.s_out) & on_g]
                                              .sum()), 1) for c in d.multi.corners}
            lap = getattr(d.multi, "s2", None)
            if lap is not None:
                out["s2"] = dict(lap.summary(), spent=round(lap.account.spent, 4),
                                 debits=lap.account.debits)
                out["s2_gate"] = [dict(r["s2"], s0=round(r["s0"], 1), chosen=r.get("chosen"),
                                       mode=r.get("mode")) for r in d.multi.records
                                  if r.get("s2")]
        return out


def _force_drift(m):
    """E4's 'D forced' (roadmap 065): S1's gate adopts the fastest feasible drift plan whenever
    a start has one, whatever it costs; else the gate as it is. An experiment's arm, never the
    planner's."""
    orig = m._choose_s1

    def forced(res, rec=None):
        rec = {} if rec is None else rec
        dr = [k for k, r in res.items() if r["ok"] and r.get("mode") == "drift"]
        if not dr:
            return orig(res, rec)
        k = min(dr, key=lambda k: res[k]["time"])
        rec.update(mode_before=m.mode, chosen=k, mode="drift", count=0, forced=True)
        m.mode, m.count = "drift", 0
        return k
    m._choose_s1 = forced


class _EcuObs:
    """The car's ECU against a planned slide (roadmap 065 E4): at every ECU step its traction
    control's and ESC's motor factors, ESC's wheel brake, the regen slip control's and ABS's
    factors; at every plant step whether the adopted plan slides there (its rear slip past the
    planner's peak by past_deg: the plan, not the car), and the car's tracking of it. Events are
    an intervention's starts (a factor leaving 1, a brake rising past ESC_ON), counted where the
    plan slides. Observes only."""
    ESC_ON = 50.0                            # N m on one wheel: an ESC brake event

    def __init__(self, d, car, plant, drift, past_deg):
        from race_planner.live import plan_at
        self.d, self.car, self.drift, self.past = d, car, drift, math.radians(past_deg)
        self.rows, self.acc = [], None
        ecu, estep, pstep = plant.ecu, plant.ecu.step, plant.step

        def estep_(*a, **kw):
            out = estep(*a, **kw)
            pt, esc = ecu.powertrain, ecu.esc.out
            v = (getattr(getattr(pt, "tc", None), "factor", 1.0), esc.motor_factor,
                 max(esc.brake), getattr(getattr(pt, "rsc", None), "factor", 1.0),
                 min([m.factor for m in getattr(pt, "abs", [])] or [1.0]))
            a_ = self.acc
            self.acc = v if a_ is None else (min(a_[0], v[0]), min(a_[1], v[1]),
                                             max(a_[2], v[2]), min(a_[3], v[3]), min(a_[4], v[4]))
            return out

        def pstep_(*a, **kw):
            self.acc = None
            st = pstep(*a, **kw)
            p = self.d.plan
            planned, n_err, slip_err = False, 0.0, 0.0
            if p is not None and p["s"][0] <= st["s"] <= p["s"][-1]:
                X, _, _ = plan_at(p, st["s"])
                rp = abs(math.atan2(X[1] - car.lr * X[2], max(X[0], 0.1)))
                rc = abs(math.atan2(st["vy"] - car.lr * st["r"], max(st["vx"], 0.1)))
                planned = rp - self.drift._peak(int(st["i"]), GRIP) >= self.past
                n_err, slip_err = abs(st["n"] - X[3]), math.degrees(rc - rp)
            self.rows.append((st["t"], st["s"], planned, n_err, slip_err)
                             + (self.acc or (1.0, 1.0, 0.0, 1.0, 1.0)))
            return st
        ecu.step, plant.step = estep_, pstep_

    def result(self):
        if len(self.rows) < 2:
            return {"ecu_obs": None}
        a = np.array(self.rows, float)
        t, s, pl, n_err, slip_err, tc, escm, escb, rsc, abs_ = a.T
        pl = pl > 0.5
        dt = np.diff(t, prepend=t[0])
        flags = {"tc_cut": tc < 0.999, "esc_cut": escm < 0.999, "esc_brake": escb > self.ESC_ON,
                 "regen_cut": rsc < 0.999, "abs": abs_ < 0.999}
        out = {"planned_s": round(float(dt[pl].sum()), 2),
               "planned_m": round(float(np.diff(s, prepend=s[0]).clip(min=0)[pl].sum()), 1),
               "n_err_rms_planned": (round(float(np.sqrt(np.mean(n_err[pl] ** 2))), 3)
                                     if pl.any() else None),
               "slip_err_mean_planned": (round(float(np.mean(slip_err[pl])), 2)
                                         if pl.any() else None),
               "slip_err_rms_planned": (round(float(np.sqrt(np.mean(slip_err[pl] ** 2))), 2)
                                        if pl.any() else None),
               "n_err_rms": round(float(np.sqrt(np.mean(n_err ** 2))), 3),
               "tc_min_planned": round(float(tc[pl].min()), 3) if pl.any() else None,
               "esc_brake_max_planned": round(float(escb[pl].max()), 0) if pl.any() else None}
        for k, f in flags.items():
            start = f & ~np.concatenate([[False], f[:-1]])
            out[k + "_events"] = int(start.sum())
            out[k + "_events_planned"] = int((start & pl).sum())
            out[k + "_s"] = round(float(dt[f].sum()), 2)
            out[k + "_s_planned"] = round(float(dt[f & pl].sum()), 2)
        return {"ecu_obs": out}


class _digest:
    """sha256 of every plan the driver makes (its first and each re-plan's, ok or not: X, U, s,
    t) and of every plant state, in order; plus each plan's own short hash, to find where two
    runs part."""

    def __init__(self, d, plant):
        import hashlib
        self.plans, self.traj, self.each = hashlib.sha256(), hashlib.sha256(), []
        start, replan, step = d.start, d._replan, plant.step

        def add(p):
            h = hashlib.sha256()
            for k in ("X", "U", "s", "t"):
                h.update(np.ascontiguousarray(np.asarray(p[k], float)).tobytes())
            self.plans.update(h.digest())
            self.each.append(h.hexdigest()[:12])

        def start_(state):
            out = start(state)
            add(d.plan)                      # the first plan (after the first re-plan's)
            return out

        def replan_(state, adopt_now=False):
            n = len(d.solves)
            out = replan(state, adopt_now)
            if len(d.solves) > n:            # the solve's plan if adopted, else its marker
                add(d.pending[1] if d.pending is not None else
                    {"X": [len(d.solves)], "U": [0.0], "s": [0.0], "t": [0.0]})
            return out

        def step_(*a, **kw):
            st = step(*a, **kw)
            self.traj.update(np.array([float(st[k]) for k in sorted(st)
                                       if isinstance(st[k], (int, float))]).tobytes())
            return st

        d.start, d._replan, plant.step = start_, replan_, step_
        self.d = d

    def result(self):
        return {"plans_digest": self.plans.hexdigest(), "traj_digest": self.traj.hexdigest(),
                "n_plans": len(self.each), "plan_hashes": self.each}


class _Traj:
    """--traj: the run tick by tick, for a video (tools/arm_video.py): the car (x, y, yaw of the
    CoM, speed, sideslip, steering), each wheel's slip against its tyre's peak (model.tyre's
    combined slip over the surface's peak, 1 at the peak: fl, fr, rl, rr), the plan the car is
    on (an index into `plans`: each adopted plan's s, n, sideslip, speed, its seed G/H/D/F and
    mode) and the road (the world's centreline and edges). Reads only: the run is the same
    with it or without (its digests too)."""

    def __init__(self, d, car, world):
        import casadi as ca
        from race_planner import model
        xs, us, acc = ca.SX.sym("x", 3), ca.SX.sym("u", 5), ca.SX.sym("acc", 2)
        sf = ca.SX.sym("surface", 4)
        o = model.dynamics(car, [xs[0], xs[1], xs[2]], [us[i] for i in range(5)], 1.0,
                           [acc[0], acc[1]], 1.0, sf)
        self.f = ca.Function("slip", [xs, us, acc, sf],
                             [ca.vertcat(*o["kappa"]), ca.vertcat(*o["alpha"])])
        self.kp, self.tp = model.peaks(car)
        self.car, self.world, self.d = car, world, d
        from race_planner.serve import PlanIds
        self.rows, self.plans, self.ids = [], [], PlanIds()
        from race_planner.track import TYRE_FIELDS
        self.tyre = np.vstack([getattr(world, k) for k in TYRE_FIELDS])
        step = d.step

        def step_(state):
            cmd = step(state)
            self.add(state)
            return cmd
        d.step = step_

    def add(self, st):
        R = self.car.r_wheel
        i = int(st["i"])
        sf = self.tyre[:, i]
        u = [st["delta"]] + list(np.asarray(st["w"], float) * R)
        kap, alp = (np.asarray(q, float).ravel() for q in
                    self.f([st["vx"], st["vy"], st["r"]], u, [st["ax"], st["ay"]], sf))
        rho = np.hypot(kap / (self.kp * sf[2]), alp / (self.tp * sf[3]))
        from race_planner.serve import plan_message
        pm = plan_message(self.d, self.ids)
        if "plan" in pm:
            self.plans.append(dict(pm["plan"], t=round(st["t"], 3), slot=pm["slot"],
                                   mode=pm["mode"]))
        j = pm["plan_id"]
        self.rows.append({"t": round(st["t"], 3), "s": round(st["s"], 2), "n": round(st["n"], 3),
                          "x": round(st["x"], 3), "y": round(st["y"], 3),
                          "yaw": round(st["psi"], 4),
                          "v": round(math.hypot(st["vx"], st["vy"]), 3),
                          "beta_deg": round(math.degrees(math.atan2(st["vy"],
                                                                    max(st["vx"], 0.5))), 2),
                          "delta_deg": round(math.degrees(st["delta"]), 2),
                          "slip": np.round(rho, 3).tolist(),
                          "kappa": np.round(kap, 4).tolist(),
                          "alpha_deg": np.round(np.degrees(np.arctan(alp)), 2).tolist(),
                          "plan": j, "stopping": self.d.stopping})

    def result(self):
        w = self.world
        k = slice(None, None, 2)
        return {"wheels": "fl fr rl rr", "car": {"lf": self.car.lf, "lr": self.car.lr,
                                                "track": self.car.track,
                                                "width": self.car.width},
                "road": {f: np.round(np.asarray(getattr(w, f), float)[k], 3).tolist()
                         for f in ("s", "x", "y", "psi", "w_left", "w_right")},
                "rows": self.rows, "plans": self.plans}


def _slot_p95(recs, key):
    """Each start's p95 of `key` over the re-plans it ran in (B's against G's, R6)."""
    out = {}
    for x in recs:
        for slot, v in x["seeds"].items():
            out.setdefault(slot, []).append(float(v[key]))
    return {k: round(float(np.percentile(v, 95)), 4) for k, v in out.items()}


def _adopted(recs):
    out = {}
    for x in recs:
        k = x.get("chosen")
        if k:
            out[k] = out.get(k, 0) + 1
    return out


def run_key(name, arm, err=0, seed=0, perc=False, est=False, rep=1, tag=""):
    return (name.replace(":", "_") + f"__{arm}" + (f"_e{err}s{seed}" if err else "")
            + ("_perc" if perc else "") + ("_est" if est else "") + f"_r{rep}"
            + (f"_{tag}" if tag else ""))


# ---- the oracle ----

_W = {}


def _window(car, held=None):
    from race_planner.window import Window
    if held not in _W:
        kw = {} if held is None else {"rear_alpha_max": math.radians(held)}
        _W[held] = Window(car, ds=DS, length=ORACLE_LEN, grip=GRIP, warm=False, tol=1e-6,
                          max_iter=3000, **kw)
    return _W[held]


def oracle(name):
    """The corner's best-of-starts plans at the loop's settings (module docstring)."""
    from race_planner.window import initial
    car = Car.from_spec(e1.SPEC)
    planner, _, _, s0, v0, span, info = tracks(name, car)
    i0 = int(round((s0 - planner.s[0]) / DS))
    tr = planner.extended(ORACLE_LEN + 4 * DS)
    c = modes.corners(planner, GRIP)[0]
    peak = modes.rear_peak(car, c.mu, c.sliding, c.surface)
    bound = held_bound(peak)
    wf, wh = _window(car), _window(car, bound)
    sec = wf.section(tr, i0)
    x0 = np.array([v0, 0, 0, 0, 0, 0.0])
    X0, U0 = initial(sec, car, wf.margin, v0)
    X0[:, 0] = x0

    def solve(w, X, U):
        X = np.array(X, float)
        X[:, 0] = x0
        w.lam = w.lam_x = None
        c0 = time.time()
        p = w.solve(tr, i0, x0, 1e3, X, U)
        p["wall"] = time.time() - c0
        ok = p["ok"]
        tt = float(np.interp(span[1], p["s"], p["t"])) if ok else None
        beta = float(np.degrees(np.max(np.abs(np.arctan2(p["X"][1], p["X"][0]))))) if ok else None
        return p, tt, beta

    rows = {}
    free = {}
    p, tt, b = solve(wf, X0, U0)
    free["G"] = (p, tt, b)
    lib = modes.Library()
    i = lib.nearest(c)
    if i is not None:
        free["Dc"] = solve(wf, *lib.seed(i, X0, U0, sec, c, car))
    free["Ds"] = solve(wf, *modes.seed_rear_out(X0, U0, sec, c.s_in, c.s_out, car, peak, c.mu,
                                                c.sign))
    if c.hairpin:
        free["F"] = solve(wf, *modes.seed_flick(X0, U0, sec, c.s_in, c.s_out, car, peak, c.mu,
                                                c.sign))
    held = {"H": solve(wh, X0, U0)}
    tb = math.tan(math.radians(bound) - math.radians(0.2))
    for k, (p, tt, b) in list(free.items()):
        if tt is not None and k == "G":
            X = np.array(p["X"], float)
            cr = car.lr * X[2]
            X[1] = np.clip(X[1], cr - tb * X[0], cr + tb * X[0])
            held["H<G"] = solve(wh, X, p["U"])
    hp = held["H"]
    if hp[1] is not None:
        free["G<H"] = solve(wf, hp[0]["X"], hp[0]["U"])
    for k, (p, tt, b) in list(free.items()) + [(k, v) for k, v in held.items()]:
        rows[k] = {"t": tt, "beta": b, "iters": int(p["iters"]), "wall": round(p["wall"], 2),
                   "status": p["status"], "past": _past(car, p, tr, i0) if tt else None}
    okf = {k: v for k, v in free.items() if v[1] is not None}
    okh = {k: v for k, v in held.items() if v[1] is not None}
    best = min(okf, key=lambda k: okf[k][1]) if okf else None
    bh = min(okh, key=lambda k: okh[k][1]) if okh else None
    res = {"course": name, "info": info, "bound": bound, "peak_deg": math.degrees(peak),
           "starts": rows, "best": best, "t_best": okf[best][1] if best else None,
           "held": bh, "t_held": okh[bh][1] if bh else None,
           "beta_best": okf[best][2] if best else None}
    if best and bh:
        res["gain_s"] = res["t_held"] - res["t_best"]
        res["gain_pct"] = 100 * res["gain_s"] / res["t_best"]
    os.makedirs(os.path.join(OUT, "oracle"), exist_ok=True)
    with open(os.path.join(OUT, "oracle", name.replace(":", "_") + ".json"), "w") as fh:
        json.dump(res, fh)
    print(name, best, res.get("t_best"), bh, res.get("t_held"), res.get("gain_pct"), flush=True)
    return res


def _past(car, p, tr, i0):
    """The plan's rear axle past its tyre's peak, largest (deg)."""
    from race_planner.track import TYRE_FIELDS
    N = p["X"].shape[1]
    sl = slice(i0, i0 + N)
    past = modes.past_peak(car, p["X"], GRIP * tr.mu[sl], tr.sliding[sl],
                           np.vstack([getattr(tr, k)[sl] for k in TYRE_FIELDS]))
    return float(np.max(past))


# ---- the report ----

def _rows():
    import glob
    return _rows_in(OUT)


def _oracles():
    import glob
    return {json.load(open(f))["course"]: json.load(open(f))
            for f in glob.glob(os.path.join(OUT, "oracle", "*.json"))}


def _stop(r):
    return r["result"] != "done"


def _ms(v):
    v = [x for x in v if x is not None]
    if not v:
        return "-"
    return f"{np.mean(v):.2f}" + (f" ± {(max(v) - min(v)) / 2:.2f}" if len(v) > 1 else "")


def _arm(r):
    return (r["arm"] + ("+perc" if r["perc"] else "") + ("+est" if r["est"] else "")
            + (f" [{r['tag']}]" if r.get("tag") else ""))


def _summary(rs):
    n = len(rs)
    ticks = sum(r["ticks"] for r in rs) or 1
    rp = sum(r["replans"] for r in rs) or 1
    return {"n": n, "done": sum(1 for r in rs if not _stop(r)),
            "stops": ", ".join(sorted({r["result"].split(":")[0] + ("@%d" % r["s"] if r.get("s")
                                                                     else "")
                                       for r in rs if _stop(r)})),
            "failed": sum(r["failed"] for r in rs), "replans": rp,
            "fb": 100.0 * sum(r["fallback_ticks"] for r in rs) / ticks,
            "lat": max(r["lat_max"] for r in rs), "lat_med": float(np.median([r["lat_max"] for r in rs])),
            "berr": float(np.median([r["beta_err_max"] for r in rs])),
            "over": sum(r["overruns"] for r in rs),
            "per": float(np.mean([r["per_replan"] for r in rs])),
            "load": max(max(r["load0"][0], r["load1"][0]) for r in rs)}


def report():
    from race_planner.schedule import DriftMap
    rows, orc, dmap = _rows(), _oracles(), DriftMap.load()
    L = ["# 061 M4: model in the loop (sim.Plant, ECU, L2)", ""]
    H = ("| {k} | arm | runs | done | stops | failed re-plans | L2 fallback ticks | lat. err. max "
         "(median of runs / worst) | sideslip err. max (median) | overruns > 1 s | solves / "
         "re-plan | load max |")
    sep = "|---" * 12 + "|"

    def line(key, arm, rs):
        x = _summary(rs)
        return (f"| {key} | {arm} | {x['n']} | {x['done']} | {x['stops']} | {x['failed']} / "
                f"{x['replans']} | {x['fb']:.1f} % | {x['lat_med']:.2f} / {x['lat']:.2f} m | "
                f"{x['berr']:.1f} deg | {x['over']} | {x['per']:.2f} | {x['load']:.1f} |")

    corners = [r for r in rows if r["course"].startswith("c:")]
    if corners:
        L += ["## Corners: robustness by class (R 9/15/25, 90/180 deg, 6 and 15 m, all repeats)",
              "", H.format(k="class"), sep]
        for cls in CLASSES:
            for arm in ("single", "always", "sched"):
                rs = [r for r in corners if r["info"]["class"] == cls and _arm(r) == arm]
                if rs:
                    L.append(line(cls, arm, rs))
        L += ["", "## Corners: time and the oracle's gain kept", "",
              "Time from the start to 50 m past the exit, mean ± half range over repeats (a run "
              "that did not finish: its result). Oracle: best-of-starts at the loop's settings "
              "(best start, held time -> best time, gain %, the plan's largest sideslip). Gain "
              "kept, on corners the drift map calls drift-optimal (map), in units of the oracle's "
              "gain: single, (oracle held - loop) / (held - best), the loop's tracking loss "
              "included; always and sched, what they add over the single start, (single - arm) "
              "/ (held - best).", "",
              "| corner | map | oracle | single | always | sched | kept: single / +always / "
              "+sched |", "|---|---|---|---|---|---|---|"]
        for c in corner_courses():
            rs = [r for r in corners if r["course"] == c]
            if not rs:
                continue
            o = orc.get(c, {})
            _, cls, R, turn, w = c.split(":")
            pays = dmap.pays(cls, float(R), math.radians(float(turn)), float(w))
            cell, kept, t_single = [], [], None
            for arm in ("single", "always", "sched"):
                ra = [r for r in rs if _arm(r) == arm]
                ts = [r["t_timed"] for r in ra if r["result"] == "done"]
                bad = [r["result"].split(":")[0] for r in ra if r["result"] != "done"]
                cell.append(_ms(ts) + (" (" + ", ".join(bad) + ")" if bad else ""))
                if arm == "single":
                    t_single = np.mean(ts) if ts else None
                    k = ((o["t_held"] - np.mean(ts)) / o["gain_s"]
                         if pays and o.get("gain_s") and ts else None)
                else:          # what the extra starts add over the single start
                    k = ((t_single - np.mean(ts)) / o["gain_s"]
                         if pays and o.get("gain_s") and ts and t_single is not None else None)
                kept.append("-" if k is None else f"{k:+.2f}")
            oc = (f"{o.get('best')} {o['t_held']:.2f} -> {o['t_best']:.2f} "
                  f"({o['gain_pct']:+.2f} %, {o['beta_best']:.0f} deg)" if o.get("gain_s")
                  is not None else "-")
            L.append(f"| {c[2:]} | {'drift' if pays else 'grip'} | {oc} | {' | '.join(cell)} | "
                     f"{' / '.join(kept)} |")
    zones = [r for r in rows if r["course"] == "zones"]
    if zones:
        L += ["", "## The class-zone chain: true and wrong maps", "",
              H.format(k="map wrong") .replace("| load max |", "| course time | load max |"),
              sep + "---|"]
        keys = sorted({(r["err"], _arm(r)) for r in zones})
        for err, arm in keys:
            rs = [r for r in zones if r["err"] == err and _arm(r) == arm]
            ts = [r["t_course"] for r in rs if r["result"] == "done"]
            x = line(f"{err} %", arm, rs)
            L.append(x[:x.rfind("|", 0, -1)] + f"| {_ms(ts)} |" + x[x.rfind("|", 0, -1) + 1:])
    dry = [r for r in rows if r["course"] in ("haruna", "tsukuba")]
    if dry:
        L += ["", "## Dry courses (R6)", "",
              "| course | arm | runs | done | time (s) | vs single | solves / re-plan | G only | "
              "failed | overruns | L2 fallback ticks | lat. err. max | load max |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
        for c in ("haruna", "tsukuba"):
            base = [r["t_course"] for r in dry if r["course"] == c and r["arm"] == "single"
                    and r["t_course"]]
            for arm in ("single", "sched", "always"):
                rs = [r for r in dry if r["course"] == c and _arm(r) == arm]
                if not rs:
                    continue
                ts = [r["t_course"] for r in rs if r["t_course"]]
                vs = (f"{100 * (np.mean(ts) - np.mean(base)) / np.mean(base):+.3f} %"
                      if ts and base and arm != "single" else "")
                x = _summary(rs)
                L.append(f"| {c} | {arm} | {x['n']} | {x['done']} | {_ms(ts)} | {vs} | "
                         f"{x['per']:.2f} | {np.mean([r['g_only'] for r in rs]) * 100:.0f} % | "
                         f"{x['failed']} | {x['over']} | {x['fb']:.1f} % | {x['lat']:.2f} | "
                         f"{x['load']:.1f} |")
    text = "\n".join(L) + "\n"
    open(os.path.join(OUT, "report.md"), "w").write(text)
    print(text)


# ---- the aggregate (the write-up's tables) ----

def _fail(r):
    """A run's outcome as a category: done, no first plan (the car never starts), off_road,
    spin (the driver's spin stop or the plant's 80 deg), solves failed, timeout, error."""
    x = r["result"]
    if x == "done":
        return "done"
    if "no first plan" in x:
        return "no first plan"
    if x.startswith("stopped: spin") or x == "spun":
        return "spin"
    if x.startswith("stopped: solves failed"):
        return "solves failed"
    return x.split(":")[0]


FAILS = ("done", "no first plan", "off_road", "spin", "solves failed", "timeout", "error")


def _rows_in(d):
    import glob
    return [json.load(open(f)) for f in sorted(glob.glob(os.path.join(d, "*.json")))
            if os.path.getsize(f) and os.path.basename(f) != "aggregate.json"]


def aggregate():
    """The write-up's tables from every result under OUT (OUT/quiet: the dry courses re-run on an
    idle host; OUT/beta45: L1's sideslip bound at 45 deg on mud_wet and snow_loose)."""
    from race_planner.schedule import DriftMap
    rows, orc, dmap = _rows_in(OUT), _oracles(), DriftMap.load()
    agg = {}
    L = ["# 061 M4: the aggregate (studies/mode_choice_m4.py aggregate)", ""]
    corners = [r for r in rows if r["course"].startswith("c:") and not r.get("tag")]
    arms = ("single", "always", "sched")

    # outcomes
    L += ["## Corners: outcomes by class (runs; 12 corners x 2 repeats an arm)", "",
          "| class | arm | " + " | ".join(FAILS) + " |", "|---|---" + "|---" * len(FAILS) + "|"]
    for cls in CLASSES:
        for arm in arms:
            rs = [r for r in corners if r["info"]["class"] == cls and r["arm"] == arm]
            n = {f: sum(1 for r in rs if _fail(r) == f) for f in FAILS}
            agg.setdefault("outcomes", {})[f"{cls}/{arm}"] = n
            L.append(f"| {cls} | {arm} | " + " | ".join(str(n[f] or "") for f in FAILS) + " |")

    # robustness, without the runs that never started (the same in every arm)
    H = ("| {k} | arm | runs | done | stops | failed re-plans | L2 fallback ticks | lat. err. max "
         "(median of runs / worst) | sideslip err. max (median) | overruns > 1 s | solves / "
         "re-plan | load max |")
    sep = "|---" * 12 + "|"

    def line(key, arm, rs):
        x = _summary(rs)
        return (f"| {key} | {arm} | {x['n']} | {x['done']} | {x['stops']} | {x['failed']} / "
                f"{x['replans']} | {x['fb']:.1f} % | {x['lat_med']:.2f} / {x['lat']:.2f} m | "
                f"{x['berr']:.1f} deg | {x['over']} | {x['per']:.2f} | {x['load']:.1f} |")

    started = [r for r in corners if _fail(r) != "no first plan"]
    L += ["", "## Corners: robustness by class (runs that started)", "", H.format(k="class"), sep]
    low = {a: {"stops": 0, "failed": 0, "replans": 0, "runs": 0} for a in arms}
    for cls in CLASSES:
        for arm in arms:
            rs = [r for r in started if r["info"]["class"] == cls and r["arm"] == arm]
            if rs:
                L.append(line(cls, arm, rs))
                if cls != "asphalt":
                    x = _summary(rs)
                    low[arm]["stops"] += x["n"] - x["done"]
                    low[arm]["failed"] += x["failed"]
                    low[arm]["replans"] += x["replans"]
                    low[arm]["runs"] += x["n"]
    agg["low_friction"] = low
    L += ["", "Low friction (every class but asphalt), runs that started: " + "; ".join(
        f"{a} {v['stops']} stops in {v['runs']} runs, {v['failed']} failed re-plans of "
        f"{v['replans']}" for a, v in low.items())]

    # time and the oracle's gain
    L += ["", "## Corners: time and the oracle's gain kept", "",
          "Time from the start to 50 m past the exit (mean ± half range of 2 repeats; a run that "
          "did not finish: its outcome). Oracle: best-of-starts offline at the loop's settings "
          "(best start: held -> best time, gain %, the plan's largest sideslip). Kept: what the "
          "arm adds over the single start in units of the oracle's gain, (single - arm) / (held "
          "- best), on corners the drift map calls drift-optimal; `*` where the difference is "
          "larger than the two arms' repeat spread.", "",
          "| corner | map | oracle | single | always | sched | kept: always / sched |",
          "|---|---|---|---|---|---|---|"]
    per_cls = {}
    for c in corner_courses():
        rs = [r for r in corners if r["course"] == c]
        if not rs:
            continue
        o = orc.get(c, {})
        _, cls, R, turn, w = c.split(":")
        pays = dmap.pays(cls, float(R), math.radians(float(turn)), float(w))
        cell, ts_arm = [], {}
        for arm in arms:
            ra = [r for r in rs if r["arm"] == arm]
            ts = [r["t_timed"] for r in ra if r["result"] == "done" and r["t_timed"]]
            bad = [_fail(r) for r in ra if r["result"] != "done"]
            ts_arm[arm] = ts if len(ts) == len(ra) else None
            cell.append(_ms(ts) + (" (" + ", ".join(bad) + ")" if bad else ""))
        kept = []
        for arm in ("always", "sched"):
            a, b = ts_arm["single"], ts_arm[arm]
            if pays and o.get("gain_s") and a and b:
                k = (np.mean(a) - np.mean(b)) / o["gain_s"]
                spread = (max(a) - min(a)) / 2 + (max(b) - min(b)) / 2
                sig = abs(np.mean(a) - np.mean(b)) > spread
                kept.append(f"{k:+.2f}" + ("*" if sig else ""))
                d = per_cls.setdefault(cls, {}).setdefault(arm, {"dt": 0.0, "gain": 0.0,
                                                                 "corners": 0, "sig": 0})
                d["dt"] += np.mean(a) - np.mean(b)
                d["gain"] += o["gain_s"]
                d["corners"] += 1
                d["sig"] += int(sig)
            else:
                kept.append("-")
        oc = (f"{o.get('best')} {o['t_held']:.2f} -> {o['t_best']:.2f} "
              f"({o['gain_pct']:+.2f} %, {o['beta_best']:.0f} deg)" if o.get("gain_s")
              is not None else "-")
        L.append(f"| {c[2:]} | {'drift' if pays else 'grip'} | {oc} | {' | '.join(cell)} | "
                 f"{' / '.join(kept)} |")
    agg["kept"] = per_cls
    L += ["", "Per class, on the drift-optimal corners every arm finished in both repeats: "
          "share of the oracle's summed gain the arm adds over the single start (corners; "
          "corners whose difference clears the repeat spread).", "",
          "| class | always | sched |", "|---|---|---|"]
    for cls in CLASSES:
        d = per_cls.get(cls, {})
        L.append(f"| {cls} | " + " | ".join(
            (f"{d[a]['dt'] / d[a]['gain']:+.2f} ({d[a]['corners']}; {d[a]['sig']})"
             if a in d and d[a]["gain"] else "-") for a in ("always", "sched")) + " |")

    # time per class against the single start, every corner
    L += ["", "Time against the single start, every corner all arms finished (mean of the "
          "corners' %; spread: the median repeat half range, %).", "",
          "| class | always | sched | repeat spread |", "|---|---|---|---|"]
    for cls in CLASSES:
        d = {"always": [], "sched": []}
        spr = []
        for c in corner_courses():
            if c.split(":")[1] != cls:
                continue
            t = {a: [r["t_timed"] for r in corners if r["course"] == c and r["arm"] == a
                     and r["result"] == "done" and r["t_timed"]] for a in arms}
            if all(len(v) == 2 for v in t.values()):
                for a in ("always", "sched"):
                    d[a].append(100 * (np.mean(t[a]) - np.mean(t["single"]))
                                / np.mean(t["single"]))
                spr += [100 * (max(v) - min(v)) / 2 / np.mean(v) for v in t.values()]
        L.append(f"| {cls} | " + " | ".join(f"{np.mean(d[a]):+.2f} % ({len(d[a])})" if d[a]
                                             else "-" for a in ("always", "sched"))
                 + f" | {np.median(spr):.2f} % |" if spr else f"| {cls} | - | - | - |")

    # the zone chain, true and wrong maps
    zones = [r for r in rows if r["course"] == "zones" and not r.get("tag")]
    if zones:
        L += ["", "## The class-zone chain: true and wrong maps", "",
              "| map wrong | arm | runs | done | outcomes | failed re-plans | solves / re-plan | "
              "estimate min | course time | load max |", "|---|---|---|---|---|---|---|---|---|---|"]
        for err, arm in sorted({(r["err"], _arm(r)) for r in zones}):
            rs = [r for r in zones if r["err"] == err and _arm(r) == arm]
            ts = [r["t_course"] for r in rs if r["result"] == "done"]
            x = _summary(rs)
            outs = ", ".join(f"{_fail(r)}@{r['s']:.0f}" if r["result"] != "done" else "done"
                             for r in rs)
            L.append(f"| {err} % | {arm} | {x['n']} | {x['done']} | {outs} | {x['failed']} / "
                     f"{x['replans']} | {x['per']:.2f} | "
                     f"{min(r['estimate_min'] for r in rs):.2f} | {_ms(ts)} | {x['load']:.1f} |")
            agg.setdefault("zones", {})[f"{err}/{arm}"] = {"runs": x["n"], "done": x["done"],
                                                         "failed": x["failed"]}

    # dry courses
    for sub, title in (("", "loaded host"), ("quiet", "idle host")):
        dry = [r for r in (rows if not sub else _rows_in(os.path.join(OUT, sub)))
               if r["course"] in ("haruna", "tsukuba")]
        if not dry:
            continue
        L += ["", f"## Dry courses (R6), {title}", "",
              "| course | arm | runs | done | time (s) | vs single | solves / re-plan | G only | "
              "failed | overruns | lat. err. max | load (start, max) |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|"]
        for c in ("haruna", "tsukuba"):
            base = [r["t_course"] for r in dry if r["course"] == c and r["arm"] == "single"
                    and r["t_course"]]
            for arm in ("single", "sched", "always"):
                rs = [r for r in dry if r["course"] == c and r["arm"] == arm]
                if not rs:
                    continue
                ts = [r["t_course"] for r in rs if r["t_course"]]
                vs = (f"{100 * (np.mean(ts) - np.mean(base)) / np.mean(base):+.3f} %"
                      if ts and base and arm != "single" else "")
                x = _summary(rs)
                l0 = min(r["load0"][0] for r in rs)
                L.append(f"| {c} | {arm} | {x['n']} | {x['done']} | "
                         f"{', '.join(f'{t:.2f}' for t in ts)} ({_ms(ts)}) | {vs} | "
                         f"{x['per']:.2f} | {np.mean([r['g_only'] for r in rs]) * 100:.0f} % | "
                         f"{x['failed']} | {x['over']} | {x['lat']:.2f} | {l0:.1f}, "
                         f"{x['load']:.1f} |")
                agg.setdefault("dry", {})[f"{sub or 'loaded'}/{c}/{arm}"] = {
                    "t": ts, "per_replan": x["per"],
                    "g_only": float(np.mean([r["g_only"] for r in rs]))}

    # the sideslip bound
    bdir = os.path.join(OUT, "beta45")
    brs = _rows_in(bdir) if os.path.isdir(bdir) else []
    if brs:
        L += ["", "## L1's sideslip bound, 45 deg (mud_wet, snow_loose)", "",
              "Each corner and arm against the same run free (repeat 1 and 2 pooled): outcome, "
              "time, the plans' and the car's largest sideslip, the sideslip error; cost: bound - "
              "free, in % and in units of the oracle's gain.", "",
              "| corner | arm | free: outcome, t, plan / car / err deg | bound 45: outcome, t, "
              "plan / car / err deg | cost % | cost / oracle gain |", "|---|---|---|---|---|---|"]
        tot = {}
        for c in corner_courses():
            for arm in ("single", "sched"):
                fb = [r for r in corners if r["course"] == c and r["arm"] == arm]
                bb = [r for r in brs if r["course"] == c and r["arm"] == arm]
                if not bb or not fb:
                    continue

                def desc(rs):
                    ts = [r["t_timed"] for r in rs if r["result"] == "done" and r["t_timed"]]
                    outs = sorted({_fail(r) for r in rs})
                    return ts, (f"{'/'.join(outs)}, {_ms(ts)}, "
                                f"{max(r['beta_plan_max'] for r in rs):.0f} / "
                                f"{max(r['beta_max'] for r in rs):.0f} / "
                                f"{max(r['beta_err_max'] for r in rs):.0f}")
                tf, df = desc(fb)
                tb, db = desc(bb)
                deep = max(r["beta_plan_max"] for r in fb) > 45.0
                cost = cg = "-"
                o = orc.get(c, {})
                if tf and tb and len(tf) == len(fb) and len(tb) == len(bb):
                    dt = np.mean(tb) - np.mean(tf)
                    cost = f"{100 * dt / np.mean(tf):+.2f}"
                    if o.get("gain_s"):
                        cg = f"{dt / o['gain_s']:+.2f}"
                    k = tot.setdefault(arm, {"deep": [], "all": []})
                    k["all"].append(100 * dt / np.mean(tf))
                    if deep:
                        k["deep"].append(100 * dt / np.mean(tf))
                L.append(f"| {c[2:]}{' (deep)' if deep else ''} | {arm} | {df} | {db} | {cost} | "
                         f"{cg} |")
                agg.setdefault("beta45", {})[f"{c}/{arm}"] = {
                    "free": [_fail(r) for r in fb], "bound": [_fail(r) for r in bb],
                    "t_free": tf, "t_bound": tb}
        bz = [r for r in brs if r["course"] == "zones"]
        if bz:
            L += ["", "The class-zone chain at the bound (free: the table above).", "",
                  "| map wrong | arm | runs | outcomes | failed re-plans | solves / re-plan | "
                  "plan / car sideslip max (deg) | course time |", "|---|---|---|---|---|---|---|---|"]
            for err, arm in sorted({(r["err"], _arm(r).split(" [")[0]) for r in bz}):
                rs = [r for r in bz if r["err"] == err and _arm(r).split(" [")[0] == arm]
                ts = [r["t_course"] for r in rs if r["result"] == "done"]
                x = _summary(rs)
                outs = ", ".join(f"{_fail(r)}@{r['s']:.0f}" if r["result"] != "done" else "done"
                                 for r in rs)
                L.append(f"| {err} % | {arm} | {x['n']} | {outs} | {x['failed']} / "
                         f"{x['replans']} | {x['per']:.2f} | "
                         f"{max(r['beta_plan_max'] for r in rs):.0f} / "
                         f"{max(r['beta_max'] for r in rs):.0f} | {_ms(ts)} |")
                agg.setdefault("beta45_zones", {})[f"{err}/{arm}"] = [_fail(r) for r in rs]
        for arm, k in tot.items():
            L.append("")
            L.append(f"{arm}: mean cost {np.mean(k['all']):+.2f} % over {len(k['all'])} corners "
                     f"both finished; on the deep ones (a free plan past 45 deg) "
                     + (f"{np.mean(k['deep']):+.2f} % over {len(k['deep'])}" if k["deep"]
                        else "none both finished") + ".")
    text = "\n".join(L) + "\n"
    open(os.path.join(OUT, "aggregate.md"), "w").write(text)
    with open(os.path.join(OUT, "aggregate.json"), "w") as fh:
        json.dump(agg, fh, indent=1, default=float)
    print(text)


# ---- S2 offline: the lap-time-against-drift curve from the signed map (roadmap 065 M2, E2) ----

S2_OUT = os.path.join(SWB, "play_log/race_planner/style/m2")
BUDGETS_S = (0.0, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0)
EPS_PCT = (0.3, 1.0, 2.0, 3.0)
S1_LAP = {"haruna": 76.28, "tsukuba": 74.33}     # S1's det laps on 95b902e (065 E1)


def course_prices(name, out=None):
    """Each corner of a course priced as the live gate sees it (roadmap 065 M3): a live.Driver on
    the planner's track, the car on the course's grip plan (drift_optimality.grip_plan) from the
    run's start, and from LIVE_D metres before each turn-in the live windows' G, D (Ds, Dc), B
    (the floor) and H, as drift_optimality.live_positions does for a cell. Writes
    <out>/prices_<course>.json: per corner its map price (drift_losses.json, as S2 prices it)
    and the live losses per position, the largest, the drift's start."""
    from race_planner.budget import LossMap, S2Lap
    from race_planner.multistart import s2_classes
    from race_planner.schedule import Scheduler
    from race_planner.style import Style
    out = out or os.path.join(SWB, "play_log/race_planner/style/m3")
    car = Car.from_spec(e1.SPEC)
    planner, world, smap, s0, v0, span, info = tracks(name, car)
    sch = Scheduler(planner, car, table(), smap=smap, grip=GRIP)
    classes, flat = s2_classes(sch, planner)
    cs = sch.corners
    base = S2Lap.build(Style(name="drift_budget"), cs, classes, LossMap.load(), flat)
    d = do.live_driver(car, planner)
    i_s = int(np.searchsorted(d.track.s, s0))
    gp = do.grip_plan(d, i_s, v0)
    print(name, "grip plan", d.length, "m windows, caps", d.rear_slip_max, flush=True)
    rows = []
    for c in cs:
        pos, deg = do.live_positions(d, c, gp, i_s)
        priced = [p for p in pos if "loss_s" in p]
        mp = base.priced[c.s_in]
        r = {"s_in": round(c.s_in, 1), "R": round(c.R, 1), "turn": round(math.degrees(c.turn)),
             "map": None if mp is None else round(mp.loss_s, 4), "floor_deg": deg,
             "live": [None if "loss_s" not in p else round(p["loss_s"], 4) for p in pos],
             "live_from": [p["drift_from"] for p in pos],
             "live_max": max((p["loss_s"] for p in priced), default=None),
             "positions": pos}
        rows.append(r)
        print(name, r["s_in"], "R", r["R"], "turn", r["turn"], "map", r["map"], "live", r["live"],
              r["live_from"], flush=True)
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, f"prices_{name.replace(':', '_')}.json"), "w") as fh:
        json.dump({"course": name, "window_m": d.length, "corners": rows}, fh,
                  default=lambda o: o.item() if hasattr(o, "item") else str(o))
    return rows


def pareto(names):
    """For each course: its corners priced from race_planner/drift_losses.json as S2 prices them
    (multistart.s2_classes, budget.S2Lap), and for each budget (BUDGETS_S seconds, and EPS_PCT %
    of the lap: S1's measured lap where known, else the quasi-steady lap of L0's speed profile on
    the planner's line) what the knapsack buys: the predicted loss (s, % of the lap), the drift
    metres and their share of the course, the corners bought, the corners where the drift gains
    (free), priced, with no drift valley, unpriced. S2_OUT/pareto.json and pareto.md."""
    from race_planner.budget import ZONE_FROM, LossMap, S2Lap
    from race_planner.line import min_curvature_line
    from race_planner.multistart import s2_classes
    from race_planner.qss import speed_profile
    from race_planner.schedule import Scheduler
    from race_planner.style import Style
    lm = LossMap.load()
    car = Car.from_spec(e1.SPEC)
    out, md = [], ["# 065 M2 (E2): what S2's budget buys, predicted from the signed map", ""]
    for name in names:
        planner, world, smap, s0, v0, span, info = tracks(name, car)
        sch = Scheduler(planner, car, table(), smap=smap, grip=GRIP)
        classes, flat = s2_classes(sch, planner)
        cs = sch.corners
        length = float(planner.s[-1] - planner.s[0])
        line, _ = min_curvature_line(planner, 0.5 * car.width, 0.3)
        v = np.maximum(speed_profile(line, car), 1.0)
        t_qss = float(np.sum(np.diff(planner.s) / (0.5 * (v[1:] + v[:-1]))))
        t_ref = S1_LAP.get(name, t_qss)
        zone = {c.s_in: c.s_out - c.s_in + ZONE_FROM for c in cs}
        base = S2Lap.build(Style(name="drift_budget"), cs, classes, lm, flat)
        pr = base.priced
        corners = [{"s_in": round(c.s_in, 1), "R": round(c.R, 1),
                    "turn": round(math.degrees(c.turn)), "classes": classes(c)[0],
                    "width": round(classes(c)[1], 1),
                    "loss_s": None if pr[c.s_in] is None else round(pr[c.s_in].loss_s, 4),
                    "share": None if pr[c.s_in] is None else pr[c.s_in].share} for c in cs]
        md += [f"## {name}", "",
               f"{len(cs)} corners, {length:.0f} m; reference lap {t_ref:.2f} s "
               f"({'S1, measured' if name in S1_LAP else 'quasi-steady, L0'}); priced "
               f"{sum(p is not None for p in pr.values())}, of which the drift gains in "
               f"{sum(p is not None and p.loss_s <= 0 for p in pr.values())}", "",
               "| budget | s | predicted loss s | % of lap | drift m | share of course | "
               "corners bought |", "|---|---|---|---|---|---|---|"]
        rows = []
        budgets = [(f"{b:g} s", b) for b in BUDGETS_S] + [
            (f"{e:g} %", e / 100.0 * t_ref) for e in EPS_PCT]
        for label, b in budgets:
            lap = S2Lap.build(Style(name="drift_budget", eps_lap_s=round(b, 4)), cs, classes, lm,
                              flat)
            loss = sum(max(0.0, p.loss_s) for p in lap.committed.values())
            dm = sum(p.share * zone[k] for k, p in lap.committed.items())
            rows.append({"budget": label, "eps_lap_s": round(b, 4), "loss_s": round(loss, 4),
                         "loss_pct": round(100 * loss / t_ref, 3), "drift_m": round(dm, 1),
                         "share": round(dm / length, 4),
                         "bought": sorted(round(k, 1) for k in lap.committed)})
            md.append(f"| {label} | {b:.3f} | {loss:.3f} | {100 * loss / t_ref:.2f} | {dm:.0f} | "
                      f"{100 * dm / length:.1f} % | {len(lap.committed)} |")
        md += [""]
        out.append({"course": name, "length": length, "t_ref": t_ref, "t_qss": t_qss,
                    "corners": corners, "rows": rows})
    os.makedirs(S2_OUT, exist_ok=True)
    json.dump(out, open(os.path.join(S2_OUT, "pareto.json"), "w"), indent=1)
    open(os.path.join(S2_OUT, "pareto.md"), "w").write("\n".join(md) + "\n")
    print("\n".join(md))


def s2_report(d=None):
    """E3's table (065 M2): every S2 run under d (default S2_OUT/e3) against its course's S1
    run (tag s1): result, time, the loss against S1 and the budget, the plant's drift (metres
    3 deg past the peak, share), mode changes, solves a re-plan, and per bought corner the
    live gate's last measured loss against its allowance (price + 0.3 %). S2_OUT/e3.md."""
    d = d or os.path.join(S2_OUT, "e3")
    rs = [json.load(open(os.path.join(d, f))) for f in sorted(os.listdir(d)) if f.endswith(".json")]
    s1 = {r["course"]: r for r in rs if r["tag"] == "s1"}
    lines = ["# 065 M2 (E3): S2 against S1 in closed loop (sim.Plant, RP_DET)", "",
             "| course | style | budget s | result | time s | vs S1 s | within budget + 0.3 % |"
             " plant drift m (share) | mode changes | solves / re-plan | bought | spent s |",
             "|" + "---|" * 12]
    t = lambda r: r["t_timed"] if r["t_timed"] is not None else r["t_course"]
    for r in sorted(rs, key=lambda r: (r["course"], r["tag"] != "s1", r["tag"])):
        st = r.get("style") or {}
        b = st.get("eps_lap_s", 0.0) if st.get("name") == "drift_budget" else None
        base = s1.get(r["course"])
        dt = (t(r) - t(base)) if base and t(r) is not None and t(base) is not None else None
        ok = None if dt is None or b is None else dt <= b + 0.003 * t(base) + 1e-9
        s2 = r.get("s2") or {}
        lines.append(
            f"| {r['course']} | {r['tag']} | {'' if b is None else b} | {r['result']} | "
            f"{'' if t(r) is None else f'{t(r):.2f}'} | {'' if dt is None else f'{dt:+.2f}'} | "
            f"{'' if ok is None else ('yes' if ok else '**no**')} | "
            f"{r.get('plant_drift_m', 0):.0f} ({100 * r.get('plant_drift_share', 0):.1f} %) | "
            f"{len(r['mode_changes'])} | {r['per_replan']:.2f} | "
            f"{len(s2.get('committed', {})) if s2 else ''} | {s2.get('spent', '') if s2 else ''} |")
    lines += ["", "## Bought corners: the live gate's loss against the allowance", "",
              "| course | style | corner s_in | price s | allow s | live loss s, last | entered |",
              "|---|---|---|---|---|---|---|"]
    for r in rs:
        if not r.get("s2"):
            continue
        last = {}
        for g in r.get("s2_gate", []):
            if g["kind"] == "budget":
                last[g["corner"]] = g
        com = r["s2"]["committed"]
        entered = {round(x["key"], 1) for x in r["s2"]["debits"]}
        for c, g in sorted(last.items()):
            pr = com.get(str(round(c, 1)), com.get(round(c, 1)))
            lv = g.get("loss")
            lines.append(f"| {r['course']} | {r['tag']} | {c:.1f} | {pr} | {g['allow']:.3f} | "
                         f"{'none' if lv is None else f'{lv:+.3f}'} | "
                         f"{'yes' if round(c, 1) in entered else 'no'} |")
    text = "\n".join(lines) + "\n"
    open(os.path.join(S2_OUT, "e3.md"), "w").write(text)
    print(text)


M3_OUT = os.path.join(SWB, "play_log/race_planner/style/m3")


def m3_report(d=None):
    """E3 at M3 (roadmap 065): the runs under d (default M3_OUT/e3), tagged <style>_j<k> (k the
    --jit draw: 0, 1e-9, 2e-9, ...), grouped by course and style over the draws: each draw's
    result and time; the mean and spread (max - min) of the loss against S1's draw of the same
    jit; whether every draw is within the budget + 0.3 % of S1; the plant's drift (metres 3 deg
    past the true peak, and past the planner's), the corners bought, entered (debited) and the
    account spent; mode changes per corner (the most). M3_OUT/e3.md and e3.json."""
    d = d or os.path.join(M3_OUT, "e3")
    rs = [json.load(open(os.path.join(d, f))) for f in sorted(os.listdir(d))
          if f.endswith(".json") and not f.endswith(".traj.json")]
    t = lambda r: r["t_timed"] if r["t_timed"] is not None else r["t_course"]
    style_of = lambda r: r["tag"].rsplit("_j", 1)[0]
    s1 = {(r["course"], r["jit"]): r for r in rs if style_of(r) == "s1"}
    groups = {}
    for r in rs:
        groups.setdefault((r["course"], style_of(r)), []).append(r)
    rows, lines = [], [
        "# 065 M3 (E3): S2 against S1 in closed loop, three draws (sim.Plant, RP_DET, --jit 0, "
        "1e-9, 2e-9)", "",
        "| course | style | budget s | results | time s (mean) | vs S1 s: mean, spread | within "
        "budget + 0.3 %: draws; mean | plant drift m (true peak; planner's) | bought | entered | spent s | "
        "mode changes per corner, most |", "|" + "---|" * 12]
    for (course, st), g in sorted(groups.items(), key=lambda x: (x[0][0], x[0][1] != "s1",
                                                                 x[0][1])):
        g = sorted(g, key=lambda r: r["jit"])
        sty = g[0].get("style") or {}
        b = sty.get("eps_lap_s", 0.0) if sty.get("name") == "drift_budget" else None
        res = [r["result"] for r in g]
        ts = [t(r) for r in g if t(r) is not None]
        dts, oks = [], []
        for r in g:
            base = s1.get((course, r["jit"]))
            if base and t(r) is not None and t(base) is not None:
                dts.append(t(r) - t(base))
                if b is not None:
                    oks.append(dts[-1] <= b + 0.003 * t(base) + 1e-9)
        s2s = [r.get("s2") or {} for r in g]
        dm = [r.get("plant_drift_m", 0.0) for r in g]
        dmg = [r.get("plant_drift_m_planner", 0.0) for r in g]
        mc = []
        for r in g:
            per = {}
            for s0, a, b2 in r["mode_changes"]:
                c = next((c for c in _corner_keys(r) if c - 40.0 <= s0 <= c + 80.0), None)
                per[c] = per.get(c, 0) + 1
            mc.append(max([v for k, v in per.items() if k is not None] or [0]))
        row = {"course": course, "style": st, "budget": b, "results": res, "t": ts,
               "dt": dts, "within": oks, "drift_m": dm, "drift_m_planner": dmg,
               "bought": [len(x.get("committed", {})) for x in s2s],
               "entered": [len(x.get("debits", [])) for x in s2s],
               "spent": [x.get("spent") for x in s2s], "mode_changes_corner": mc,
               "solve_p95": [r["solve_p95"] for r in g], "draws": len(g)}
        rows.append(row)
        f = lambda v: "" if not v else f"{np.mean(v):.2f}"
        lines.append(
            f"| {course} | {st} | {'' if b is None else b} | "
            f"{', '.join(sorted(set(res)))} ({len(g)}) | {f(ts)} | "
            f"{'' if not dts else f'{np.mean(dts):+.3f}, {max(dts) - min(dts):.3f}'} | "
            f"{'' if not oks else ('yes' if all(oks) else f'{sum(oks)} of {len(oks)}')}"
            f"{'' if not oks else ('; mean yes' if np.mean(dts) <= b + 0.003 * np.mean(ts) else '; mean **no**')} | "
            f"{np.mean(dm):.0f} ({min(dm):.0f}-{max(dm):.0f}); {np.mean(dmg):.0f} | "
            f"{'' if b is None else row['bought'][0]} | "
            f"{'' if b is None else '/'.join(map(str, row['entered']))} | "
            f"{'' if b is None else '/'.join(str(round(x or 0, 3)) for x in row['spent'])} | "
            f"{max(mc)} |")
    text = "\n".join(lines) + "\n"
    os.makedirs(M3_OUT, exist_ok=True)
    open(os.path.join(M3_OUT, "e3.md"), "w").write(text)
    json.dump(rows, open(os.path.join(M3_OUT, "e3.json"), "w"), indent=1)
    print(text)
    return rows


def _corner_keys(r):
    return sorted(float(k) for k in (r.get("plant_drift_corners") or {}))


def corner_courses(widths=(6.0, 15.0)):
    out = []
    for cls in CLASSES:
        for R in (9, 15, 25):
            for turn in (90, 180):
                for w in widths:
                    out.append(f"c:{cls}:{R}:{turn}:{w:g}")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("what")
    ap.add_argument("course", nargs="?")
    ap.add_argument("--arm", default="single")
    ap.add_argument("--err", type=int, default=0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--perc", action="store_true")
    ap.add_argument("--est", action="store_true")
    ap.add_argument("--rep", type=int, default=1)
    ap.add_argument("--cpus", default="")
    ap.add_argument("--rules", default="")
    ap.add_argument("--sd", type=float, default=None)
    ap.add_argument("--tag", default="")
    ap.add_argument("--beta", type=float, default=None, help="L1's sideslip bound, deg")
    ap.add_argument("--front", type=float, default=None,
                    help="L1's front slip bound, times the surface's peak (live.Driver's "
                         "front_slip_max; default the Driver's, 0 free)")
    ap.add_argument("--rear", type=float, default=None,
                    help="L1's rear slip bound, as --front (rear_slip_max)")
    ap.add_argument("--rear-low", type=float, default=None,
                    help="L1's rear slip bound on slippery fall-off classes (rear_slip_low)")
    ap.add_argument("--style", default=None,
                    help="the driving style record (roadmap 065): a YAML path or JSON")
    ap.add_argument("--digest", action="store_true",
                    help="hash every plan and plant state (bit-for-bit comparisons)")
    ap.add_argument("--traj", action="store_true",
                    help="also write the run tick by tick (<key>.traj.json: the car, each "
                         "wheel's slip against its peak, every adopted plan, the road)")
    ap.add_argument("--prices", default=None,
                    help="S2: the course's own prices (course-prices' json; roadmap 065 M3)")
    ap.add_argument("--jit", type=float, default=0.0,
                    help="the start speed times (1 + jit): another draw of a chaotic run")
    ap.add_argument("--ecu", default=None,
                    help="the ECU's calibration over the spec's, JSON by field name (roadmap 065 "
                         "E4, e.g. '{\"esc_enabled\": true}'); implies --ecu-obs")
    ap.add_argument("--ecu-obs", action="store_true",
                    help="count the ECU's interventions against the planned slides (065 E4)")
    ap.add_argument("--force-drift", action="store_true",
                    help="E4's 'D forced': S1's gate adopts any feasible drift plan (065 E4)")
    ap.add_argument("--b-iter", type=int, default=None,
                    help="S2's B on its own IPOPT iteration cap (065 R6)")
    ap.add_argument("--b-budget", type=float, default=None,
                    help="S2's B on its own wall budget, s (065 R6)")
    a = ap.parse_args()
    style = None
    if a.style is not None:
        from race_planner.style import Style
        style = (Style.from_dict(json.loads(a.style)) if a.style.lstrip().startswith("{")
                 else Style.from_yaml(a.style))
    cpus = [int(x) for x in a.cpus.split(",") if x] or None
    if cpus:
        os.sched_setaffinity(0, {cpus[0]})
    if a.what == "oracle":
        return oracle(a.course)
    if a.what == "report":
        return report()
    if a.what == "aggregate":
        return aggregate()
    if a.what == "s2-report":
        return s2_report()
    if a.what == "m3-report":
        return m3_report(a.course)
    if a.what == "pareto":
        return pareto(a.course.split(","))
    if a.what == "course-prices":              # roadmap 065 M3: the live gate's prices
        return [course_prices(c) for c in a.course.split(",")]
    if a.what == "list":
        print("\n".join(corner_courses()))
        return
    return run(a.course, a.arm, a.err, a.seed, a.perc, a.est, a.rep, cpus[1:] if cpus else None,
               json.loads(a.rules) if a.rules else None, a.sd, a.tag, a.beta, a.front, a.rear, a.rear_low,
               style, a.digest, a.jit, a.traj, a.prices,
               json.loads(a.ecu) if a.ecu else None, a.ecu_obs, a.force_drift,
               a.b_iter, a.b_budget)


if __name__ == "__main__":
    main()
