"""L1 from more than one start: grip or drift chosen live (roadmap 061 M3; the method in
docs/mode-choice-methods.md, its numbers in mode-choice-m1.md and mode-choice-m3.md).

Each re-plan solves the window from a start in each valley, each in a process of its own with
its own compiled Window and its own budget (IPOPT's max_wall_time), all at once:

- G, the grip plan: the adopted plan shifted while it is a grip plan (live.Driver's one start),
  otherwise G's own last plan shifted, so the grip valley stays seeded while a drift is driven
  (the adopted plan again if G's own last solve failed: warm, if in the other valley);
- D, rear out: D's own last plan shifted while it still slides in the corner ahead (a seed
  converges over re-plans, continuation in time), otherwise a fresh seed for it from the corner
  library (modes.Library: Dc) or, where no archetype is near, the synthetic one (Ds); idle when
  no corner is ahead in the window;
- F, a flick (optional): as D, but only into a tight hairpin (modes.HAIRPIN_R, HAIRPIN_TURN).

Every start carries its plan's multipliers when it is a plan shifted (window.warm shifts them),
none when it is a seed. A plan is grip or drift by what it does in the corner the choice is
about, the first whose turn-in is SEED_LEAD or more ahead (modes.REAR_PAST past the rear tyre's
peak from CORNER_FROM before its turn-in to its exit), not by the start it came from: a drift seed
that falls back into the grip valley is grip. (Classified over the whole window instead, beyond
its first EXCLUDE metres, both modes almost never had a plan at once in E1: a window holding two
or three corners on low friction slid in one of them whatever its start, so there was nothing
to choose; docs/mode-choice-m3.md.) With no corner ahead there is no choice to make: every plan
keeps the current mode (the car is in the corner it chose for, or past it), and the fastest is
adopted.

The fastest feasible plan of each mode competes; the mode changes only when the other's is
faster by more than `threshold` of the current one's window time on `confirm` re-plans in a row,
and a drift only when its slide starts at least `ahead` metres into the window (its trail
braking must not be cut short). The car starts in grip, so a tie stays grip. The adoption rules
are live.Driver's: a fixed latency (the budget) and adopted once the car reaches its first node.
"""
import multiprocessing as mp
import os
import time

import numpy as np

from race_planner import modes
from race_planner.track import TYRE_FIELDS

THRESHOLD = 0.003     # of the window's time: M1's noise floor (the live p75 on mu 0.45)
CONFIRM = 2           # re-plans in a row
AHEAD = 25.0          # m: a drift's slide starts at least this far into the window
EXCLUDE = 12.0        # m: a plan's first metres say nothing of its mode (the car's own slide)
SEED_SHARE = 0.9      # of the re-plan's budget, each seed's: the answers and the choice fit in it
SEED_LEAD = 10.0      # m: a corner is seeded only while its turn-in is this far ahead or more
CORNER_FROM = 30.0    # m before the turn-in a corner's slide may start (the seeds' D_FROM is 25)


def _worker(conn, car, track, kw, cpus):
    """A seed's process: one Window, solving what it is sent until it is sent None."""
    if cpus:
        os.sched_setaffinity(0, cpus)
    from race_planner.live import warm_from
    from race_planner.window import Window
    w = Window(car, **kw)
    conn.send("ready")
    while True:
        msg = conn.recv()
        if msg is None:
            break
        i0, x0, v_end, mu, start = msg
        if "plan" in start:                  # a plan shifted, with its multipliers
            w.lam, w.lam_x = start["plan"].get("lam"), start["plan"].get("lam_x")
            X, U = warm_from(w, start["plan"], i0, x0, track)
        else:                                # a seed: none of its own
            w.lam = w.lam_x = None
            X, U = start["X"], start["U"]
        r = w.solve(track, i0, x0, v_end, X, U, mu_scale=mu)
        r["lam"], r["lam_x"] = w.lam, w.lam_x
        conn.send(r)


class MultiStart:
    """The seed processes and the chooser for one live.Driver (built by it from its
    `multistart` options: threshold, confirm, ahead, flick, seed_budget, max_iter, cpus,
    library)."""

    def __init__(self, driver, threshold=THRESHOLD, confirm=CONFIRM, ahead=AHEAD, flick=False,
                 seed_budget=None, max_iter=400, cpus=None, library=True):
        d = driver
        self.car, self.track, self.grip = d.car, d.track, d.grip
        self.threshold, self.confirm, self.ahead = threshold, confirm, ahead
        self.slots = ("G", "D", "F") if flick else ("G", "D")
        kw = {"ds": d.ds, "length": d.length, "grip": d.grip, "max_iter": max_iter,
              "budget": SEED_SHARE * d.budget if seed_budget is None else seed_budget}
        ctx = mp.get_context("spawn")        # a fresh interpreter: no solver state forked
        omp = os.environ.get("OMP_NUM_THREADS")
        os.environ["OMP_NUM_THREADS"] = "1"  # one core a seed
        self.conns, self.procs = {}, []
        try:
            for k, slot in enumerate(self.slots):
                a, b = ctx.Pipe()
                cpu = None if not cpus else {cpus[k % len(cpus)]}
                p = ctx.Process(target=_worker, args=(b, d.car, d.track, kw, cpu), daemon=True)
                p.start()
                self.conns[slot], self.procs = a, self.procs + [p]
        finally:
            if omp is None:
                os.environ.pop("OMP_NUM_THREADS", None)
            else:
                os.environ["OMP_NUM_THREADS"] = omp
        for a in self.conns.values():
            assert a.recv() == "ready"
        self.corners = modes.corners(d.track, d.grip)
        self.library = (modes.Library(library if isinstance(library, str) else modes.LIBRARY)
                        if library else None)
        self.last = {s: None for s in self.slots}     # each seed's own last plan
        self.mode, self.count = "grip", 0
        self.records = []

    def close(self):
        """The seed processes told to stop, and killed if they do not."""
        for a in self.conns.values():
            try:
                a.send(None)
            except (BrokenPipeError, OSError):
                pass
        for p in self.procs:
            p.join(timeout=5)
            if p.is_alive():
                p.kill()
        self.conns, self.procs = {}, []

    def classify(self, plan, i0, mu, corner=None):
        """The plan's mode (and where its slide starts, m into the window) by its rear axle past
        the tyre's peak at the planner's friction there: in `corner` (from CORNER_FROM before its
        turn-in to its exit), or anywhere beyond the window's first EXCLUDE metres (the first
        plan's, and the E1 study's adopted plans')."""
        N = plan["X"].shape[1]
        sl = slice(i0, i0 + N)
        mu_eff = self.grip * self.track.mu[sl] * (1.0 if mu is None else np.asarray(mu)[:N])
        past = modes.past_peak(self.car, plan["X"], mu_eff, self.track.sliding[sl],
                               np.vstack([getattr(self.track, k)[sl] for k in TYRE_FIELDS]))
        s = plan["s"] - plan["s"][0]
        hit = (past >= modes.REAR_PAST) & (s >= EXCLUDE)
        if corner is not None:
            hit &= (plan["s"] >= corner.s_in - CORNER_FROM) & (plan["s"] <= corner.s_out)
        plan["past"] = past
        plan["mode"] = "drift" if hit.any() else "grip"
        plan["onset"] = float(s[np.argmax(hit)]) if hit.any() else None
        return plan["mode"]

    def _slides_in(self, plan, c):
        """Whether the plan slides in corner c (its own slide carried on into it)."""
        return plan is not None and c is not None and bool(np.any(
            (plan["past"] >= modes.REAR_PAST) & (plan["s"] >= c.s_in - CORNER_FROM)
            & (plan["s"] <= c.s_out)))

    def corner_ahead(self, s0, s_end):
        """The first corner whose turn-in is SEED_LEAD or more ahead of s0 and 40 m or more
        inside the window (ending at s_end): the one a re-plan's choice is about."""
        for c in self.corners:
            if c.s_in - s0 >= SEED_LEAD and c.s_in <= s_end - 40.0:
                return c
        return None

    def _seed(self, slot, base, sec, c):
        """A fresh start for D or F into corner c (None: none to seed; F only into a hairpin)."""
        if c is None or (slot == "F" and not c.hairpin):
            return None, None
        X, U = base
        if slot == "F":
            peak = modes.rear_peak(self.car, c.mu, c.sliding, c.surface)
            return "F", modes.seed_flick(X, U, sec, c.s_in, c.s_out, self.car, peak, c.mu, c.sign)
        i = self.library.nearest(c) if self.library is not None else None
        if i is not None:
            return "Dc", self.library.seed(i, X, U, sec, c, self.car)
        peak = modes.rear_peak(self.car, c.mu, c.sliding, c.surface)
        return "Ds", modes.seed_rear_out(X, U, sec, c.s_in, c.s_out, self.car, peak, c.mu, c.sign)

    def replan(self, d, i0, x0, v_end, mu):
        """Every seed's solve of the window from node i0 at state x0; returns the plan chosen
        (or a failed result when none is feasible) and the wall time from sending the starts to
        the last answer."""
        from race_planner.live import warm_from
        sec = d.warm.section(self.track, i0)
        s0 = float(sec.s[0])
        adopted = d.plan
        g_from = adopted if adopted.get("mode", "grip") == "grip" else self.last["G"]
        if g_from is None:                   # G's own failed: the adopted plan is still warm
            g_from = adopted
        d.warm.lam = d.warm.lam_x = None     # the seeds' base: no multipliers to shift here
        base = warm_from(d.warm, g_from, i0, x0, self.track)
        starts = {"G": {"plan": g_from}}
        kinds = {"G": "adopted" if g_from is adopted else "own"}
        corner = self.corner_ahead(s0, float(sec.s[-1]))     # the one the choice is about
        for slot in self.slots[1:]:
            own = self.last[slot]
            if self._slides_in(own, corner):
                starts[slot], kinds[slot] = {"plan": own}, "own"
                continue
            kind, seed = self._seed(slot, base, sec, corner)
            if seed is not None:
                starts[slot], kinds[slot] = {"X": seed[0], "U": seed[1]}, kind
        c = time.time()
        for slot, st in starts.items():
            self.conns[slot].send((i0, x0, v_end, mu, st))
        res = {slot: self.conns[slot].recv() for slot in starts}
        wall = time.time() - c
        for slot in self.slots:
            r = res.get(slot)
            self.last[slot] = r if r is not None and r["ok"] else None
            if r is not None and r["ok"]:
                self.classify(r, i0, mu, corner)
                if corner is None:           # nothing ahead to choose for: the car is in the
                    r["mode"], r["onset"] = self.mode, None   # corner it chose, or past it
        rec = {"s0": s0, "i0": i0, "wall": wall,
               "corner": None if corner is None else corner.s_in,
               "seeds": {slot: {"start": kinds[slot], "ok": bool(r["ok"]), "iters": int(r["iters"]),
                                "solve_s": float(r["solve_s"]), "status": r["status"],
                                "time": float(r["time"]), "mode": r.get("mode"),
                                "onset": r.get("onset")}
                         for slot, r in res.items()}}
        slot = self.choose(res, rec)
        self.records.append(rec)
        if slot is None:
            worst = max(res.values(), key=lambda r: r["iters"])
            return dict(worst, ok=False, status="no seed feasible"), wall
        return dict(res[slot], slot=slot), wall

    def choose(self, res, rec=None):
        """The seed whose plan is adopted, from each seed's result (ok, time, mode, onset): the
        fastest feasible plan of the mode chosen, the mode changed by the hysteresis gate (module
        docstring); None if no plan is feasible. rec, if given, is filled in."""
        rec = {} if rec is None else rec
        rec["mode_before"] = self.mode
        best = {}
        for slot, r in res.items():
            if r["ok"] and (r["mode"] not in best or r["time"] < res[best[r["mode"]]]["time"]):
                best[r["mode"]] = slot
        if not best:
            self.count = 0
            rec.update(chosen=None, mode=self.mode)
            return None
        cur = self.mode
        other = "drift" if cur == "grip" else "grip"
        if cur not in best:                  # the slide is over, or the mode has no plan
            self.mode, self.count = other, 0
        elif other in best:
            t_cur, t_other = res[best[cur]]["time"], res[best[other]]["time"]
            early = other == "grip" or res[best[other]]["onset"] >= self.ahead
            rec["gain"] = (t_cur - t_other) / t_cur
            if t_cur - t_other > self.threshold * t_cur and early:
                self.count += 1
                if self.count >= self.confirm:
                    self.mode, self.count = other, 0
            else:
                self.count = 0
        else:
            self.count = 0
        rec.update(chosen=best[self.mode], mode=self.mode, count=self.count)
        return best[self.mode]
