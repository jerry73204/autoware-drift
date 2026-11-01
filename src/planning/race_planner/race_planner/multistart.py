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
import math
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


def past_of(car, track, grip, plan, mu):
    """The plan's rear axle past its tyre's peak at each node (rad; modes.past_peak), at the
    planner's friction there (mu: the window's scale, None for 1)."""
    N = plan["X"].shape[1]
    sl = slice(plan["i0"], plan["i0"] + N)
    mu_eff = grip * track.mu[sl] * (1.0 if mu is None else np.asarray(mu)[:N])
    return modes.past_peak(car, plan["X"], mu_eff, track.sliding[sl],
                           np.vstack([getattr(track, k)[sl] for k in TYRE_FIELDS]))


def slides_in(plan, c, past=None):
    """Whether the plan slides in corner c (its own slide carried on into it): its rear axle
    modes.REAR_PAST past the peak from CORNER_FROM before c's turn-in to c's exit. past: the
    plan's (default plan["past"], set by MultiStart.classify)."""
    if plan is None or c is None:
        return False
    past = plan["past"] if past is None else past
    return bool(np.any((past >= modes.REAR_PAST) & (plan["s"] >= c.s_in - CORNER_FROM)
                       & (plan["s"] <= c.s_out)))


def drift_metres(plan, c, past_deg):
    """S2's drift measure D_k: the metres of corner c's zone (CORNER_FROM before its turn-in to
    its exit) over which the plan's rear axle runs past_deg or more past its tyre's peak
    (style.drift_past_deg; plan["past"] from MultiStart.classify, rad); 0 with no corner."""
    if c is None:
        return 0.0
    import math
    s = np.asarray(plan["s"], float)
    on = (plan["past"] >= math.radians(past_deg)) & (s >= c.s_in - CORNER_FROM) & (s <= c.s_out)
    ds = np.diff(s, append=s[-1] + (s[-1] - s[-2] if len(s) > 1 else 0.0))
    return float(np.sum(ds[on]))


def s2_classes(sch, track):
    """S2's view of a corner through a schedule.Scheduler: (classes(c) -> ({class: its largest
    probability over the approach and the corner, those at least p_add}, the least road width
    there), flat(class) -> no fall-off past the peak), as the scheduler reads them."""
    a = sch.rules["approach"]

    def classes(c):
        m = (track.s >= c.s_in - a) & (track.s <= c.s_out)
        pc = sch._probs(track.s[m])
        probs = {k: float(np.max(v)) for k, v in pc.items()
                 if float(np.max(v)) >= sch.rules["p_add"]}
        return probs, float(np.min(track.w_left[m] + track.w_right[m]))
    return classes, (lambda k: sch.table.sliding(k) >= sch.rules["flat"])


def _worker(conn, car, track, kw, cpus, held=None, floors=None):
    """A seed's process: one Window (or, for H, one per bound in `held`, degrees: the rear
    axle's slip angle held within it; for S2's B, one per floor in `floors`, degrees: the rear
    held past it on a stretch, forced.py), solving what it is sent until it is sent None."""
    if cpus:
        os.sched_setaffinity(0, cpus)
    import math
    from race_planner.live import warm_from
    from race_planner.window import Window
    if held:
        ws = {b: Window(car, rear_alpha_max=math.radians(b), **kw) for b in held}
    elif floors:
        from race_planner import forced
        ws = {b: Window(car, **forced.floor_kwargs(math.radians(b)), **kw) for b in floors}
    else:
        w = Window(car, **kw)
    conn.send("ready")
    while True:
        msg = conn.recv()
        if msg is None:
            break
        i0, x0, v_end, mu, start = msg
        if held:
            w = ws[start["bound"]]
        elif floors:                         # (degrees, from, to, sign): the corner's floor
            deg, a, b, sign = start["floor"]
            w = ws[deg]
            forced.set_floor(w, w.section(track, i0).s, a, b, sign)
        if "plan" in start:                  # a plan shifted, with its multipliers
            w.lam, w.lam_x = _fitting(w, start["plan"].get("lam"), start["plan"].get("lam_x"))
            X, U = warm_from(w, start["plan"], i0, x0, track)
        else:                                # a seed: none of its own
            w.lam = w.lam_x = None
            X, U = start["X"], start["U"]
        r = w.solve(track, i0, x0, v_end, X, U, mu_scale=mu)
        r["lam"], r["lam_x"] = w.lam, w.lam_x
        conn.send(r)


def _fitting(w, lam, lam_x):
    """A plan's multipliers if they are window w's shape (a held window has more rows), else
    none."""
    from race_planner.window import NZ
    if (lam is None or lam_x is None or len(lam) != len(w.lbg_all)
            or len(lam_x) != NZ * w.M + 6):
        return None, None
    return lam, lam_x


HELD_MARGIN = 1.0     # deg: H's bound over the rear tyre's peak (the study's REAR_MARGIN)


def held_bound(peak):
    """H's bound (whole degrees) for a rear tyre peaking at `peak` (rad): the degree below
    the peak plus HELD_MARGIN, so a few compiled windows serve every surface."""
    import math
    return int(math.floor(math.degrees(peak))) + int(HELD_MARGIN)


class MultiStart:
    """The seed processes and the chooser for one live.Driver (built by it from its
    `multistart` options: threshold, confirm, ahead, flick, seed_budget, max_iter, cpus,
    library)."""

    def __init__(self, driver, threshold=THRESHOLD, confirm=CONFIRM, ahead=AHEAD, flick=False,
                 seed_budget=None, max_iter=400, cpus=None, library=True, schedule=None,
                 perception=None, h_competes=False, style=None, loss_map=None, lap=None,
                 course_prices=None, b_max_iter=None, b_budget=None):
        """schedule (roadmap 061 M3b): None, M3's fixed starts ({G, D}, F with `flick`, G in a
        process of its own); else the starts chosen per re-plan by a schedule.Scheduler built
        from this dict (its keyword arguments: table, a track.SurfaceTable, default swb's;
        dmap, smap, always, rules), G solved in the driver's own process as 060 solves it, H, D
        and F each in a process. perception: a surface.Perception that may veto a drift commit
        and lower the planned friction near the car (scheduled only). h_competes: H's plan
        weighed by the gate like the others; by default (False) it is the fallback, adopted only
        when no free start (G, D, F) has a feasible plan (M3b's chain check: as a competitor it
        held the grip mode on a slower plan wherever the free G slid by itself, Tsukuba +0.55 %).
        style (roadmap 065): a style.Style (or what style.as_style takes); None, the driver's
        (live.Driver.style), else fastest. S1 is this class unchanged; S2 (drift_budget, M2)
        needs the schedule (refused without it: ValueError naming 'name'); S3 is refused
        (NotImplementedError). loss_map: S2's budget.LossMap or its path (default
        budget.LOSS_MAP); lap: S2's budget per lap of this length (m) on a circuit driven as
        laps (serve.py's), None per run (design 065 call 3). course_prices: S2's budget.
        CoursePrices or its path (roadmap 065 M3), the course's own corners priced offline at the
        live settings; None, the map's only. b_max_iter, b_budget: S2's B (the forced drift)
        on its own IPOPT iteration cap and wall budget (s), roadmap 065 R6 (M4: a cap of 110 met
        R6 on Haruna by failing B twice as often, so not the default); None, the other starts'
        (max_iter, the seed budget). S1 never builds B, so neither touches it."""
        d = driver
        from race_planner.style import as_style
        self.style = as_style(getattr(d, "style", None) if style is None else style)
        self.style.require_implemented()
        if self.style.name == "drift_budget" and schedule is None:
            raise ValueError("'name': drift_budget needs the scheduled starts (multistart's "
                             "schedule): its drifts are bought from the drift map")
        self.loss_map, self.lap, self.s2 = loss_map, lap, None
        self.course_prices = course_prices
        self.car, self.track, self.grip = d.car, d.track, d.grip
        self.threshold, self.confirm, self.ahead = threshold, confirm, ahead
        self.scheduler, self.perception = None, perception
        held = None
        if schedule is not None:
            from race_planner.schedule import Scheduler
            opts = dict(schedule) if isinstance(schedule, dict) else {}
            if opts.get("table") is None:
                from race_planner.track import SurfaceTable
                swb = os.environ.get("SWB", os.path.expanduser("~/repos/carla-scenario-bridge"))
                opts["table"] = SurfaceTable.from_yaml(
                    os.path.join(swb, "src/swb_isaac_world/config/surface_classes.yaml"))
            self.scheduler = Scheduler(d.track, d.car, grip=d.grip, **opts)
            self.slots = ("G", "H", "D", "F")
            self.peaks = self._node_peaks()
            held = sorted({held_bound(p) for p in self.peaks})
        else:
            self.slots = ("G", "D", "F") if flick else ("G", "D")
        self.held, self.h_competes = held, h_competes
        floors = None
        if (self.style.name == "drift_budget" and self.scheduler is not None
                and getattr(d, "beta_max", None) is None):
            # S2's B: a bought corner's drift forced (forced.py), one window a floor angle over
            # the track's corners (built at the start: a style switched to S2 later runs without)
            floors = sorted({self._floor(c)[0] for c in modes.corners(d.track, d.grip)})
            if floors:
                self.slots = self.slots + ("B",)
        self.floors, self.b_max_iter = floors, b_max_iter
        kw = {"ds": d.ds, "length": d.length, "grip": d.grip, "max_iter": max_iter,
              "beta_max": getattr(d, "beta_max", None),
              "front_slip_max": getattr(d, "front_slip_max", None),
              "rear_slip_max": getattr(d, "rear_slip_max", None),
              "rear_slip_low": getattr(d, "rear_slip_low", None),
              "free_stages": getattr(d, "free_stages", 0),
              "budget": SEED_SHARE * d.budget if seed_budget is None else seed_budget}
        ctx = mp.get_context("spawn")        # a fresh interpreter: no solver state forked
        omp = os.environ.get("OMP_NUM_THREADS")
        os.environ["OMP_NUM_THREADS"] = "1"  # one core a seed
        self.conns, self.procs = {}, []
        try:
            for k, slot in enumerate(self.slots):
                if slot == "G" and self.scheduler is not None:
                    continue                 # G is solved in the driver's process
                a, b = ctx.Pipe()
                cpu = None if not cpus else {cpus[k % len(cpus)]}
                args = (b, d.car, d.track, kw, cpu, held if slot == "H" else None)
                if slot == "B":
                    bkw = dict(kw)
                    if b_max_iter is not None:
                        bkw["max_iter"] = min(int(b_max_iter), max_iter)
                    if b_budget is not None:
                        bkw["budget"] = float(b_budget)
                    args = (b, d.car, d.track, bkw, cpu, None, floors)
                p = ctx.Process(target=_worker, args=args, daemon=True)
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
        return slides_in(plan, c)

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
        if self.scheduler is not None:
            return self._replan_scheduled(d, i0, x0, v_end, mu)
        from race_planner.live import warm_from
        sec = d.warm.section(self.track, i0)
        s0 = float(sec.s[0])
        adopted = d.plan
        g_from = adopted if adopted.get("mode", "grip") == "grip" else self.last["G"]
        if g_from is None:                   # G's own failed: the adopted plan is still warm
            g_from = adopted
        if d.cut is not None:                # G's last solve cut near feasible (issue 005)
            g_from = d.cut
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
        from race_planner.live import cut_iterate
        d.cut = cut_iterate(res["G"]) if "G" in res and d.cut_warm else None
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
        docstring); None if no plan is feasible. rec, if given, is filled in. Under S2
        (drift_budget, roadmap 065) _choose_s2, which is this gate wherever S2 has nothing to
        buy."""
        st = getattr(self, "style", None)
        if st is not None and st.name == "drift_budget":
            return self._choose_s2(res, rec)
        return self._choose_s1(res, rec)

    def _choose_s1(self, res, rec=None):
        """S1's gate (choose's docstring): today's, unchanged."""
        rec = {} if rec is None else rec
        rec["mode_before"] = self.mode
        if not getattr(self, "h_competes", True) and "H" in res and any(
                r["ok"] for k, r in res.items() if k != "H"):
            res = {k: r for k, r in res.items() if k != "H"}     # H: the fallback only
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
            if other == "drift" and t_cur - t_other > self.threshold * t_cur and self._veto():
                rec["veto"] = True               # the camera reads a surface where none pays
                early = False
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

    # ---- S2, drift_budget (roadmap 065 M2; design 065 4.3, call 3) ----

    def _choose_s2(self, res, rec=None):
        """S2's gate for the corner the choice is about (self._corner), by its kind in the lap's
        plan (self.s2, budget.S2Lap):

        - 'budget' (bought offline by the knapsack): the fastest drift plan is entered when it
          is no slower than the fastest grip plan by more than the corner's price (its predicted
          loss, at least 0) plus `threshold` (the 0.3 % noise) on `confirm` re-plans, and left
          beyond that plus exit_hyst on `confirm` re-plans (or at once with no drift plan);
        - 'lex' (eps_window, classes without fall-off): among the plans within (1 + eps) of the
          fastest, the one that drifts most (drift_m, the corner's metres drift_past_deg past the
          peak; the faster on a tie), entered on `confirm` re-plans while the account can pay
          its price (the map's, else the window's measured loss; only with eps_lap_s set), left
          when no drift plan is within (1 + eps + exit_hyst) on `confirm` re-plans;
        - None (no corner, or one S2 does not buy): S1's gate (_choose_s1).

        A drift's slide must start `ahead` metres into the window, as S1's. The account is
        debited once a corner, when its drift is entered."""
        rec = {} if rec is None else rec
        lap, c = self.s2, getattr(self, "_corner", None)
        kind = lap.kind(c)
        if kind is None:
            out = self._choose_s1(res, rec)
            rec["s2"] = None
            return out
        rec["mode_before"] = self.mode
        if not getattr(self, "h_competes", True) and "H" in res and any(
                r["ok"] for k, r in res.items() if k != "H"):
            res = {k: r for k, r in res.items() if k != "H"}     # H: the fallback only
        ok = {k: r for k, r in res.items() if r["ok"]}
        if not ok:
            self.count = 0
            rec.update(chosen=None, mode=self.mode)
            return None
        fast = lambda ks: min(ks, key=lambda k: ok[k]["time"])
        most = lambda ks: max(ks, key=lambda k: (ok[k].get("drift_m", 0.0), -ok[k]["time"]))
        grip = [k for k in ok if ok[k]["mode"] == "grip"]
        drift = [k for k in ok if ok[k]["mode"] == "drift"]
        t_min = min(r["time"] for r in ok.values())
        g = fast(grip) if grip else None
        ref = ok[g]["time"] if g is not None else t_min
        price = lap.price(c)
        info = {"kind": kind, "corner": c.s_in, "t_grip": None if g is None else ref}
        if kind == "budget":
            d = fast(drift) if drift else None
            allow = max(0.0, price.loss_s) + self.threshold * ref
            loss = None if d is None else ok[d]["time"] - ref
            enter = d is not None and loss <= allow
            stay = d is not None and loss <= allow + lap.h * ref
            predicted = price.loss_s
            info.update(allow=allow, loss=loss)
        else:
            within = [k for k in drift if ok[k]["time"] <= (1 + lap.eps) * t_min]
            wider = [k for k in drift if ok[k]["time"] <= (1 + lap.eps + lap.h) * t_min]
            d = most(within) if within else most(wider) if wider else (
                fast(drift) if drift else None)
            loss = None if d is None else ok[d]["time"] - ref
            predicted = price.loss_s if price is not None else (loss or 0.0)
            pays = lap.style.eps_lap_s <= 0 or max(0.0, predicted) <= lap.account.free() + 1e-12
            enter = bool(within) and pays
            stay = bool(wider)
            info.update(eps=lap.eps, loss=loss, predicted=predicted, pays=pays)
        if self.mode == "grip":
            if g is None:                    # no grip plan: the drift (S1 changes mode too)
                self.mode, self.count = "drift", 0
                lap.account.debit(c.s_in, predicted, kind + ", no grip plan")
                self._s2_drift = c.s_in
            elif enter and ok[d]["onset"] is not None and ok[d]["onset"] >= self.ahead:
                self.count += 1
                if self.count >= self.confirm:
                    self.mode, self.count = "drift", 0
                    lap.account.debit(c.s_in, predicted, kind)
                    self._s2_drift = c.s_in
            else:
                self.count = 0
        else:
            if d is None:                    # the slide is over, or no drift plan
                self.mode, self.count = "grip", 0
            elif getattr(self, "_s2_drift", None) != c.s_in:
                # a drift carried in from another corner (issue 010): this corner's entry rule
                # (its allowance, or no grip plan), then its debit; else left as by the exit rule
                info["carried"] = True
                if g is None or enter:
                    self.count = 0
                    lap.account.debit(c.s_in, predicted,
                                      kind + (", no grip plan" if g is None else "") + ", carried")
                    self._s2_drift = c.s_in
                else:
                    self.count += 1
                    if self.count >= self.confirm:
                        self.mode, self.count = "grip", 0
            elif stay:
                self.count = 0
            else:
                self.count += 1
                if self.count >= self.confirm and g is not None:
                    self.mode, self.count = "grip", 0
        chosen = g if self.mode == "grip" else d
        info["account"] = round(lap.account.remaining, 4)
        rec.update(chosen=chosen, mode=self.mode, count=self.count, s2=info)
        return chosen

    # ---- the scheduled starts (roadmap 061 M3b) ----

    def _node_peaks(self):
        """The rear tyre's peak (rad) at every node of the track, at the planner's friction."""
        from race_planner import modes
        t = self.track
        return np.array([modes.rear_peak(self.car, self.grip * m, sl, tuple(sf))
                         for m, sl, sf in zip(t.mu, t.sliding,
                                              np.vstack([getattr(t, k) for k in TYRE_FIELDS]).T)])

    def _bound(self, i0, n):
        """H's bound for the window from node i0 (n nodes): its least peak's (held_bound)."""
        return held_bound(float(np.min(self.peaks[i0:i0 + n])))

    def _veto(self):
        """Whether the camera (perception) vetoes a drift into the corner the choice is about:
        it reads the corner (from CORNER_FROM before its turn-in to its exit) and no class it
        reads there is one the scheduler would start a drift for. Nothing read: no veto."""
        c, p = getattr(self, "_corner", None), getattr(self, "perception", None)
        if p is None or c is None or getattr(self, "scheduler", None) is None:
            return False
        m = (self.track.s >= c.s_in - CORNER_FROM) & (self.track.s <= c.s_out)
        reads = {x for x in p.read(self._s0, self.track.s[m]) if x is not None}
        if not reads:
            return False
        sch = self.scheduler
        width = float(np.min(self.track.w_left[m] + self.track.w_right[m]))
        return not any(sch.dmap.pays(x, c.R, c.turn, width,
                                     falloff=sch.table.sliding(x) < sch.rules["flat"])
                       for x in reads)

    def _held_seed(self, base, bound):
        """H's start where it has no plan of its own: the grip base with the rear axle's slip
        clipped into the bound (the first node is the car's and stays)."""
        import math
        X, U = (np.array(v, float) for v in base)
        tb = math.tan(math.radians(bound) - math.radians(0.2))
        c = self.car.lr * X[2, 1:]
        X[1, 1:] = np.clip(X[1, 1:], c - tb * X[0, 1:], c + tb * X[0, 1:])
        return X, U

    def _replan_scheduled(self, d, i0, x0, v_end, mu):
        """A re-plan with the starts the scheduler picks: G in this process (060's warm start
        from the adopted plan, its multipliers as 060 keeps them, while that plan is G's
        and grip), H, D and F in theirs when scheduled."""
        from race_planner.live import warm_from
        sec = d.warm.section(self.track, i0)
        s0, s1 = float(sec.s[0]), float(sec.s[-1])
        n = len(sec.s)
        if self.perception is not None:      # lowers the friction near the car, never raises
            m = np.broadcast_to(1.0 if mu is None else np.asarray(mu, float), (n,))
            planned = self.track.mu[i0:i0 + n] * m
            mu = m * self.perception.scale(s0, sec.s, planned)
        corner = self.corner_ahead(s0, s1)
        self._corner, self._s0 = corner, s0
        running = [k for k in ("D", "F") if self._slides_in(self.last[k], corner)]
        scn = self.scheduler.scan(s0, s1)
        want, why = self.scheduler.starts(scn, corner, running)
        s2 = self.style.name == "drift_budget"
        if s2:                               # S2: a D (and H) into a corner it may buy
            lap = self._s2_lap()
            kind = lap.kind(corner)
            if kind and "D" not in want and corner.s_in - s0 >= self.scheduler.rules["ahead"]:
                want = want | {"H", "D"}
                why = why + [f"s2 {kind}"]
            if "B" in self.conns and kind == "budget" and (
                    corner.s_in - s0 >= self.scheduler.rules["ahead"]
                    or self._slides_in(self.last["B"], corner)):
                want = want | {"H", "B"}
                why = why + ["s2 forced"]
        adopted = d.plan
        g_from = adopted if adopted.get("mode", "grip") == "grip" else self.last["G"]
        if g_from is None:
            g_from = adopted
        if d.cut is not None:                # G's last solve cut near feasible (issue 005)
            g_from = d.cut
        starts, kinds = {}, {}
        if want - {"G"}:
            keep = d.warm.lam, d.warm.lam_x
            d.warm.lam = d.warm.lam_x = None
            base = warm_from(d.warm, g_from, i0, x0, self.track)
            d.warm.lam, d.warm.lam_x = keep
            for slot in ("D", "F"):
                if slot not in want:
                    continue
                own = self.last[slot]
                if self._slides_in(own, corner):
                    starts[slot], kinds[slot] = {"plan": own}, "own"
                    continue
                kind, seed = self._seed(slot, base, sec, corner)
                if seed is not None:
                    starts[slot], kinds[slot] = {"X": seed[0], "U": seed[1]}, kind
            if "B" in want:                  # S2: the bought corner's drift, forced
                fl = self._floor(corner)
                own = self.last["B"]
                if self._slides_in(own, corner) and own.get("floor") == fl:
                    starts["B"], kinds["B"] = {"plan": own, "floor": fl}, "own"
                else:
                    kind, seed = self._seed("D", base, sec, corner)
                    if seed is not None:
                        starts["B"], kinds["B"] = {"X": seed[0], "U": seed[1], "floor": fl}, kind
            if "H" in want:
                b = self._bound(i0, n)
                own = self.last["H"]
                if own is not None and own.get("bound") == b:
                    starts["H"], kinds["H"] = {"plan": own, "bound": b}, "own"
                else:
                    X, U = self._held_seed(base, b)
                    starts["H"], kinds["H"] = {"X": X, "U": U, "bound": b}, "held"
        c = time.time()
        for slot, st in starts.items():
            self.conns[slot].send((i0, x0, v_end, mu, st))
        # G here, as 060's one start: the multipliers d.warm has are G's own while the adopted
        # plan is G's; otherwise the start's own (if they fit this window)
        if not (g_from is adopted and adopted.get("slot", "G") == "G"):
            d.warm.lam, d.warm.lam_x = _fitting(d.warm, g_from.get("lam"), g_from.get("lam_x"))
        Xw, Uw = warm_from(d.warm, g_from, i0, x0, self.track)
        g = d.warm.solve(self.track, i0, x0, v_end, Xw, Uw, mu_scale=mu)
        g["lam"], g["lam_x"] = d.warm.lam, d.warm.lam_x
        from race_planner.live import cut_iterate
        d.cut = cut_iterate(g) if d.cut_warm else None
        res = {"G": g}
        kinds["G"] = "adopted" if g_from is adopted else "own"
        res.update({slot: self.conns[slot].recv() for slot in starts})
        # G alone: its own solve time, as 060 counts it (the warm start's few ms not added)
        wall = time.time() - c if starts else g["solve_s"]
        for slot in self.slots:
            r = res.get(slot)
            if slot in res or slot == "G":
                self.last[slot] = r if r is not None and r["ok"] else None
            elif slot in ("D", "F", "H", "B"):
                self.last[slot] = None       # not run: its plan is no longer current
            if r is not None and r["ok"]:
                if slot == "H":
                    r["bound"] = starts["H"]["bound"]
                if slot == "B":
                    r["floor"] = starts["B"]["floor"]
                self.classify(r, i0, mu, corner)
                if corner is None:
                    r["mode"], r["onset"] = self.mode, None
                if s2:
                    r["drift_m"] = drift_metres(r, corner, self.style.drift_past_deg)
        rec = {"s0": s0, "i0": i0, "wall": wall,
               "corner": None if corner is None else corner.s_in,
               "slots": sorted(res), "why": why,
               "seeds": {slot: {"start": kinds[slot], "ok": bool(r["ok"]), "iters": int(r["iters"]),
                                "solve_s": float(r["solve_s"]), "status": r["status"],
                                "time": float(r["time"]), "mode": r.get("mode"),
                                "onset": r.get("onset"),
                                **({"drift_m": r.get("drift_m")} if s2 else {})}
                         for slot, r in res.items()}}
        slot = self.choose(res, rec)
        self.records.append(rec)
        if slot is None:
            worst = max(res.values(), key=lambda r: r["iters"])
            return dict(worst, ok=False, status="no seed feasible"), wall
        return dict(res[slot], slot=slot), wall

    def _floor(self, c):
        """S2's floor for corner c: (whole degrees past which the rear is held, from its turn-in,
        to its middle, its hand) (forced.py). The peak is the tyre's at the map's friction there
        (c.mu / grip), not the planner's: a drift is the car's rear past its own peak, and on
        asphalt the planner's (grip 0.92) peaks 1.25 deg earlier, so a floor 3 deg past it held
        the car 1.9 deg past its real one (roadmap 065 M3)."""
        from race_planner import forced
        peak = modes.rear_peak(self.car, c.mu / self.grip, c.sliding, c.surface)
        deg = int(round(math.degrees(forced.angle_for(peak, self.style.drift_past_deg))))
        return (deg, float(c.s_in), float(0.5 * (c.s_in + c.s_out)), float(c.sign))

    def _s2_lap(self):
        """S2's plan for the run (budget.S2Lap), built at the first S2 re-plan and again when the
        style changes (a new account: the budget counts from the change)."""
        if self.s2 is None or self.s2.style != self.style:
            from race_planner.budget import CoursePrices, LossMap, S2Lap
            lm = self.loss_map
            if not isinstance(lm, LossMap):
                lm = LossMap.load() if lm is None else LossMap.load(lm)
            cp = getattr(self, "course_prices", None)
            if cp is not None and not isinstance(cp, CoursePrices):
                cp = CoursePrices.load(cp)
            classes, flat = s2_classes(self.scheduler, self.track)
            self.s2 = S2Lap.build(self.style, self.corners, classes, lm, flat, lap=self.lap,
                                  course=cp)
        return self.s2
