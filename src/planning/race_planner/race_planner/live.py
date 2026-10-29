"""The live loop: L1 re-planning the next window from the car's state about once a second (and at
once when the car leaves the plan), L2 tracking the current plan every tick (the live planner's
design, sections 4.2 and 5). The same loop drives sim.py's plant (model in the loop) and Isaac's
car (swb's course_probe), whatever calls `step` with the car's state at the control rate.

Time is the caller's (simulated). L1 solves synchronously, but its plan is adopted only once
its measured wall-clock solve time has passed in the caller's time, so the car feels the
latency a solver on its own core would give it, while the simulation waits for the solve. A
solve that is not feasible is dropped: the car keeps the plan it has.

The window ends no faster than the long view allows: L0's speed (the whole-track plan, if one
was computed for the track) or else the quasi-steady speed profile along the track's
minimum-curvature line (qss.py, computed here once). Every setting is the car's or the
solver's; none is per track.
"""
from dataclasses import dataclass, field
import math

import numpy as np

from race_planner.line import min_curvature_line
from race_planner.qss import speed_profile
from race_planner.window import G, Window, friction_ahead, friction_scale, initial

# leaving the plan's envelope re-plans at once (set once on the proving ground, not per track)
ENVELOPE_LAT, ENVELOPE_BETA, ENVELOPE_SPEED = 0.5, math.radians(5.0), 2.0
# and when the car finds the road markedly slipperier than the plan assumed: its friction scale
# (window.friction_scale) this far below the plan's, a share of the grip; about the margin L1
# keeps (grip 0.92), so a plan is replaced once the car has found less than it left for L2
ENVELOPE_MU = 0.08
MU_BETA = math.radians(2.5)    # the car's sideslip past the plan's that says it is at the limit
REPLAN_PERIOD = 1.0
# the car stops (L2's safety layer, design 4.3): after this many failed solves in a row, past this
# sideslip (a spin), or within this distance of the plan's end with no newer plan
FAILS_TO_STOP, SPIN_BETA, END_MARGIN = 3, math.radians(60.0), 10.0
STOP_DECEL = -6.0
BLEND = 10                         # nodes over which the warm start eases to the shifted plan
TRACK_TAU = 0.5                    # s: how fast L2 takes a deviation from the plan out


def plan_at(plan, s):
    """The plan's states (vx, vy, r, n, xi, delta), request a and time at arclength s, by
    interpolation along its nodes."""
    ss = plan["s"]
    X = np.array([np.interp(s, ss, plan["X"][i]) for i in range(6)])
    a = float(np.interp(s, ss[:-1], plan["U"][1]))
    t = float(np.interp(s, ss, plan["t"]))
    return X, a, t


class SimpleTracker:
    """Today's tracker (swb's course_probe PLAN mode), ported: the plan's steering ahead as
    feedforward with feedback on the offset, the heading and the yaw rate; the plan's request
    with feedback on the speed, never more drive than the plan's; drive and brakes eased while
    the car slides more than the plan. M3's baseline until L2 replaces it."""

    def __init__(self, car, k_lat=0.15, k_yaw=0.5, k_speed=1.0, k_r=0.3,
                 slip_cut=(math.radians(2.0), math.radians(8.0))):
        self.car, self.k = car, (k_lat, k_yaw, k_speed, k_r)
        self.slip_cut = slip_cut

    def step(self, state, plan, plan_t0):
        k_lat, k_yaw, k_speed, k_r = self.k
        s = state["s"]
        X, a_ff, _ = plan_at(plan, s)
        Xa, _, _ = plan_at(plan, s + max(state["vx"], 1.0) * 0.1)
        v = math.hypot(state["vx"], state["vy"])
        k_lat = k_lat * 10.0 / max(10.0, v)
        steer = (Xa[5] - k_lat * (state["n"] - X[3]) - k_yaw * (state["xi"] - X[4])
                 + k_r * (X[2] - state["r"]))
        steer = max(-self.car.steer_max, min(self.car.steer_max, steer))
        v_plan = math.hypot(X[0], X[1])
        a = a_ff + k_speed * (v_plan - v)
        if a_ff > 0:
            a = min(a, a_ff)
        beta = math.atan2(state["vy"], max(state["vx"], 0.5))
        excess = abs(beta) - abs(math.atan2(X[1], max(X[0], 0.5)))
        lo, hi = self.slip_cut
        a *= min(1.0, max(0.0, (hi - excess) / (hi - lo)))
        return {"accel": a, "tire_angle": steer, "ok": True, "solve_s": 0.0, "fallback": False}


@dataclass
class Driver:
    car: object
    track: object                  # on the window's grid
    tracker: object
    l0: dict = None                # {"s": ..., "speed": ...}: the whole-track plan's speed
    ds: float = 4.0
    length: float = 200.0
    grip: float = 0.92
    budget: float = 1.0            # s: L1's wall-clock budget, and the latency a plan is made for
    log: list = field(default_factory=list)

    def __post_init__(self):
        if not self.track.closed:     # the window reaches past the finish: a straight run-off,
            # two windows long (a warm start's cold tail starts up to 0.8 of a window on and
            # runs a whole one: one window crashed Isaac's run at the finish)
            self.track = self.track.extended(2 * self.length + 4 * self.ds)
        self.cold = Window(self.car, ds=self.ds, length=self.length, grip=self.grip, warm=False)
        self.warm = Window(self.car, ds=self.ds, length=self.length, grip=self.grip,
                           budget=self.budget)
        if self.l0 is None:
            line, _ = min_curvature_line(self.track, 0.5 * self.car.width, 0.3)
            self.ceiling = (self.track.s, speed_profile(line, self.car))
        else:
            self.ceiling = (np.asarray(self.l0["s"]), np.asarray(self.l0["speed"]))
        self.plan = self.pending = None
        self.next_replan = 0.0
        self.busy_until = 0.0          # L1's core is busy with a solve until then
        self.solves = []
        self.fails = 0                 # failed solves in a row
        self.stopping = None           # why the car is being stopped, once it is
        self.stop_law = SimpleTracker(self.car)
        self.plan_mu = 1.0             # the friction scale the current plan was made with

    def v_end(self, i0):
        s_end = self.track.s[min(i0 + self.warm.N - 1, len(self.track.s) - 1)]
        return float(np.interp(s_end, *self.ceiling))

    def node(self, s):
        return int(round((s - self.track.s[0]) / self.ds))

    def x_of(self, state):
        return np.array([state["vx"], state["vy"], state["r"], state["n"], state["xi"],
                         state["delta"]])

    def start(self, state):
        """The first plan, from IPOPT's own start (not under the budget: before the car goes)."""
        i0 = self.node(state["s"])
        X, U = initial(self.cold.section(self.track, i0), self.car, v_start=state["vx"])
        x0 = self.x_of(state)
        X[:, 0] = x0
        plan = self.cold.solve(self.track, i0, x0, self.v_end(i0), X, U,
                               mu_scale=self.mu_ahead(state, i0))
        self.warm.lam, self.warm.lam_x = self.cold.lam, self.cold.lam_x   # as a pair, or not at all
        if not plan["ok"]:
            raise RuntimeError(f"no first plan: {plan['status']}")
        self.plan, self.plan_t0, self.plan_mu = plan, state["t"], self.plan_mu_next
        self.next_replan = state["t"] + REPLAN_PERIOD
        # the warm solver is built on its first call: do it now, off the clock
        self._replan(state, adopt_now=True)

    def mu_scale(self, state):
        """The car's friction scale where it is (window.friction_scale; 1 without an estimate),
        against what the plan asks of the car there."""
        if "mu_ratio" not in state or self.plan is None:
            return 1.0
        mu_map = float(np.interp(state["s"], self.track.s, self.track.mu))
        s = min(max(state["s"], self.plan["s"][0]), self.plan["s"][-2])
        ax = float(np.interp(s, self.plan["s"][:-1], self.plan["U"][2]))
        ay = float(np.interp(s, self.plan["s"][:-1], self.plan["U"][3]))
        self.demand = math.hypot(ax, ay) / G
        # and only where the car shows it, sliding more than the plan meant: on dry asphalt the
        # estimate read 0.6 against a demand of 0.75-0.8 g with the car on the plan's sideslip
        # (Haruna s 204), in the patch 0.67-0.72 against 0.83 with the car sliding 1.5-5 deg more
        X, _, _ = plan_at(self.plan, state["s"])
        beta = math.atan2(state["vy"], max(state["vx"], 0.5))
        if abs(beta) - abs(math.atan2(X[1], max(X[0], 0.5))) < MU_BETA:
            return 1.0
        return friction_scale(self.car, mu_map, state["mu_ratio"], self.demand, self.grip)

    def pending_mu(self):
        return self.pending[2] if self.pending is not None else 1.0

    def mu_ahead(self, state, i0):
        """The friction scale over the window from node i0 (window.friction_ahead), from the
        car's estimate now, the window's nodes measured from the car."""
        scale = self.mu_scale(state)
        self.plan_mu_next = scale
        d = self.track.s[i0:i0 + self.warm.N] - state["s"]
        return friction_ahead(self.car, scale, d, math.hypot(state["vx"], state["vy"]))

    def off_plan(self, state):
        X, _, _ = plan_at(self.plan, state["s"])
        beta = math.atan2(state["vy"], max(state["vx"], 0.5))
        return (abs(state["n"] - X[3]) > ENVELOPE_LAT
                or abs(beta - math.atan2(X[1], max(X[0], 0.5))) > ENVELOPE_BETA
                or abs(math.hypot(state["vx"], state["vy"]) - math.hypot(X[0], X[1])) > ENVELOPE_SPEED)

    def predict(self, state, lead_t):
        """The car's state lead_t from now: where the current plan will be by then, off it by the
        car's deviation now, decayed as L2 takes it out (time constant TRACK_TAU). Neither
        extreme works: the deviation carried forward whole started windows 20-60 deg off the car
        wherever the plan's sideslip changed within the latency (and L2 did close it); the car
        integrated open loop under the plan's inputs diverged in a drift, which is unstable
        without the tracker (vy 8.6 m/s off the plan in 1 s while the real car held it)."""
        X, _, t_now = plan_at(self.plan, state["s"])
        ss, tt = self.plan["s"], self.plan["t"]
        s_ahead = float(np.interp(t_now + lead_t, tt, ss)) if t_now + lead_t < tt[-1] else \
            state["s"] + lead_t * max(state["vx"], 0.0)
        # and the car off the plan's schedule: its speed's deviation decays like the states', the
        # distance it makes on the way. Without it a car slower than its plan got windows that
        # started ahead of where it would be; plan_at then clamped it to the new plan's first
        # node, the next prediction started from that clamped time and it compounded (Isaac,
        # Tsukuba lap 2: n_plan, v_plan frozen for whole seconds, then a spin at H1)
        dv = math.hypot(state["vx"], state["vy"]) - math.hypot(X[0], X[1])
        s_ahead += dv * TRACK_TAU * (1.0 - math.exp(-lead_t / TRACK_TAU))
        Xa, _, _ = plan_at(self.plan, s_ahead)
        x = Xa + (self.x_of(state) - X) * math.exp(-lead_t / TRACK_TAU)
        x[0] = max(x[0], 1.0)
        x[5] = max(-self.car.steer_max, min(self.car.steer_max, x[5]))
        return s_ahead, x

    def _replan(self, state, adopt_now=False):
        # the window starts where the car will be when the plan is adopted, a fixed latency (the
        # budget) from now, in the state predicted for then
        lead_t = 0.0 if adopt_now else self.budget
        s0, x0 = (state["s"], self.x_of(state)) if adopt_now else self.predict(state, lead_t)
        i0 = self.node(s0)
        j = i0 - self.plan["i0"]
        if j < 1 or j >= int((1 - 0.2) * self.warm.N):
            Xw, Uw = initial(self.warm.section(self.track, i0), self.car, v_start=x0[0])
            Xw[:, 0] = x0
        else:
            Xw, Uw = self.warm.warm(self.plan, j, self.track)
            # the shifted plan starts where the plan is, the window where the car will be: the
            # difference eased out over the first nodes, so the warm start is near feasible
            k = min(BLEND, Xw.shape[1])
            Xw[:, :k] += np.outer(x0 - Xw[:, 0], 1.0 - np.arange(k) / k)
        new = self.warm.solve(self.track, i0, x0, self.v_end(i0), Xw, Uw,
                              mu_scale=self.mu_ahead(state, i0))
        self.solves.append({"t": state["t"], "s": state["s"], "ok": new["ok"],
                            "iters": new["iters"], "solve_s": new["solve_s"],
                            "status": new["status"]})
        # adopted at the latency the plan was made for (or later, if the solve overran it)
        ready = state["t"] + (0.0 if adopt_now else max(lead_t, new["solve_s"]))
        self.busy_until = ready        # one solver: the next solve starts after this one ends
        if new["ok"]:
            self.pending = (ready, new, self.plan_mu_next)
            self.fails = 0
        else:
            self.fails += 1

    def step(self, state):
        t = state["t"]
        if (self.pending is not None and t >= self.pending[0]
                and (state["s"] >= self.pending[1]["s"][0] - self.ds
                     or state["s"] > self.plan["s"][-1] - END_MARGIN)):
            # adopted once the car has reached its first node (a plan the car is behind has no
            # reference for it: plan_at clamps), or when the current plan runs out
            self.plan, self.plan_t0, self.plan_mu = self.pending[1], self.pending[0], self.pending[2]
            self.pending = None
        if state["s"] < self.plan["s"][0] - self.ds:
            self.behind = getattr(self, "behind", 0) + 1     # should not happen: counted
        reason = None
        if self.mu_scale(state) < min(self.plan_mu, self.pending_mu()) - ENVELOPE_MU:
            # the road is slipperier than any plan made or in the making knows: a new one at once,
            # the solve in hand dropped (its plan is for a road that is not there)
            reason, self.pending, self.busy_until = "mu_drop", None, t
        elif self.pending is None and t >= self.busy_until:
            if t >= self.next_replan:
                reason = "timer"
            elif self.off_plan(state):
                reason = "off_plan"
        if reason and self.stopping is None:
            self._replan(state)
            self.next_replan = t + REPLAN_PERIOD
        beta = math.atan2(state["vy"], max(abs(state["vx"]), 0.5))
        if self.stopping is None:
            if self.fails >= FAILS_TO_STOP:
                self.stopping = "solves failed"
            elif abs(beta) > SPIN_BETA:
                self.stopping = "spin"
            elif state["s"] > self.plan["s"][-1] - END_MARGIN and self.pending is None:
                self.stopping = "plan end"
        if self.stopping is None:
            cmd = self.tracker.step(dict(state, mu_scale=self.mu_scale(state)), self.plan,
                                    self.plan_t0)
        else:
            cmd = self.stop(state, beta)
        X, _, _ = plan_at(self.plan, state["s"])
        self.log.append({"t": round(t, 3), "s": round(state["s"], 2), "n": round(state["n"], 3),
                         "n_plan": round(float(X[3]), 3), "v": round(math.hypot(state["vx"], state["vy"]), 2),
                         "v_plan": round(float(math.hypot(X[0], X[1])), 2),
                         "beta_deg": round(math.degrees(math.atan2(state["vy"], max(state["vx"], 0.5))), 2),
                         "beta_plan_deg": round(math.degrees(math.atan2(X[1], max(X[0], 0.5))), 2),
                         "accel": round(cmd["accel"], 2), "steer_deg": round(math.degrees(cmd["tire_angle"]), 2),
                         "replan": reason, "fallback": cmd.get("fallback", False),
                         "mu_ratio": round(float(state.get("mu_ratio", 1.0)), 3),
                         "mu_scale": round(self.mu_scale(state), 3),
                         "demand_g": round(getattr(self, "demand", 0.0), 3),
                         "stopping": self.stopping,
                         "behind": round(float(state["s"] - self.plan["s"][0]), 2)})
        return cmd

    def stop(self, state, beta):
        """Brake to a stop: wheels along the travel in a spin (countersteer, so the car slides
        straight rather than rotating on), else a geometric law's steering on the last plan's line
        while it lasts (not L2's: its model is linearised about the plan's speed, and braking
        leaves that speed far behind -- it then countered the slowing car into a spin); straight
        ahead past it."""
        if abs(beta) > SPIN_BETA:
            steer = beta                       # the front wheels point along the velocity
        elif state["s"] < self.plan["s"][-1]:
            steer = self.stop_law.step(state, self.plan, self.plan_t0)["tire_angle"]
        else:
            steer = 0.0
        steer = max(-self.car.steer_max, min(self.car.steer_max, steer))
        return {"accel": STOP_DECEL, "tire_angle": steer, "ok": False, "solve_s": 0.0,
                "fallback": True, "stop": self.stopping}


def run(driver, plant, s_end, rate=50.0, t_max=400.0, half_width=None):
    """Drive `plant` with `driver` until s_end, a stop, or leaving the road. Returns a summary."""
    state = plant.state()
    driver.start(state)
    dt = 1.0 / rate
    result = "timeout"
    half = half_width if half_width is not None else 0.5 * driver.car.width
    while state["t"] < t_max:
        cmd = driver.step(state)
        if driver.stopping and math.hypot(state["vx"], state["vy"]) < 0.5:
            result = "stopped: " + driver.stopping
            break
        state = plant.step(cmd["accel"], cmd["tire_angle"], dt=dt)
        i = state["i"]
        if state["s"] >= s_end:
            result = "done"
            break
        if (state["n"] > plant.track.w_left[i] - half + 0.5
                or state["n"] < -(plant.track.w_right[i] - half + 0.5)):
            result = "off_road"
            break
        if abs(math.atan2(state["vy"], max(abs(state["vx"]), 0.5))) > math.radians(80):
            result = "spun"
            break
    ts = np.array([x["solve_s"] for x in driver.solves[1:]]) if len(driver.solves) > 1 else np.zeros(1)
    return {"result": result, "t": round(state["t"], 2), "s": round(state["s"], 1),
            "solves": len(driver.solves), "solves_ok": sum(x["ok"] for x in driver.solves),
            "solve_p50": round(float(np.median(ts)), 3), "solve_p95": round(float(np.percentile(ts, 95)), 3),
            "solve_max": round(float(ts.max()), 3),
            "replans_off_plan": sum(1 for r in driver.log if r["replan"] == "off_plan")}
