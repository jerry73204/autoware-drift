#!/usr/bin/env python3
"""The same stretch driven three ways, side by side: one panel per arm of roadmap 061's mode
choice (`single`, the fastest-lap start alone; `sched`, drift starts where the surface rule says;
`always`, every start every re-plan), each a camera following its car, and a strip below
comparing them. 1920x1080, Pillow + numpy + ffmpeg, no ROS.

    tools/arm_video.py <out dir> --arm single=<run> --arm sched=<run> --arm always=<run>
    tools/arm_video.py <out dir> --arm fastest=<run> --arm s2=<run>      (driving styles)
        [--map <course dir>] [--title S] [--fps 25] [--from S] [--to S] [--speed 1]

A run is either
- a planner-simulation run, the `<key>.traj.json` of studies/mode_choice_m4.py run ... --traj
  (sim.Plant behind sim_vehicle_ecu): the top-down view takes the whole panel;
- an Isaac run, swb_isaac_world's spikes/course_probe.py LIVE=1 output directory (live.jsonl
  every control frame with the plan the car is on, record/<time>-ego/<camera>/*.jpg): the
  recorder's chase camera above the top-down view. --map is the course's map dir (its
  centreline and surfaces.yaml: the tyre peaks the wheels are measured against).

Panels are synchronised by simulated time since the planner took the car (Isaac: LIVE_FROM;
the planner simulation: the start), stated on screen; the strip's time gap is each arm's time
to a distance against single's to the same distance. Each panel: the road, the plan the car is
on ahead of it (orange where the plan's sideslip passes 5 deg, cyan below), the car's path (white,
orange where it slid more than 5 deg), the car seen from above with its velocity against its
heading and its tyres green to red by slip against the surface's peak (red at the peak and past
it), and the seed of the plan (G the warm start, H held at the rear tyre's peak, D a drift
seed, F a flick seed) and its mode (grip or drift as the multistart classed the plan, from its
rear axle past the tyre's peak; single's one start is not classed).

Writes <out dir>/arms.mp4 and arms.png (the paths overlaid, times, sideslip and drift share).
"""
import argparse
import json
import math
import os
import subprocess
import sys

import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont

W, H = 1920, 1080
COL = W // 3
TITLE_H, STRIP_H = 64, 300
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
MONO = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
BG, PANEL, FG, DIM, GRID = (14, 15, 18), (26, 28, 33), (236, 236, 236), (140, 142, 150), (52, 54, 60)
ROAD, EDGE, GRASS = (78, 80, 86), (200, 200, 200), (34, 44, 36)
GRIP_C, DRIFT_C, SLID_C = (80, 200, 255), (255, 150, 40), (255, 140, 60)
ARM_C = {"single": (95, 170, 255), "sched": (255, 200, 70), "always": (235, 95, 205)}
ARM_TEXT = {"single": "single: grip, the fastest-lap start alone",
            "sched": "sched: drift starts where the surface rule says",
            "always": "always: every start every re-plan"}
# driving styles (roadmap 065): the same course, fastest lap against drift where it costs little
ARM_C.update({"fastest": (95, 170, 255), "s2": (255, 150, 40)})
ARM_TEXT.update({"fastest": "fastest: the fastest lap (S1)",
                 "s2": "drift_budget: drift where it costs little (S2)"})
SEED_TEXT = {"G": "G (warm start)", "H": "H (held at the peak)", "D": "D (drift seed)",
             "F": "F (flick seed)"}
SLID_DEG = 5.0
L_BOX, W_BOX = 4.8, 2.1
_fonts = {}
LEGEND = ("plan ahead: cyan, orange where its sideslip passes 5 deg  |  path: white, orange where "
          "the car slid past 5 deg  |  tyres: green gripping to red at or past the surface's "
          "peak  |  white arrow: velocity, grey line: heading")


def font(n, kind=FONT):
    if (n, kind) not in _fonts:
        _fonts[(n, kind)] = ImageFont.truetype(kind, n)
    return _fonts[(n, kind)]


def slide_color(k):
    """0 green, 0.5 yellow, 1 (the peak, and past it) red."""
    k = max(0.0, min(1.0, k))
    if k < 0.5:
        return (int(70 + 370 * k), 210, 70)
    return (255, int(210 - 380 * (k - 0.5)), 50)


# ---- the runs ----

class Road:
    def __init__(self, s, x, y, psi, wl, wr):
        self.s, self.x, self.y, self.psi = (np.asarray(v, float) for v in (s, x, y, psi))
        self.wl, self.wr = np.asarray(wl, float), np.asarray(wr, float)
        self.lx, self.ly = self.xy(self.s, self.wl)
        self.rx, self.ry = self.xy(self.s, -self.wr)

    def xy(self, s, n):
        s, n = np.asarray(s, float), np.asarray(n, float)
        x, y = np.interp(s, self.s, self.x), np.interp(s, self.s, self.y)
        ps = np.interp(s, self.s, np.unwrap(self.psi))
        return x - n * np.sin(ps), y + n * np.cos(ps)

    def heading(self, s):
        return float(np.interp(s, self.s, np.unwrap(self.psi)))

    @classmethod
    def of_map(cls, d):
        c = np.load(os.path.join(d, "course/centreline.npz"))
        half = float(np.load(os.path.join(d, "course/meshes.npz"))["half_width"])
        one = np.ones_like(c["s"])
        return cls(c["s"], c["x"], c["y"], c["heading"], half * one, half * one)


def peaks_of(map_dir):
    """The road class's peak slip ratio and slip angle (deg) from swb's surface classes."""
    cls = yaml.safe_load(open(os.path.join(map_dir, "surfaces.yaml")))["defaults"]["Road"]
    swb = os.environ.get("SWB", os.path.expanduser("~/repos/carla-scenario-bridge"))
    tab = yaml.safe_load(open(os.path.join(swb, "src/swb_isaac_world/config/surface_classes.yaml")))
    c = tab["classes"][cls]
    return cls, float(c.get("peak_slip", 0.1)), float(c.get("peak_slip_angle", 8.0)), float(c["mu"])


class Run:
    """One arm, tick by tick: t (s since the planner took the car), s, x, y (the CoM), yaw, v,
    beta (deg), slip (4: each tyre's slip over its peak), plan id; plans {id: (s, n, slot,
    mode)}; frames (t, jpg) for a camera, or none; result and its distance."""

    def __init__(self, arm):
        self.arm, self.frames, self.plans = arm, None, {}

    @classmethod
    def sim(cls, arm, path):
        d = json.load(open(path))
        r = cls(arm)
        rows = d["rows"]
        r.t = np.array([x["t"] for x in rows])
        r.t = r.t - r.t[0]
        for k in ("s", "x", "y", "yaw", "v", "beta_deg"):
            setattr(r, k, np.array([x[k] for x in rows], float))
        r.beta = r.beta_deg
        r.slip = np.array([x["slip"] for x in rows], float)
        r.pid = np.array([-1 if x["plan"] is None else x["plan"] for x in rows])
        r.stop = [x.get("stopping") for x in rows]
        for i, p in enumerate(d["plans"]):
            r.plans[i] = (np.array(p["s"]), np.array(p["n"]), p.get("slot") or "G", p.get("mode"),
                          np.array(p["beta_deg"]))
        rd = d["road"]
        r.road = Road(rd["s"], rd["x"], rd["y"], rd["psi"], rd["w_left"], rd["w_right"])
        res = json.load(open(path.replace(".traj.json", ".json")))
        r.result, r.config = res["result"], res
        r.source = "planner simulation (sim.Plant, sim_vehicle_ecu)"
        return r

    @classmethod
    def isaac(cls, arm, d, road, peaks):
        r = cls(arm)
        rows = [json.loads(x) for x in open(os.path.join(d, "live.jsonl"))]
        r.t = np.array([x["t"] for x in rows], float)
        r.t_sim = np.array([x.get("t_sim", np.nan) for x in rows], float)
        for k in ("s", "x", "y", "v"):
            setattr(r, k, np.array([x[k] for x in rows], float))
        r.beta = np.array([x["beta_deg"] for x in rows], float)
        r.yaw = np.array([road.heading(x["s"]) + math.radians(x["xi_deg"]) for x in rows])
        _, kp, ap, _ = peaks
        sl = []
        for x in rows:
            w = x.get("wheels") or [[0] * 7] * 4
            sl.append([math.hypot(q[5] / kp, math.degrees(q[6]) / ap) for q in w])
        r.slip = np.array(sl, float)
        r.pid = np.array([x.get("plan_id", -1) if x.get("plan_id") is not None else -1
                          for x in rows])
        r.stop = [x.get("stop") for x in rows]
        for x in rows:
            if "plan" in x:
                p = x["plan"]
                r.plans[x["plan_id"]] = (np.array(p["s"]), np.array(p["n"]), x.get("slot") or "G",
                                         x.get("mode"), np.array(p["beta_deg"]))
        r.road = road
        rec = os.path.join(d, "record")
        sub = sorted(os.listdir(rec))[-1]
        cam = os.path.join(rec, sub, "chase")
        st = [json.loads(x) for x in open(os.path.join(rec, sub, "state.jsonl"))]
        t0 = r.t_sim[0] - r.t[0]
        r.frames = (np.array([x["t"] for x in st]) - t0,
                    [os.path.join(cam, "%06d.jpg" % x["frame"]) for x in st])
        log = open(os.path.join(d, "run.log"), errors="replace").read()
        i = log.rfind("SPIKE-RESULT ")
        r.config = json.loads(log[i + 13:log.index("\n", i)]) if i >= 0 else {}
        r.result = r.config.get("result", "?")
        r.source = "Isaac Sim (PhysX vehicle, sim_vehicle_ecu)"
        return r

    def at(self, T):
        return max(0, min(len(self.t) - 1, int(np.searchsorted(self.t, T, side="right")) - 1))

    def finished(self):
        return self.result in ("done", "reached")

    def time_to(self, s):
        """Time to reach distance s (None if it never did); s is monotone enough to search."""
        sm = np.maximum.accumulate(self.s)
        if s > sm[-1] or s < sm[0]:
            return None
        return float(np.interp(s, sm, self.t))

    def stats(self, s0, s1):
        m = (self.s >= s0) & (self.s <= s1)
        ds = np.gradient(self.s)
        ds = np.where(m, np.maximum(ds, 0.0), 0.0)
        tot = max(ds.sum(), 1e-9)
        slid = float(ds[np.abs(self.beta) > SLID_DEG].sum() / tot)
        modes = [self.plans.get(int(p), (None,) * 5)[3] for p in self.pid]
        known = [md is not None for md in modes]
        dr = (float(sum(d for d, md in zip(ds, modes) if md == "drift") / tot) if any(known)
              else None)
        seeds = {}
        for p, d in zip(self.pid, ds):
            k = self.plans.get(int(p), (None, None, "G", None, None))[2]
            seeds[k] = seeds.get(k, 0.0) + d / tot
        t0, t1 = self.time_to(s0), self.time_to(s1)
        return {"t": None if t0 is None or t1 is None else t1 - t0,
                "beta_max": float(np.max(np.abs(self.beta[m]))) if m.any() else 0.0,
                "slid": slid, "drift_plans": dr, "seeds": seeds,
                "v_max": float(np.max(self.v[m])) * 3.6 if m.any() else 0.0}


# ---- drawing ----

class View:
    """A top-down camera: the road's direction at the car up, the car at (cx, cy) of a box."""

    def __init__(self, box, scale, anchor=0.68):
        self.x0, self.y0, self.x1, self.y1 = box
        self.k = scale
        self.cx, self.cy = (self.x0 + self.x1) / 2, self.y0 + anchor * (self.y1 - self.y0)

    def set(self, x, y, up):
        self.ox, self.oy = x, y
        a = math.pi / 2 - up
        self.c, self.s_ = math.cos(a), math.sin(a)

    def P(self, x, y):
        dx, dy = np.asarray(x) - self.ox, np.asarray(y) - self.oy
        rx, ry = self.c * dx - self.s_ * dy, self.s_ * dx + self.c * dy
        return self.cx + self.k * rx, self.cy - self.k * ry


def poly(px, py):
    return list(zip(np.asarray(px).tolist(), np.asarray(py).tolist()))


def draw_road(d, v, road, s_now, ahead=120.0, behind=250.0):
    m = (road.s >= s_now - behind) & (road.s <= s_now + ahead)
    if m.sum() < 2:
        return
    lx, ly = v.P(road.lx[m], road.ly[m])
    rx, ry = v.P(road.rx[m], road.ry[m])
    # quads, not one polygon: a hairpin's polygon folds on itself
    L, R = poly(lx, ly), poly(rx, ry)
    for i in range(len(L) - 1):
        d.polygon([L[i], L[i + 1], R[i + 1], R[i]], fill=ROAD)
    d.line(L, fill=EDGE, width=2)
    d.line(R, fill=EDGE, width=2)
    cx, cy = v.P(*road.xy(road.s[m], 0 * road.s[m]))
    pts = poly(cx, cy)
    for i in range(0, len(pts) - 1, 6):            # a dashed centre line
        d.line(pts[i:i + 3], fill=(150, 150, 120), width=1)


def draw_car(d, v, x, y, yaw, beta_deg, speed, slip, car_c):
    c, s = math.cos(yaw), math.sin(yaw)

    def at(lx, ly):
        return v.P(x + lx * c - ly * s, y + lx * s + ly * c)
    hl, hw = L_BOX / 2, W_BOX / 2
    body = [at(hl, hw), at(hl, -hw), at(-hl, -hw), at(-hl, hw)]
    d.polygon([tuple(map(float, p)) for p in body], fill=(30, 30, 34), outline=car_c, width=3)
    d.polygon([tuple(map(float, p)) for p in (at(hl, hw * 0.8), at(hl, -hw * 0.8),
                                               at(hl - 0.7, 0))], fill=car_c)
    lf, lr, tr = 1.45, 1.44, 1.59
    for (wx, wy), k in zip(((lf, tr / 2), (lf, -tr / 2), (-lr, tr / 2), (-lr, -tr / 2)), slip):
        q = [at(wx + 0.36, wy + 0.13), at(wx + 0.36, wy - 0.13), at(wx - 0.36, wy - 0.13),
             at(wx - 0.36, wy + 0.13)]
        d.polygon([tuple(map(float, p)) for p in q], fill=slide_color(k), outline=(0, 0, 0))
    # heading and velocity
    hx, hy = at(hl + 4.0, 0)
    ox, oy = at(0, 0)
    d.line([(float(ox), float(oy)), (float(hx), float(hy))], fill=(200, 200, 200), width=1)
    va = yaw + math.radians(beta_deg)
    ln = 1.2 + 0.25 * speed
    vx, vy = v.P(x + ln * math.cos(va), y + ln * math.sin(va))
    d.line([(float(ox), float(oy)), (float(vx), float(vy))], fill=(255, 255, 255), width=3)
    ang = math.atan2(float(vy - oy), float(vx - ox))
    for da in (2.6, -2.6):
        d.line([(float(vx), float(vy)), (float(vx) + 10 * math.cos(ang + da),
                                         float(vy) + 10 * math.sin(ang + da))],
               fill=(255, 255, 255), width=3)


def panel(img, run, T, box, ref, scale):
    bx0, by0 = box[0], box[1]
    sub = Image.new("RGB", (box[2] - box[0], box[3] - box[1]), GRASS)   # clipped to its box
    d = ImageDraw.Draw(sub)
    x0, y0, x1, y1 = box = (0, 0, sub.width - 1, sub.height - 1)
    i = run.at(T)
    v = View(box, scale)
    v.set(run.x[i], run.y[i], run.road.heading(run.s[i]))
    draw_road(d, v, run.road, run.s[i])
    # the plan the car is on, ahead of it
    p = run.plans.get(int(run.pid[i]))
    if p is not None:
        ps, pn, slot, mode, pb = p
        m = ps >= run.s[i] - 2
        if m.sum() > 1:
            px, py = v.P(*run.road.xy(ps[m], pn[m]))
            pts, bb = poly(px, py), np.abs(pb[m])
            for k in range(len(pts) - 1):      # orange where the plan slides past 5 deg
                d.line(pts[k:k + 2], fill=DRIFT_C if bb[k] > SLID_DEG else GRIP_C, width=4)
    # the car's path, the last 250 m
    j0 = max(0, int(np.searchsorted(run.s, run.s[i] - 250.0)))
    if i - j0 > 1:
        px, py = v.P(run.x[j0:i + 1], run.y[j0:i + 1])
        pts = poly(px, py)
        slid = np.abs(run.beta[j0:i + 1]) > SLID_DEG
        for k in range(0, len(pts) - 1):
            d.line(pts[k:k + 2], fill=SLID_C if slid[k] else (235, 235, 235), width=2)
    draw_car(d, v, run.x[i], run.y[i], run.yaw[i], run.beta[i], run.v[i], run.slip[i],
             ARM_C[run.arm])
    # the numbers
    ended = T > run.t[-1] + 1e-6
    slot = p[2] if p else "G"
    mode = (p[3] if p else None) or "not classed"
    col = DRIFT_C if mode == "drift" else GRIP_C if mode == "grip" else DIM
    d.rectangle((x0, y0, x0 + 236, y0 + 96), fill=(0, 0, 0))
    d.text((x0 + 8, y0 + 4), f"{run.v[i] * 3.6:5.1f} km/h", fill=FG, font=font(19, MONO))
    d.text((x0 + 8, y0 + 28), f"sideslip {run.beta[i]:+5.1f} deg", fill=FG, font=font(16, MONO))
    d.text((x0 + 8, y0 + 50), f"mode: {mode}", fill=col, font=font(17, BOLD))
    d.text((x0 + 8, y0 + 73), f"seed {SEED_TEXT.get(slot, slot)}", fill=col, font=font(15))
    d.rectangle((x1 - 200, y0, x1, y0 + 54), fill=(0, 0, 0))
    d.text((x1 - 192, y0 + 4), f"s {run.s[i]:6.0f} m", fill=FG, font=font(16, MONO))
    if ref is not None and run is not ref:
        ta, tr = run.time_to(run.s[i]), ref.time_to(run.s[i])
        if ta is not None and tr is not None:
            g = ta - tr
            d.text((x1 - 192, y0 + 28), f"vs {ref.arm} {g:+.2f} s",
                   fill=(120, 230, 120) if g < 0 else (255, 120, 110), font=font(16, MONO))
    if ended:
        msg = ("FINISHED" if run.finished() else "STOPPED: " + str(run.result)) + \
            f"  at {run.t[-1]:.1f} s, s {run.s[-1]:.0f} m"
        tw = d.textlength(msg, font=font(22, BOLD))
        cx = (x0 + x1) / 2
        d.rectangle((cx - tw / 2 - 12, y1 - 50, cx + tw / 2 + 12, y1 - 12), fill=(0, 0, 0))
        d.text((cx - tw / 2, y1 - 46), msg,
               fill=(120, 230, 120) if run.finished() else (255, 110, 100), font=font(22, BOLD))
    img.paste(sub, (bx0, by0))


class Strip:
    """Sideslip, speed and the time gap to single against distance, the three arms."""

    def __init__(self, runs, s0, s1, y0):
        self.runs, self.s0, self.s1, self.y0 = runs, s0, s1, y0
        self.boxes = [(20 + k * (W // 3), y0 + 34, (k + 1) * (W // 3) - 14, H - 46) for k in range(3)]
        bmax = max(10.0, max(float(np.max(np.abs(r.beta))) for r in runs.values()))
        self.ranges = [(-bmax * 1.05, bmax * 1.05), (0.0, max(float(np.max(r.v)) for r in
                                                                runs.values()) * 3.6 * 1.08)]
        self.ref = next(iter(runs.values()))
        gaps = []
        for r in runs.values():
            if r is not self.ref and self.ref is not None:
                g = self.gap(r)
                if g is not None and len(g[1]):
                    gaps.append(np.max(np.abs(g[1])))
        gm = max([1.0] + gaps) * 1.1
        self.ranges.append((-gm, gm))
        self.titles = ["sideslip (deg) vs distance", "speed (km/h) vs distance",
                       f"time gap to {self.ref.arm} at equal distance (s; < 0 ahead)"]
        self.base = self._base()

    def gap(self, r):
        if self.ref is None:
            return None
        sm = np.maximum.accumulate(r.s)
        hi = min(sm[-1], np.maximum.accumulate(self.ref.s)[-1])
        ss = np.arange(max(self.s0, sm[0], self.ref.s[0]), hi, 2.0)
        if len(ss) == 0:
            return ss, ss
        return ss, np.array([r.time_to(s) - self.ref.time_to(s) for s in ss])

    def X(self, b, s):
        return b[0] + (np.asarray(s) - self.s0) / (self.s1 - self.s0) * (b[2] - b[0])

    def Y(self, b, k, v):
        lo, hi = self.ranges[k]
        return b[3] - (np.asarray(v) - lo) / (hi - lo) * (b[3] - b[1])

    def _base(self):
        img = Image.new("RGB", (W, H - self.y0), BG)
        d = ImageDraw.Draw(img)
        for k, b in enumerate(self.boxes):
            bb = (b[0], b[1] - self.y0, b[2], b[3] - self.y0)
            d.rectangle(bb, fill=PANEL)
            d.text((b[0], b[1] - self.y0 - 26), self.titles[k], fill=FG, font=font(17))
            lo, hi = self.ranges[k]
            for val in np.linspace(lo, hi, 5):
                yy = float(self.Y(b, k, val)) - self.y0
                d.line([(bb[0], yy), (bb[2], yy)], fill=GRID)
                d.text((bb[0] + 3, yy - 14), f"{val:.0f}" if hi - lo > 8 else f"{val:.1f}",
                       fill=DIM, font=font(11))
            for sv in range(int(self.s0 // 200 + 1) * 200, int(self.s1), 200):
                xx = float(self.X(b, sv))
                d.line([(xx, bb[1]), (xx, bb[3])], fill=GRID)
                d.text((xx + 2, bb[3] + 2), f"{sv} m", fill=DIM, font=font(11))
            if k == 0:
                for sg in (SLID_DEG, -SLID_DEG):
                    yy = float(self.Y(b, 0, sg)) - self.y0
                    d.line([(bb[0], yy), (bb[2], yy)], fill=(90, 70, 50))
        return img

    def draw(self, img, T):
        img.paste(self.base, (0, self.y0))
        d = ImageDraw.Draw(img)
        for arm, r in self.runs.items():
            i = r.at(T)
            c = ARM_C[arm]
            m = slice(0, i + 1)
            for k, val in ((0, r.beta), (1, r.v * 3.6)):
                b = self.boxes[k]
                if i > 1:
                    d.line(poly(self.X(b, r.s[m]), self.Y(b, k, val[m])), fill=c, width=2)
            if r is not self.ref and self.ref is not None:
                g = self.gap(r)
                if g is not None and len(g[0]):
                    mm = g[0] <= r.s[i]
                    if mm.sum() > 1:
                        b = self.boxes[2]
                        d.line(poly(self.X(b, g[0][mm]), self.Y(b, 2, g[1][mm])), fill=c, width=2)
            b = self.boxes[0]
            xx = float(self.X(b, r.s[i]))
            for b in self.boxes[:2]:
                xx = float(self.X(b, r.s[i]))
                d.line([(xx, b[1]), (xx, b[3])], fill=c)
        b = self.boxes[2]
        yy = float(self.Y(b, 2, 0.0))
        d.line([(b[0], yy), (b[2], yy)], fill=ARM_C[self.ref.arm], width=1)


def frame(runs, T, strip, title, sub, cam_h, scale):
    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)
    d.text((20, 8), title, fill=FG, font=font(24, BOLD))
    d.text((20, 40), sub, fill=DIM, font=font(16))
    tt = f"t = {T:6.2f} s since the planner took the car (panels synced by sim time)"
    d.text((W - 20 - d.textlength(tt, font=font(16, MONO)), 40), tt, fill=FG, font=font(16, MONO))
    ref = next(iter(runs.values()))
    top = TITLE_H
    bottom = H - STRIP_H
    for k, (arm, r) in enumerate(runs.items()):
        x0 = k * COL
        d.rectangle((x0, top, x0 + COL - 1, top + 30), fill=PANEL)
        d.text((x0 + 10, top + 4), ARM_TEXT[arm], fill=ARM_C[arm], font=font(18, BOLD))
        y = top + 32
        if r.frames is not None and cam_h:
            ft, fp = r.frames
            j = max(0, min(len(fp) - 1, int(np.searchsorted(ft, min(T, r.t[-1])))))
            try:
                im = Image.open(fp[j]).convert("RGB")
                ch = int(im.width * cam_h / (COL - 4))          # the middle band, not squeezed
                top_ = (im.height - ch) // 2
                im = im.crop((0, top_, im.width, top_ + ch)).resize((COL - 4, cam_h),
                                                                   Image.BILINEAR)
                img.paste(im, (x0 + 2, y))
            except OSError:
                pass
            y += cam_h + 2
        panel(img, r, T, (x0 + 2, y, x0 + COL - 3, bottom - 4), ref, scale)
    strip.draw(img, T)
    d.text((20, H - 22), LEGEND, fill=DIM, font=font(14))
    return img


def summary_png(runs, out, title, sub, s0, s1):
    """The paths overlaid on the whole stretch, and the arms' numbers."""
    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)
    d.text((20, 8), title, fill=FG, font=font(26, BOLD))
    d.text((20, 40), sub, fill=DIM, font=font(16))
    road = next(iter(runs.values())).road
    m = (road.s >= s0 - 20) & (road.s <= s1 + 20)
    xs = np.concatenate([road.lx[m], road.rx[m]])
    ys = np.concatenate([road.ly[m], road.ry[m]])
    bx = (20, 80, 1180, H - 20)
    k = min((bx[2] - bx[0]) / (xs.max() - xs.min()), (bx[3] - bx[1]) / (ys.max() - ys.min()))
    ox = bx[0] + ((bx[2] - bx[0]) - k * (xs.max() - xs.min())) / 2
    oy = bx[3] - ((bx[3] - bx[1]) - k * (ys.max() - ys.min())) / 2

    def P(x, y):
        return ox + k * (np.asarray(x) - xs.min()), oy - k * (np.asarray(y) - ys.min())
    d.rectangle(bx, fill=GRASS)
    L, R = poly(*P(road.lx[m], road.ly[m])), poly(*P(road.rx[m], road.ry[m]))
    for i in range(len(L) - 1):
        d.polygon([L[i], L[i + 1], R[i + 1], R[i]], fill=ROAD)
    for arm, r in runs.items():
        d.line(poly(*P(r.x, r.y)), fill=ARM_C[arm], width=2)
        ex, ey = P(r.x[-1], r.y[-1])
        if not r.finished():
            d.ellipse((ex - 9, ey - 9, ex + 9, ey + 9), outline=ARM_C[arm], width=3)
            d.text((ex + 10, ey - 10), f"{arm}: {r.result}", fill=ARM_C[arm], font=font(16, BOLD))
    for sv in range(int(s0 // 100 + 1) * 100, int(s1), 100):
        px, py = P(*road.xy(sv, 0.0))
        d.text((float(px) + 4, float(py) - 6), f"{sv}", fill=DIM, font=font(11))
    x, y = 1210, 90
    d.text((x, y), f"stretch: s {s0:.0f} to {s1:.0f} m", fill=FG, font=font(20, BOLD))
    y += 40
    rows = {}
    for arm, r in runs.items():
        st = r.stats(s0, s1)
        rows[arm] = st
        d.text((x, y), ARM_TEXT[arm], fill=ARM_C[arm], font=font(18, BOLD))
        y += 28
        tt = f"{st['t']:.2f} s" if st["t"] is not None else "did not reach the end"
        lines = [f"result: {r.result} at s {r.s[-1]:.0f} m",
                 f"time over the stretch: {tt}",
                 f"peak sideslip {st['beta_max']:.1f} deg, top speed {st['v_max']:.0f} km/h",
                 f"distance with |sideslip| > {SLID_DEG:.0f} deg: {100 * st['slid']:.0f} %",
                 "distance on drift-mode plans: " + ("-" if st["drift_plans"] is None
                                                     else f"{100 * st['drift_plans']:.0f} %"),
                 "seeds (share of distance): " + ", ".join(
                     f"{s} {100 * v:.0f} %" for s, v in sorted(st["seeds"].items()))]
        for ln in lines:
            d.text((x + 14, y), ln, fill=FG, font=font(15))
            y += 22
        y += 18
    d.text((x, y + 10), "sideslip: the CoM's velocity against the body's heading", fill=DIM,
           font=font(13))
    img.save(out)
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--arm", action="append", required=True, help="name=path (single, sched, always)")
    ap.add_argument("--map", default=None, help="the course's map dir (Isaac runs)")
    ap.add_argument("--title", default="")
    ap.add_argument("--sub", default="")
    ap.add_argument("--fps", type=int, default=25)
    ap.add_argument("--speed", type=float, default=1.0, help="sim seconds per video second")
    ap.add_argument("--from", dest="s0", type=float, default=None, help="the stretch's first s")
    ap.add_argument("--to", dest="s1", type=float, default=None, help="the stretch's last s")
    ap.add_argument("--scale", type=float, default=6.0, help="px per m, top-down")
    ap.add_argument("--frames", type=int, default=None, help="only this many (a test)")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    runs = {}
    road = Road.of_map(a.map) if a.map else None
    peaks = peaks_of(a.map) if a.map else None
    for x in a.arm:
        arm, path = x.split("=", 1)
        runs[arm] = (Run.sim(arm, path) if path.endswith(".traj.json")
                     else Run.isaac(arm, path, road, peaks))
    global COL
    COL = W // len(runs)                    # two or three panels; the first arm is the reference
    s0 = a.s0 if a.s0 is not None else max(r.s[0] for r in runs.values())
    s1 = a.s1 if a.s1 is not None else min(r.s[-1] for r in runs.values())   # the stretch all arms drove
    stats = summary_png(runs, os.path.join(a.out, "arms.png"), a.title, a.sub, s0, s1)
    json.dump(stats, open(os.path.join(a.out, "arms_stats.json"), "w"), indent=1)
    print(json.dumps(stats, indent=1))
    strip = Strip(runs, s0, s1, H - STRIP_H)
    cam = any(r.frames is not None for r in runs.values())
    cam_h = 300 if cam else 0
    T1 = max(r.t[-1] for r in runs.values()) + 2.0
    n = int(T1 / a.speed * a.fps)
    if a.frames:
        n = min(n, a.frames)
    mp4 = os.path.join(a.out, "arms.mp4")
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt",
                           "rgb24", "-s", f"{W}x{H}", "-r", str(a.fps), "-i", "-", "-c:v",
                           "libx264", "-preset", "medium", "-crf", "22", "-pix_fmt", "yuv420p",
                           mp4], stdin=subprocess.PIPE)
    for f in range(n):
        T = f * a.speed / a.fps
        ff.stdin.write(frame(runs, T, strip, a.title, a.sub, cam_h, a.scale).tobytes())
        if f % 250 == 0:
            print(f"{f}/{n}", flush=True)
    ff.stdin.close()
    ff.wait()
    print(mp4)


if __name__ == "__main__":
    sys.exit(main())
