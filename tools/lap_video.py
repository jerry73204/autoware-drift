#!/usr/bin/env python3
"""A lap's proof video: a camera that follows the car, a bird's-eye view of the ring, the drift's
numbers and the localization against the simulator's truth, one 1920x1080 frame per simulated
frame.

    tools/lap_video.py <record dir>/<time>-<ego> <bag dir> <lanelet2_map.osm> [out.mp4]
        [--title S] [--from T] [--to T] [--fps 20] [--camera chase|dash|course]

The record directory is swb_isaac_world's (`just isaac-world record:=<dir>`): <camera>/*.jpg and
state.jsonl, the simulator's truth for the ego every frame, each wheel's slip and slip angle
among it. The slide is drawn from those: on the camera, a car seen from above whose tires go from
green (gripping) to red (sliding) with its velocity against its heading (the sideslip); in the
bird's-eye inset, a skid mark wherever a tire slides, kept for the rest of the lap. The bag is the lap script's
(rosbag2, mcap) on the ego's domain: /localization/kinematic_state (the EKF),
/localization/pose_estimator/pose_with_covariance (NDT) and its NVTL,
/vehicle/status/velocity_status, /drift/state, the control commands, /api/operation_mode/state,
/api/fail_safe/mrm_state, /clock. Stamps are simulated time + 1000 s (swb's BASE_NS); the
recorder's `t` is simulated time.

Needs the ROS environment for the bag (source Autoware) and Pillow, numpy, lanelet2, ffmpeg.
"""
import argparse
import bisect
import json
import math
import os
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

W, H = 960, 540
BASE = 1000.0
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"
BG, PANEL, FG, DIM, GRID = (16, 16, 18), (24, 24, 28), (235, 235, 235), (140, 140, 148), (55, 55, 60)
TRUTH, EKF, NDT, ACCENT, BAD = (240, 240, 240), (80, 170, 255), (255, 170, 60), (255, 210, 70), (240, 80, 80)
PHASE_COLOR = {"cruise": (120, 200, 120), "brake": (255, 150, 60), "turn_in": (255, 90, 160),
               "hold": (255, 60, 60), "exit": (200, 120, 255), "recover": (90, 200, 255),
               "rejoin": (180, 180, 80)}
_fonts = {}
# drift_mustang's wheels in base_link (the rear axle at x 0), aib's order: FL, FR, RL, RR
WHEEL_BASE, TRACK = 2.886, 1.586
WHEEL_XY = ((WHEEL_BASE, TRACK / 2), (WHEEL_BASE, -TRACK / 2), (0.0, TRACK / 2), (0.0, -TRACK / 2))
# a tire slides when its slip ratio or slip angle is past roughly its peak's
SLIP_FULL, ANGLE_FULL = 0.15, 8.0
SKID = (12, 12, 12)


def slide(w):
    """0 for a tire well inside its grip, 1 for one at or past its peak."""
    return min(1.0, math.hypot(w.get("slip", 0.0) / SLIP_FULL, w.get("slip_angle_deg", 0.0) / ANGLE_FULL))


def slide_color(k):
    """Green to yellow to red."""
    if k < 0.5:
        return (int(80 + 350 * k), 210, 80)
    return (255, int(210 - 340 * (k - 0.5)), 60)


def wheel_world(r, lx, ly):
    yaw = math.radians(r["yaw_deg"])
    return (r["x"] + lx * math.cos(yaw) - ly * math.sin(yaw), r["y"] + lx * math.sin(yaw) + ly * math.cos(yaw))


def slide_hud(d, x0, y0, r, beta_deg):
    """A car from above, nose up, its tires coloured by how much they slide, and its velocity's
    direction against its heading."""
    w, h = 230, 250
    d.rectangle((x0, y0, x0 + w, y0 + h), fill=(0, 0, 0))
    d.text((x0 + 8, y0 + 6), "tires: grip -> slide", fill=FG, font=font(14))
    cx, cy, s = x0 + w / 2, y0 + 140, 26.0  # the car's middle, px per m
    mid = WHEEL_BASE / 2
    P = lambda lx, ly: (cx - ly * s, cy - (lx - mid) * s)
    body = [P(3.72, 0.95), P(3.72, -0.95), P(-1.0, -0.95), P(-1.0, 0.95)]
    d.polygon(body, outline=FG, width=2)
    for (lx, ly), wh in zip(WHEEL_XY, r["wheels"]):
        k = slide(wh)
        steer = r["tire_angle"] if lx > 0 else 0.0
        a, b = math.cos(steer), math.sin(steer)
        hl, hw = 0.33, 0.12
        pts = [P(lx + dx * a - dy * b, ly + dx * b + dy * a) for dx, dy in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]
        d.polygon(pts, fill=slide_color(k))
    # the velocity: the car slides when it points away from the nose
    beta = math.radians(beta_deg)
    L = 3.2
    tip = P(mid + L * math.cos(beta), L * math.sin(beta))
    d.line([P(mid, 0), tip], fill=ACCENT, width=4)
    d.ellipse((tip[0] - 5, tip[1] - 5, tip[0] + 5, tip[1] + 5), fill=ACCENT)
    d.line([P(mid, 0), P(mid + L, 0)], fill=DIM, width=1)
    color = BAD if abs(beta_deg) > 4 else FG
    d.text((x0 + 8, y0 + h - 26), f"sideslip {beta_deg:+5.1f} deg", fill=color, font=font(18, True))
    if any(slide(wh) > 0.8 for wh in r["wheels"][2:]):
        d.text((x0 + 8, y0 + 26), "REAR SLIDING", fill=BAD, font=font(16, True))


def font(size, bold=False):
    key = (size, bold)
    if key not in _fonts:
        _fonts[key] = ImageFont.truetype(BOLD if bold else FONT, size)
    return _fonts[key]


class Series:
    """A time series, sorted by t, looked up by the latest sample at or before a time."""

    def __init__(self):
        self.t, self.v = [], []

    def add(self, t, v):
        self.t.append(t)
        self.v.append(v)

    def finish(self):
        order = sorted(range(len(self.t)), key=self.t.__getitem__)
        self.t = [self.t[i] for i in order]
        self.v = [self.v[i] for i in order]
        return self

    def at(self, t, default=None):
        i = bisect.bisect_right(self.t, t) - 1
        return self.v[i] if i >= 0 else default

    def window(self, t0, t1):
        a, b = bisect.bisect_left(self.t, t0), bisect.bisect_right(self.t, t1)
        return self.t[a:b], self.v[a:b]


def stamp(s):
    return s.sec + s.nanosec * 1e-9 - BASE


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    s = {k: Series() for k in ("ekf", "ndt", "nvtl", "vel", "drift", "cmd", "mode", "mrm")}
    clock = None
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic not in types:
            continue
        m = deserialize_message(data, types[topic])
        if topic == "/clock":
            clock = stamp(m.clock)
        elif topic == "/localization/kinematic_state":
            p = m.pose.pose.position
            s["ekf"].add(stamp(m.header.stamp), (p.x, p.y, p.z, yaw_of(m.pose.pose.orientation)))
        elif topic == "/localization/pose_estimator/pose_with_covariance":
            p = m.pose.pose.position
            s["ndt"].add(stamp(m.header.stamp), (p.x, p.y, p.z))
        elif topic == "/localization/pose_estimator/nearest_voxel_transformation_likelihood":
            s["nvtl"].add(stamp(m.stamp), m.data)
        elif topic == "/vehicle/status/velocity_status":
            s["vel"].add(stamp(m.header.stamp), (m.longitudinal_velocity, m.lateral_velocity,
                                                 m.heading_rate))
        elif topic == "/drift/state" and clock is not None:
            s["drift"].add(clock, json.loads(m.data))
        elif topic == "/control/command/drift/control_cmd":
            s["cmd"].add(stamp(m.stamp), (m.lateral.steering_tire_angle, m.longitudinal.acceleration))
        elif topic == "/api/operation_mode/state":
            s["mode"].add(stamp(m.stamp), m.mode)
        elif topic == "/api/fail_safe/mrm_state":
            s["mrm"].add(stamp(m.stamp), m.state)
    return {k: v.finish() for k, v in s.items()}


def ring_lines(osm, bbox):
    """Every road lanelet's bounds inside bbox (x0, y0, x1, y1), in the map frame."""
    import lanelet2
    from lanelet2.io import Origin
    from lanelet2.projection import LocalCartesianProjector

    m = lanelet2.io.load(osm, LocalCartesianProjector(Origin(0, 0)))
    for p in m.pointLayer:
        if "local_x" in p.attributes:
            p.x, p.y = float(p.attributes["local_x"]), float(p.attributes["local_y"])
    x0, y0, x1, y1 = bbox
    lines = []
    for ll in m.laneletLayer:
        if ll.attributes["subtype"] != "road" if "subtype" in ll.attributes else True:
            continue
        for b in (ll.leftBound, ll.rightBound):
            pts = [(p.x, p.y) for p in b]
            if any(x0 <= x <= x1 and y0 <= y <= y1 for x, y in pts):
                lines.append(pts)
    return lines


class Bev:
    """The ring from above, north up, with an inset that follows the car."""

    def __init__(self, lines, truth, points):
        xs = [p["x"] for p in points] + [r["x"] for _, r in truth]
        ys = [p["y"] for p in points] + [r["y"] for _, r in truth]
        xs += [-250.0, 215.0]  # Town05's ring, whatever part of it the run covers
        ys += [-215.0, 215.0]
        cx, cy = 0.5 * (min(xs) + max(xs)), 0.5 * (min(ys) + max(ys))
        span = max(max(xs) - min(xs) + 60, (max(ys) - min(ys) + 60) * W / H)
        self.cx, self.cy, self.scale = cx, cy, (W - 20) / span
        self.base = Image.new("RGB", (W, H), PANEL)
        d = ImageDraw.Draw(self.base)
        for pts in lines:
            d.line([self.px(x, y) for x, y in pts], fill=GRID, width=1)
        for p in points:
            x, y = self.px(p["x"], p["y"])
            d.ellipse((x - 5, y - 5, x + 5, y + 5), outline=ACCENT, width=2)
            d.text((x + 8, y - 8), p["name"], fill=ACCENT, font=font(13))
        self.lines = lines

    def px(self, x, y, c=None, s=None):
        cx, cy, s = (self.cx, self.cy, self.scale) if c is None else (c[0], c[1], s)
        return (W / 2 + (x - cx) * s, H / 2 - (y - cy) * s)

    def car(self, d, x, y, yaw, c, s, color, length=4.72, width=1.9, rear=1.0):
        pts = []
        for lx, ly in ((length - rear, width / 2), (length - rear, -width / 2), (-rear, -width / 2),
                       (-rear, width / 2)):
            pts.append(self.px(x + lx * math.cos(yaw) - ly * math.sin(yaw),
                               y + lx * math.sin(yaw) + ly * math.cos(yaw), c, s))
        d.polygon(pts, outline=color, width=2)

    def draw(self, trail, now_truth, ekf, ndt, phase, skids=(), beta_deg=0.0):
        img = self.base.copy()
        d = ImageDraw.Draw(img)
        for (a, b, ph) in trail:
            d.line([self.px(*a), self.px(*b)], fill=PHASE_COLOR.get(ph, FG), width=3)
        x, y = now_truth["x"], now_truth["y"]
        px, py = self.px(x, y)
        d.ellipse((px - 6, py - 6, px + 6, py + 6), fill=TRUTH)
        # inset: 70 m around the car
        iw, ih, s = 360, 270, 270 / 70.0
        inset = Image.new("RGB", (iw, ih), (10, 10, 12))
        di = ImageDraw.Draw(inset)
        ip = lambda X, Y: (iw / 2 + (X - x) * s, ih / 2 - (Y - y) * s)
        for pts in self.lines:
            if any(abs(X - x) < 60 and abs(Y - y) < 60 for X, Y in pts):
                di.line([ip(X, Y) for X, Y in pts], fill=GRID, width=1)
        for (a, b, k) in skids:
            if abs(a[0] - x) < 60 and abs(a[1] - y) < 60:
                di.line([ip(*a), ip(*b)], fill=tuple(int(c * (0.4 + 0.6 * k)) for c in (255, 120, 90)), width=3)
        for (a, b, ph) in trail[-400:]:
            di.line([ip(*a), ip(*b)], fill=PHASE_COLOR.get(ph, FG), width=1)
        yaw = math.radians(now_truth["yaw_deg"])

        def box(X, Y, Yaw, color):
            pts = []
            for lx, ly in ((3.72, 0.95), (3.72, -0.95), (-1.0, -0.95), (-1.0, 0.95)):
                pts.append(ip(X + lx * math.cos(Yaw) - ly * math.sin(Yaw),
                              Y + lx * math.sin(Yaw) + ly * math.cos(Yaw)))
            di.polygon(pts, outline=color, width=2)

        box(x, y, yaw, TRUTH)
        course = yaw + math.radians(beta_deg)  # the velocity's direction
        sp = max(now_truth["speed"], 0.0)
        a0 = ip(x + 1.4 * math.cos(yaw), y + 1.4 * math.sin(yaw))
        a1 = ip(x + 1.4 * math.cos(yaw) + 0.4 * sp * math.cos(course), y + 1.4 * math.sin(yaw) + 0.4 * sp * math.sin(course))
        di.line([a0, a1], fill=ACCENT, width=3)
        if ekf:
            box(ekf[0], ekf[1], ekf[3], EKF)
        if ndt:
            nx, ny = ip(ndt[0], ndt[1])
            di.ellipse((nx - 4, ny - 4, nx + 4, ny + 4), outline=NDT, width=2)
        di.rectangle((0, 0, iw - 1, ih - 1), outline=GRID)
        img.paste(inset, (W - iw - 10, H - ih - 10))
        d.text((12, 8), "bird's-eye view", fill=DIM, font=font(16))
        d.text((12, 30), f"phase: {phase}", fill=PHASE_COLOR.get(phase, FG), font=font(20, True))
        d.text((W - iw - 10, H - ih - 32), "70 m around:", fill=DIM, font=font(14))
        lx = W - iw - 10 + 110
        for label, color in (("truth", TRUTH), ("EKF", EKF), ("NDT", NDT), ("skid", (255, 120, 90))):
            d.text((lx, H - ih - 32), label, fill=color, font=font(14, True))
            lx += 60
        return img


def plot(d, box, series, t0, t1, lo, hi, label, ref=None):
    """series: [(ts, ys, color)]."""
    x0, y0, x1, y1 = box
    d.rectangle(box, outline=GRID)
    d.text((x0 + 6, y0 + 3), label, fill=DIM, font=font(14))
    if ref is not None and lo < ref < hi:
        ry = y1 - (ref - lo) / (hi - lo) * (y1 - y0)
        d.line([(x0, ry), (x1, ry)], fill=(90, 50, 50), width=1)
    for ts, ys, color in series:
        pts = [(x0 + (t - t0) / (t1 - t0) * (x1 - x0),
                y1 - (min(max(v, lo), hi) - lo) / (hi - lo) * (y1 - y0)) for t, v in zip(ts, ys)
               if v is not None]
        if len(pts) > 1:
            d.line(pts, fill=color, width=2)
    d.text((x1 - 70, y0 + 3), f"{hi:g}", fill=DIM, font=font(12))
    d.text((x1 - 70, y1 - 16), f"{lo:g}", fill=DIM, font=font(12))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("record")
    ap.add_argument("bag")
    ap.add_argument("osm")
    ap.add_argument("out", nargs="?", default="lap.mp4")
    ap.add_argument("--title", default="autoware-drift: drift_mustang on Town05's ring")
    ap.add_argument("--points", default=os.path.join(os.path.dirname(__file__), "..", "src",
                    "control", "drift_controller", "config", "town05_ring.yaml"))
    ap.add_argument("--from", dest="t_from", type=float, default=None)
    ap.add_argument("--to", dest="t_to", type=float, default=None)
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--note", default="", help="a line under the localization panel's title")
    ap.add_argument("--camera", default="chase", help="the recorder's camera for the main panel: chase, dash or course")
    a = ap.parse_args()

    import yaml
    truth = []
    with open(os.path.join(a.record, "state.jsonl")) as f:
        for line in f:
            r = json.loads(line)
            if a.t_from is not None and r["t"] < a.t_from or a.t_to is not None and r["t"] > a.t_to:
                continue
            truth.append((r["t"], r))
    if not truth:
        sys.exit("no frames")
    print(f"{len(truth)} frames, t {truth[0][0]:.1f}..{truth[-1][0]:.1f} s; reading the bag",
          flush=True)
    b = read_bag(a.bag)
    points = yaml.safe_load(open(a.points))["points"]
    xs, ys = [r["x"] for _, r in truth], [r["y"] for _, r in truth]
    lines = ring_lines(a.osm, (min(xs + [-250.0]) - 40, min(ys + [-215.0]) - 40,
                               max(xs + [215.0]) + 40, max(ys + [215.0]) + 40))
    bev = Bev(lines, truth, points)

    tt = [t for t, _ in truth]
    # the true sideslip: the course against the heading over +-2 frames (the truth has no velocity
    # vector), positive with the velocity left of the nose, as the controller's estimate
    sideslip = []
    for j, (_, r) in enumerate(truth):
        p, q = truth[max(0, j - 2)][1], truth[min(len(truth) - 1, j + 2)][1]
        if r["speed"] > 2.0 and math.dist((p["x"], p["y"]), (q["x"], q["y"])) > 0.2:
            course = math.degrees(math.atan2(q["y"] - p["y"], q["x"] - p["x"]))
            sideslip.append((course - r["yaw_deg"] + 180.0) % 360.0 - 180.0)
        else:
            sideslip.append(0.0)

    def truth_at(ts):
        """The simulator's (x, y) at simulated time ts, interpolated between frames."""
        j = bisect.bisect_left(tt, ts)
        if j <= 0 or j >= len(tt):
            return None
        (ta, ra), (tb, rb) = truth[j - 1], truth[j]
        f = (ts - ta) / (tb - ta) if tb > ta else 0.0
        return (ra["x"] + f * (rb["x"] - ra["x"]), ra["y"] + f * (rb["y"] - ra["y"]))

    err_ekf, err_ndt = Series(), Series()
    for name, series, out in (("ekf", b["ekf"], err_ekf), ("ndt", b["ndt"], err_ndt)):
        for ts, v in zip(series.t, series.v):
            xy = truth_at(ts)
            if xy is not None:
                out.add(ts, math.hypot(v[0] - xy[0], v[1] - xy[1]))
    err_ekf.finish()
    err_ndt.finish()

    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                           "-s", f"{2 * W}x{2 * H}", "-r", str(a.fps), "-i", "-", "-c:v", "libx264",
                           "-pix_fmt", "yuv420p", "-crf", "20", "-preset", "veryfast", a.out],
                          stdin=subprocess.PIPE)
    trail, last_xy = [], None
    skids, last_wheels = [], None
    tz, ez = Series(), Series()
    beta_true, beta_est, yawr, spd = Series(), Series(), Series(), Series()
    for i, (t, r) in enumerate(truth):
        st = b["drift"].at(t) or {}
        phase = st.get("phase", "-")
        xy = (r["x"], r["y"])
        if last_xy is not None and math.dist(last_xy, xy) > 0.3:
            trail.append((last_xy, xy, phase))
            last_xy = xy
        elif last_xy is None:
            last_xy = xy
        wheels = [wheel_world(r, lx, ly) for lx, ly in WHEEL_XY]
        if last_wheels is not None:
            for wp, wq, wh in zip(last_wheels, wheels, r["wheels"]):
                k = slide(wh)
                if k > 0.6 and math.dist(wp, wq) < 3.0:
                    skids.append((wp, wq, k))
        last_wheels = wheels
        beta_now = sideslip[i]
        ekf, ndt = b["ekf"].at(t), b["ndt"].at(t)
        vel = b["vel"].at(t)
        if ekf:
            ez.add(t, ekf[2])
        tz.add(t, r["z"])
        if vel:
            spd.add(t, vel[0] * 3.6)
            yawr.add(t, vel[2])
        beta_true.add(t, beta_now)
        beta_est.add(t, st.get("beta_deg"))

        frame = Image.new("RGB", (2 * W, 2 * H), BG)
        # the camera
        jpg = os.path.join(a.record, a.camera, f"{r['frame']:06d}.jpg")
        if os.path.exists(jpg):
            frame.paste(Image.open(jpg).convert("RGB").resize((W, H)), (0, 0))
        d = ImageDraw.Draw(frame)
        d.rectangle((0, 0, 12 + 11 * len(a.title), 34), fill=(0, 0, 0))
        d.text((10, 6), a.title, fill=FG, font=font(18, True))
        d.rectangle((0, H - 34, 440, H), fill=(0, 0, 0))
        d.text((10, H - 30), f"{a.camera} camera   t = {t:7.2f} s (simulated)", fill=FG, font=font(16))
        slide_hud(d, W - 240, H - 260, r, beta_now)
        # bird's-eye view
        frame.paste(bev.draw(trail, r, ekf, ndt, phase, skids, beta_now), (W, 0))
        # stats
        sp = Image.new("RGB", (W, H), PANEL)
        ds = ImageDraw.Draw(sp)
        cmd = b["cmd"].at(t)
        mode = {1: "STOP", 2: "AUTONOMOUS", 3: "LOCAL", 4: "REMOTE"}.get(b["mode"].at(t), "-")
        mrm = {1: "NORMAL", 2: "MRM OPERATING", 3: "MRM SUCCEEDED", 4: "MRM FAILED"}.get(b["mrm"].at(t), "-")
        bt = beta_true.at(t)
        lines_ = [
            (f"{phase.upper():9s} {('bend ' + st['point']) if st.get('point') else ''}", PHASE_COLOR.get(phase, FG), 26),
            (f"speed     {spd.at(t, 0):6.1f} km/h", FG, 20),
            (f"sideslip  {bt if bt is not None else 0:6.1f} deg true   {st.get('beta_deg', 0) or 0:6.1f} deg est", FG, 20),
            (f"yaw rate  {yawr.at(t, 0):6.2f} rad/s", FG, 20),
            (f"command   steer {math.degrees(cmd[0]) if cmd else 0:6.1f} deg   accel {cmd[1] if cmd else 0:5.1f} m/s^2", FG, 20),
            (f"Autoware  {mode}   fail-safe {mrm}", BAD if mrm.startswith("MRM") else FG, 20),
        ]
        y = 8
        for text, color, size in lines_:
            ds.text((14, y), text, fill=color, font=font(size, size > 22))
            y += size + 10
        t0, t1 = t - 30.0, t + 0.5
        plot(ds, (14, 230, W - 14, 330), [(*spd.window(t0, t1), FG)], t0, t1, 0, 130, "speed km/h (30 s)")
        plot(ds, (14, 336, W - 14, 436), [(*beta_true.window(t0, t1), TRUTH), (*beta_est.window(t0, t1), ACCENT)],
             t0, t1, -60, 60, "sideslip deg: true (white), controller's estimate (yellow)", ref=0.0)
        plot(ds, (14, 442, W - 14, H - 8), [(*yawr.window(t0, t1), EKF)], t0, t1, -2.5, 2.5, "yaw rate rad/s", ref=0.0)
        frame.paste(sp, (0, H))
        # localization
        lp = Image.new("RGB", (W, H), PANEL)
        dl = ImageDraw.Draw(lp)
        nv = b["nvtl"].at(t)
        ee, ne = err_ekf.at(t), err_ndt.at(t)
        n_ndt = len(b["ndt"].window(t - 1.0, t)[0])
        dl.text((14, 8), "localization against the simulator's truth (each estimate at its own stamp)",
                fill=DIM, font=font(16))
        dl.text((14, 34), f"EKF error {ee if ee is not None else float('nan'):6.2f} m", fill=EKF, font=font(22, True))
        dl.text((360, 34), f"NDT error {ne if ne is not None else float('nan'):6.2f} m", fill=NDT, font=font(22, True))
        dl.text((14, 66), f"NDT score (NVTL) {nv if nv is not None else float('nan'):5.2f} (accepts >= 2.0)   "
                f"NDT poses in the last second: {n_ndt}", fill=BAD if (nv or 3) < 2.0 else FG, font=font(18))
        dl.text((14, 92), f"height: true {r['z']:6.2f} m   EKF {ekf[2] if ekf else float('nan'):6.2f} m", fill=FG, font=font(18))
        if a.note:
            dl.text((14, 112), a.note, fill=ACCENT, font=font(14))
        plot(dl, (14, 134, W - 14, 260), [(*err_ekf.window(t0, t1), EKF), (*err_ndt.window(t0, t1), NDT)],
             t0, t1, 0, 5, "horizontal error m: EKF (blue), NDT (orange)", ref=1.0)
        plot(dl, (14, 266, W - 14, 396), [(*tz.window(t0, t1), TRUTH), (*ez.window(t0, t1), EKF)],
             t0, t1, -2, 12, "height m: true (white), EKF (blue)")
        plot(dl, (14, 402, W - 14, H - 8), [(*b["nvtl"].window(t0, t1), NDT)], t0, t1, 0, 4,
             "NDT score (NVTL), the 2.0 gate in red", ref=2.0)
        frame.paste(lp, (W, H))
        ff.stdin.write(frame.tobytes())
        if i % 200 == 0:
            print(f"frame {i}/{len(truth)}", flush=True)
    ff.stdin.close()
    ff.wait()
    print(a.out)


if __name__ == "__main__":
    main()
