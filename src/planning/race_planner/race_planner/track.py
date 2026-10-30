"""A track as the planner sees it: a reference line with the free width either side, the
road's grade and its friction, along s. Nothing in it is a setting for a corner.

`Track` is the planner's one input format. Adapters build it from a source:
`from_swb_course` from swb's course generator (a map directory: course/centreline.npz,
course/meshes.npz, surfaces.yaml), more (a Lanelet2 map's boundaries) as tracks need them.

Fields (arrays over s, metres and radians, map frame): x, y, z, psi (the line's heading),
kappa (its curvature, > 0 turning left), w_left / w_right (the road's free width either side of
the line, to its edges; the car's half width and a margin are the planner's to take off), mu (the
tyre-road friction there for this car), sliding (its share past the peak), grade (dz/ds), closed
(a circuit), and the surface's tyre (roadmap 061 M2: stiff_long, stiff_lat, peak_long,
peak_lat, its stiffnesses and peak slips as shares of the rig's, model.tyre's `surface`; all 1,
the default, is the rig's own tyre, dry asphalt's).
"""
from dataclasses import dataclass, field, replace
import math
import os

import numpy as np
import yaml


@dataclass
class SurfaceTable:
    """swb's surface classes: each class's peak mu, an absolute coefficient at tyre friction 1.0
    (the reference class's was measured on the PhysX rig: dry asphalt 1.12), its sliding share,
    and where its tyre peaks (`peak_slip`, `peak_slip_angle` in degrees; None: the rig's)."""
    reference: str
    mu_of: dict
    sliding_of: dict = field(default_factory=dict)
    peaks_of: dict = field(default_factory=dict)

    @classmethod
    def from_yaml(cls, path):
        d = yaml.safe_load(open(path))
        c = d["classes"]
        return cls(d["reference"], {k: float(v["mu"]) for k, v in c.items()},
                   {k: float(v.get("sliding", 1.0)) for k, v in c.items()},
                   {k: (v.get("peak_slip"), v.get("peak_slip_angle")) for k, v in c.items()})

    def sliding(self, cls_name):
        """The class's friction past the peak, sliding, as a share of the peak's."""
        return self.sliding_of.get(cls_name, 1.0)

    def mu(self, cls_name, car):
        """A wheel of `car` on `cls_name`: the class's mu times the tyre's friction (swb's surface
        layer; checked on Isaac's sweeps of drift_mustang, see test_model)."""
        return car.mu * self.mu_of[cls_name]

    def tyre(self, cls_name, car, friction_scale=1.0):
        """A wheel of `car` on `cls_name` (its friction times friction_scale, swb's road
        condition): its tyre as model.tyre's `surface`, (stiff_long, stiff_lat, peak_long,
        peak_lat), shares of the rig's. swb's rule (surfaces.WheelSurfaces.update): the peak is
        the class's own, or the rig's at the tyre's friction; each stiffness is the rig's peak
        times the friction written (the tyre's times the class's mu over the reference's) over
        the class's peak, so PhysX's tyre peaks there. On the reference class, dry: all 1."""
        from race_planner.model import peaks
        k_rig, t_rig = peaks(car)                 # at the tyre's friction already
        kp, deg = self.peaks_of.get(cls_name, (None, None))
        kp = k_rig if kp is None else float(kp)
        tp = t_rig if deg is None else math.tan(math.radians(float(deg)))
        rel = friction_scale * self.mu_of[cls_name] / self.mu_of[self.reference]
        a_lon, a_lat = kp / k_rig, tp / t_rig
        return (rel / a_lon, rel / a_lat, a_lon, a_lat)


TYRE_FIELDS = ("stiff_long", "stiff_lat", "peak_long", "peak_lat")   # model.tyre's surface


@dataclass
class Track:
    s: np.ndarray
    x: np.ndarray
    y: np.ndarray
    z: np.ndarray
    psi: np.ndarray
    kappa: np.ndarray
    w_left: np.ndarray
    w_right: np.ndarray
    mu: np.ndarray
    closed: bool
    name: str = ""
    grade: np.ndarray = field(default=None)
    sliding: np.ndarray = field(default=None)
    stiff_long: np.ndarray = field(default=None)
    stiff_lat: np.ndarray = field(default=None)
    peak_long: np.ndarray = field(default=None)
    peak_lat: np.ndarray = field(default=None)

    def __post_init__(self):
        if self.sliding is None:
            self.sliding = np.ones_like(self.s)
        for k in TYRE_FIELDS:
            if getattr(self, k) is None:
                setattr(self, k, np.ones_like(self.s, dtype=float))
        if self.grade is None:
            self.grade = np.gradient(self.z, self.s) if len(self.s) > 1 else np.zeros_like(self.s)

    @property
    def length(self):
        return float(self.s[-1] - self.s[0])

    def at(self, s):
        """Every field at s (linear interpolation; psi unwrapped)."""
        return {k: float(np.interp(s, self.s, getattr(self, k)))
                for k in ("x", "y", "z", "psi", "kappa", "w_left", "w_right", "mu", "grade",
                          "sliding") + TYRE_FIELDS}

    def resampled(self, ds):
        n = int(round(self.length / ds))
        s = np.linspace(self.s[0], self.s[-1], n + 1)
        f = {k: np.interp(s, self.s, getattr(self, k))
             for k in ("x", "y", "z", "psi", "kappa", "w_left", "w_right", "mu", "grade", "sliding")
                + TYRE_FIELDS}
        return replace(self, s=s, **f)

    def extended(self, length):
        """An open track with a straight run-off `length` m past its end, on its last node's
        heading, width and surface: what a window reaching past the finish plans on (a car
        crosses the line at speed and has to stop somewhere)."""
        assert not self.closed, "a closed track has no end"
        step = float(self.s[-1] - self.s[-2])
        d = np.arange(1, int(np.ceil(length / step)) + 1) * step
        psi = float(self.psi[-1])
        f = {k: np.concatenate([getattr(self, k), np.full(len(d), getattr(self, k)[-1])])
             for k in ("z", "w_left", "w_right", "mu", "grade", "sliding") + TYRE_FIELDS}
        f["z"][len(self.s):] = self.z[-1] + float(self.grade[-1]) * d
        return replace(self, s=np.concatenate([self.s, self.s[-1] + d]),
                       x=np.concatenate([self.x, self.x[-1] + d * np.cos(psi)]),
                       y=np.concatenate([self.y, self.y[-1] + d * np.sin(psi)]),
                       psi=np.concatenate([self.psi, np.full(len(d), psi)]),
                       kappa=np.concatenate([self.kappa, np.zeros(len(d))]), **f)

    def unrolled(self, laps):
        """A closed track as an open road `laps` laps long, its loop repeated end to end (s and
        psi running on): what the live loop drives, since a receding window never needs the loop
        to close. A closed track's last node is one step before its first (058's convention);
        the result ends on the last lap's first node, so it is `laps` laps long exactly."""
        assert self.closed, "only a closed track unrolls"
        step = float(self.s[1] - self.s[0])
        lap = self.length + step
        # the loop's whole turn: the last node's heading plus the last step's, a multiple of 2 pi
        last = float(np.remainder(self.psi[0] - self.psi[-1] + np.pi, 2 * np.pi) - np.pi)
        turn = float(self.psi[-1] - self.psi[0]) + last
        n = len(self.s)
        idx = np.concatenate([np.arange(n)] * laps + [[0]])
        k = np.repeat(np.arange(laps + 1), n)[:len(idx)]
        f = {key: getattr(self, key)[idx]
             for key in ("x", "y", "z", "kappa", "w_left", "w_right", "mu", "grade", "sliding")
             + TYRE_FIELDS}
        return replace(self, s=self.s[idx] + k * lap, psi=self.psi[idx] + k * turn, closed=False,
                       **f)

    @classmethod
    def from_swb_course(cls, map_dir, surfaces, car):
        """A section built by swb's course generator (swb_isaac_world.course): its centreline,
        the road's half width either side, and the road's surface class from surfaces.yaml's
        label defaults (a course's road is labelled Road; zones are not read yet)."""
        c = np.load(os.path.join(map_dir, "course/centreline.npz"))
        m = np.load(os.path.join(map_dir, "course/meshes.npz"))
        half = float(m["half_width"])
        road_cls = yaml.safe_load(open(os.path.join(map_dir, "surfaces.yaml")))["defaults"]["Road"]
        s = np.asarray(c["s"], float)
        ones = np.ones_like(s)
        return cls(s=s, x=np.asarray(c["x"], float), y=np.asarray(c["y"], float),
                   z=np.asarray(c["z"], float), psi=np.asarray(c["heading"], float),
                   kappa=np.asarray(c["curvature"], float), w_left=half * ones,
                   w_right=half * ones, mu=surfaces.mu(road_cls, car) * ones,
                   sliding=surfaces.sliding(road_cls) * ones,
                   **{k: v * ones for k, v in zip(TYRE_FIELDS, surfaces.tyre(road_cls, car))},
                   closed="closed" in c.files and bool(c["closed"]),   # a circuit (058's loops)
                   name=os.path.basename(os.path.normpath(map_dir)),
                   grade=np.asarray(c["grade"], float))
