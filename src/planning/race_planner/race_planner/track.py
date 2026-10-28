"""A track as the planner sees it: a reference line with the free width either side, the
road's grade and its friction, along s. Nothing in it is a setting for a corner.

`Track` is the planner's one input format. Adapters build it from a source:
`from_swb_course` from swb's course generator (a map directory: course/centreline.npz,
course/meshes.npz, surfaces.yaml), more (a Lanelet2 map's boundaries) as tracks need them.

Fields (arrays over s, metres and radians, map frame): x, y, z, psi (the line's heading),
kappa (its curvature, > 0 turning left), w_left / w_right (the road's free width either side of
the line, to its edges; the car's half width and a margin are the planner's to take off), mu (the
tyre-road friction there for this car), sliding (its share past the peak), grade (dz/ds), closed
(a circuit).
"""
from dataclasses import dataclass, field, replace
import os

import numpy as np
import yaml


@dataclass
class SurfaceTable:
    """swb's surface classes: each class's peak mu, an absolute coefficient at tyre friction 1.0
    (the reference class's was measured on the PhysX rig: dry asphalt 1.12)."""
    reference: str
    mu_of: dict
    sliding_of: dict = field(default_factory=dict)

    @classmethod
    def from_yaml(cls, path):
        d = yaml.safe_load(open(path))
        return cls(d["reference"], {k: float(v["mu"]) for k, v in d["classes"].items()},
                   {k: float(v.get("sliding", 1.0)) for k, v in d["classes"].items()})

    def sliding(self, cls_name):
        """The class's friction past the peak, sliding, as a share of the peak's."""
        return self.sliding_of.get(cls_name, 1.0)

    def mu(self, cls_name, car):
        """A wheel of `car` on `cls_name`: the class's mu times the tyre's friction (swb's surface
        layer; checked on Isaac's sweeps of drift_mustang, see test_model)."""
        return car.mu * self.mu_of[cls_name]


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

    def __post_init__(self):
        if self.sliding is None:
            self.sliding = np.ones_like(self.s)
        if self.grade is None:
            self.grade = np.gradient(self.z, self.s) if len(self.s) > 1 else np.zeros_like(self.s)

    @property
    def length(self):
        return float(self.s[-1] - self.s[0])

    def at(self, s):
        """Every field at s (linear interpolation; psi unwrapped)."""
        return {k: float(np.interp(s, self.s, getattr(self, k)))
                for k in ("x", "y", "z", "psi", "kappa", "w_left", "w_right", "mu", "grade", "sliding")}

    def resampled(self, ds):
        n = int(round(self.length / ds))
        s = np.linspace(self.s[0], self.s[-1], n + 1)
        f = {k: np.interp(s, self.s, getattr(self, k))
             for k in ("x", "y", "z", "psi", "kappa", "w_left", "w_right", "mu", "grade", "sliding")}
        return replace(self, s=s, **f)

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
                   sliding=surfaces.sliding(road_cls) * ones, closed=False,
                   name=os.path.basename(os.path.normpath(map_dir)),
                   grade=np.asarray(c["grade"], float))
