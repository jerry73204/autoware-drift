"""The driving style: one record that chooses the formulation each piece of the planner runs
(roadmap 065; design docs/superpowers/specs/2026-10-31-drift-style-layer-design.md).

- S1 `fastest`: the shortest lap, today's planner unchanged (bit for bit);
- S2 `drift_budget`: a fast lap that takes a drift where it costs no or little time;
- S3 `drift_max`: drift wherever the car can, keeping speed.

L1's objective stays minimum time in every style. A style only tightens: the invariants
(design section 5: the grip share, the edge margins, the slip caps, ice's rear at its peak, the
friction quantile, the estimator's rules, H as the fallback, L0's end speed, the stop,
FREE_STAGES) are not fields, and a record naming one is refused.

Validation refuses by name, never clamps (the repo's rule): every refusal is a ValueError whose
message starts with the field's name in quotes. A field that belongs to another style than the
record's, set to other than its default, is refused too (design 8, call 9, and its extension
to S2 against S3: a field that does nothing is a mistake, not a setting).

Roadmap 065 M1 builds the record and its plumbing only: S1 runs; S2 and S3 validate, and the
planner refuses to run them (NotImplementedError, `require_implemented`) until M3 and M5.
"""
from dataclasses import MISSING, dataclass, field, fields
import math

import yaml

NAMES = ("fastest", "drift_budget", "drift_max")
IMPLEMENTED = ("fastest",)          # the styles the planner runs (M1: S1 only)

# the fields each style reads; the rest must keep their defaults under it
S2_FIELDS = ("eps_lap_s", "eps_window", "exit_hyst")
S3_FIELDS = ("zones", "straights", "slip_floor_past_deg", "speed_floor", "swing_rate_max",
             "width_use", "classes", "l2_slip_gain")
SHARED_FIELDS = ("drift_past_deg",)  # S2's D_k and S3's floor/measures
OWNERS = {"fastest": (), "drift_budget": S2_FIELDS + SHARED_FIELDS,
          "drift_max": S3_FIELDS + SHARED_FIELDS}

EPS_WINDOW = (0.003, 0.03)          # S2's eps_window when set: below the 0.3 % noise, or a lap
RULE_REAR_K = 3.0                   # the rear slip cap on classes without fall-off, x the peak
NEVER = ("ice",)                    # never a planned slide (R5)

# the invariants (design section 5): named here so a record giving one is refused as such,
# not as an unknown field. Driver / Window / surface names and the design's words alike
INVARIANTS = (
    "grip", "grip_share", "g1",
    "margin", "margins", "edge_margin", "edge_margins",
    "slip_caps", "front_slip_max", "rear_slip_max", "rear_slip_low", "rear_alpha_max",
    "ice_rear", "ice", "k_sigma", "friction_quantile", "mu_quantile",
    "estimator", "use_estimate", "friction_estimate",
    "fallback", "h_fallback", "h_competes",
    "end_speed", "l0", "v_end",
    "stop", "stop_decel", "stopping_distance", "failure_table",
    "free_stages", "FREE_STAGES",
)

# S3's default classes: the design's list (4.5) in swb's class names (mud: mud_wet, mud_soft;
# sand: sand, sand_loose); asphalt and concrete after DP7; ice never
S3_CLASSES = ("mud_wet", "mud_soft", "sand", "sand_loose", "snow_loose", "gravel", "dirt",
              "grass", "snow_packed")


def _refuse(name, why):
    raise ValueError(f"'{name}': {why}")


@dataclass(frozen=True)
class Style:
    """The style record (design 4.5). Build with `Style()` (fastest), `Style.from_dict` or
    `Style.from_yaml`; every constructor validates."""
    name: str = "fastest"
    eps_lap_s: float = 0.0           # S2: seconds a lap the user gives for drift (preferred)
    eps_window: float = 0.0          # S2 without a map: refused outside EPS_WINDOW when nonzero
    exit_hyst: float = 0.003         # S2: leave outside 1 + eps + h on two re-plans
    drift_past_deg: float = 3.0      # S2's D_k, S3's floor and measures; S1 keeps modes.REAR_PAST
    zones: str = "auto"              # S3: auto | path/to/zones.yaml
    straights: bool = True           # S3: manji on straights
    slip_floor_past_deg: float = 3.0  # S3: rear past the peak in a zone, at least
    speed_floor: float = 0.10        # S3: v >= (1 - this) * v_ref
    swing_rate_max: float = 2.0      # S3: rad/s in switch intervals
    width_use: float = 0.67          # S3: share of drivable width a manji may sweep
    classes: tuple = field(default=S3_CLASSES)   # S3: where a zone may be planned
    l2_slip_gain: float = 1.0        # S3: 3-5

    def __post_init__(self):
        object.__setattr__(self, "classes", tuple(self.classes))
        self.validate()

    # ---- loading ----

    @classmethod
    def from_dict(cls, d):
        """A record from a dict: {"style": {...}} or the inner dict itself; None or {} is the
        default (fastest). Unknown and invariant fields refused by name."""
        if d is None:
            return cls()
        if isinstance(d, Style):
            return d
        if not isinstance(d, dict):
            _refuse("style", f"a mapping, not {type(d).__name__}")
        if set(d) == {"style"}:
            d = d["style"]
            if d is None:
                return cls()
            if not isinstance(d, dict):
                _refuse("style", f"a mapping, not {type(d).__name__}")
        known = {f.name for f in fields(cls)}
        for k in d:
            if k in INVARIANTS or (isinstance(k, str) and k.lower() in INVARIANTS):
                _refuse(k, "an invariant, not a style field (design section 5: a style may "
                           "only tighten, and not through the record)")
            if k not in known:
                _refuse(k, f"not a style field (fields: {', '.join(sorted(known))})")
        return cls(**d)

    @classmethod
    def from_yaml(cls, path):
        with open(path) as fh:
            return cls.from_dict(yaml.safe_load(fh))

    def to_dict(self):
        d = {f.name: getattr(self, f.name) for f in fields(self)}
        d["classes"] = list(d["classes"])
        return d

    # ---- validation ----

    def validate(self, table=None):
        """Refuse a record that is not one (ValueError naming the field). With `table` (a
        track.SurfaceTable) the classes must be its own."""
        if self.name not in NAMES:
            _refuse("name", f"{self.name!r} is not a style ({' | '.join(NAMES)})")
        num = {"eps_lap_s", "eps_window", "exit_hyst", "drift_past_deg", "slip_floor_past_deg",
               "speed_floor", "swing_rate_max", "width_use", "l2_slip_gain"}
        for k in num:
            v = getattr(self, k)
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                _refuse(k, f"a finite number, not {v!r}")
        if not isinstance(self.straights, bool):
            _refuse("straights", f"true or false, not {self.straights!r}")
        if not isinstance(self.zones, str) or not self.zones:
            _refuse("zones", f"'auto' or a path, not {self.zones!r}")
        if self.eps_lap_s < 0:
            _refuse("eps_lap_s", f"{self.eps_lap_s} s < 0")
        if self.eps_window != 0 and not EPS_WINDOW[0] <= self.eps_window <= EPS_WINDOW[1]:
            _refuse("eps_window", f"{self.eps_window} outside [{EPS_WINDOW[0]}, {EPS_WINDOW[1]}] "
                                  "(below the 0.3 % solver noise the gate cannot price it)")
        if self.eps_window < 0:
            _refuse("eps_window", f"{self.eps_window} < 0")
        if self.exit_hyst < 0:
            _refuse("exit_hyst", f"{self.exit_hyst} < 0")
        if self.drift_past_deg <= 0:
            _refuse("drift_past_deg", f"{self.drift_past_deg} deg: a drift is past the peak, > 0")
        if self.slip_floor_past_deg <= 0:
            _refuse("slip_floor_past_deg", f"{self.slip_floor_past_deg} deg <= 0")
        if not 0 <= self.speed_floor < 1:
            _refuse("speed_floor", f"{self.speed_floor} outside [0, 1)")
        if self.swing_rate_max <= 0:
            _refuse("swing_rate_max", f"{self.swing_rate_max} rad/s <= 0")
        if not 0 < self.width_use <= 1:
            _refuse("width_use", f"{self.width_use} outside (0, 1] (edge margins only widen)")
        if self.l2_slip_gain <= 0:
            _refuse("l2_slip_gain", f"{self.l2_slip_gain} <= 0")
        for c in self.classes:
            if not isinstance(c, str):
                _refuse("classes", f"class names, not {c!r}")
            if c in NEVER:
                _refuse("classes", f"{c!r}: never a planned slide (roadmap 065 R5)")
            if table is not None and c not in table.mu_of:
                _refuse("classes", f"{c!r} is not a class of the surface table")
        if len(set(self.classes)) != len(self.classes):
            _refuse("classes", "a class named twice")
        owned = OWNERS[self.name]
        for f in fields(Style):
            if f.name == "name" or f.name in owned:
                continue
            d = f.default if f.default is not MISSING else f.default_factory()
            if getattr(self, f.name) != d:
                _refuse(f.name, f"set ({getattr(self, f.name)!r}) under style {self.name!r}, "
                                f"which does not read it (refused, never ignored)")
        return self

    def check_slip_floor(self, peaks, k_rear=RULE_REAR_K):
        """S3's floor against the rear slip cap (design 5; Driver.rear_slip_max, 3 x the peak on
        surfaces without fall-off): the floor, peak + slip_floor_past, must not pass the cap,
        k_rear x the peak, or the cap would be loosened to hold it. `peaks`: the rear tyre's
        peak (rad) on each such surface (a class name or a node to a peak); where none is
        given the rule is not computable and passes. Refuses 'slip_floor_past_deg'."""
        if self.name != "drift_max" or k_rear is None:
            return self
        past = math.radians(self.slip_floor_past_deg)
        items = peaks.items() if isinstance(peaks, dict) else enumerate(peaks)
        for where, p in items:
            if p + past > k_rear * p + 1e-12:
                _refuse("slip_floor_past_deg",
                        f"{self.slip_floor_past_deg} deg past a peak of {math.degrees(p):.2f} deg "
                        f"at {where} passes the rear cap, {k_rear:g} x the peak "
                        f"(at most {math.degrees((k_rear - 1) * p):.2f} deg there)")
        return self

    @property
    def implemented(self):
        return self.name in IMPLEMENTED

    def require_implemented(self):
        """Refuse a style the planner cannot run yet (roadmap 065 M1: S1 only)."""
        if not self.implemented:
            raise NotImplementedError(
                f"style {self.name!r} validates but is not built yet (roadmap 065: S2 at M3, "
                f"S3 at M5); the planner runs {', '.join(IMPLEMENTED)} only, and never runs a "
                "style it was not asked for")
        return self


FASTEST = Style()


def as_style(x):
    """A Style from a Style, a dict ({"style": {...}} or the inner one), a YAML path, or None
    (the default, fastest)."""
    if isinstance(x, Style):
        return x
    if isinstance(x, str):
        return Style.from_yaml(x)
    return Style.from_dict(x)
