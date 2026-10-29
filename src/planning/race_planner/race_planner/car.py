"""The car as the planner models it, read from sim_vehicle_spec's `vehicles/<car>.yaml` (spec v2).

Only what the spec says goes in; nothing is per track. Derived quantities:

- lf, lr: CoM to the front and rear axle (wheel_base split by com_front_fraction).
- Iz: yaw inertia of the spec's inertia box (length x width), m (l^2 + w^2) / 12.
- axle_torque_max: at the drive axle; `ev`: the motor's peak torque through its gear (to base
  speed, then power_max, the motor's peak power); `ideal`: max_drive_torque per driven wheel.
- width: the body's, wheel_tread plus the side overhangs (sim_vehicle_spec's rule).
- transfer_height: the load transfer's arm, the CoM's height over where the tyre forces act:
  h_cg, the contact patch (Isaac's too since aib abaf103, swb docs/issues/023).
- wheel_speed_max: the driven wheels' top peripheral speed, m/s: `ev`'s motor_max_rpm through its
  gear (None for `ideal`, which has no motor); so the car's top speed is the lower of it and
  where power meets drag.
- regen_torque_max (at the axle) and regen_power_max: what the motor regenerates under braking,
  within the rear's share (sim_vehicle_ecu); 0 for `ideal`.
- steer_rate_max: the steering actuator's rate limit, rad/s at the bicycle angle (the spec's
  `steering.rate_limit`); None when the spec sets none.
- brake_torque_max: per wheel; brake_distribution: the axles' split of a brake demand
  (sim_vehicle_ecu: static, the static axle loads; dynamic_axle_load, shifted with deceleration).
- drag: 1/2 rho Cd A (N per (m/s)^2); roll: the rolling coefficient.
- The tyre: `mu` (the spec's friction, relative to the reference surface: a surface's mu is
  this times its class's over the reference's, see track.SurfaceTable), PhysX's stiffnesses
  (lateral: lat_stiff_value N/rad per N of rest load, up to lat_stiff_max_load rest loads;
  longitudinal: long_stiff_value per g) and its friction-vs-slip curve. How these become a
  tyre model is model.py's business.
"""
from dataclasses import dataclass
import math

import yaml


@dataclass
class Car:
    name: str
    m: float
    Iz: float
    lf: float
    lr: float
    h_cg: float
    track: float
    width: float
    r_wheel: float
    steer_max: float
    mu: float
    lat_stiff_max_load: float
    lat_stiff_value: float
    long_stiff_value: float
    friction_vs_slip: list
    axle_torque_max: float
    power_max: float
    brake_torque_max: float
    drag: float
    roll: float
    diff: str
    drive_axle: str
    brake_distribution: str = "dynamic_axle_load"   # sim_vehicle_ecu's default
    transfer_height: float = None   # the tyre forces' arm about the CoM; None: h_cg (the ground)
    wheel_speed_max: float = None   # m/s at the driven wheels' rim; None: no motor limit
    regen_torque_max: float = 0.0   # N m at the axle
    regen_power_max: float = 0.0    # W
    steer_rate_max: float = None    # rad/s; None: the spec sets none

    def __post_init__(self):
        if self.transfer_height is None:
            self.transfer_height = self.h_cg

    @property
    def wheel_base(self):
        return self.lf + self.lr

    @classmethod
    def from_spec(cls, path):
        d = yaml.safe_load(open(path))
        g, i, t, p = d["geometry"], d["inertia"], d["tires"], d["powertrain"]
        wb = g["wheel_base"]
        lf = wb * (1.0 - i["com_front_fraction"])
        l, w = i["inertia_box"][0], i["inertia_box"][1]
        rw = float(g["wheel_radius"])
        if p["model"] == "ideal":  # `ideal`'s cap is per wheel (spec-format.md)
            n_driven = 4 if p["drive_axle"] == "all" else 2
            axle = n_driven * float(p["max_drive_torque"])
            wheel_v, regen_t, regen_p = None, 0.0, 0.0
        else:  # `ev`: the motor's envelope through its single gear
            axle = p["motor_peak_torque"] * p["gear_ratio"]
            wheel_v = p["motor_max_rpm"] * 2 * math.pi / 60 / p["gear_ratio"] * rw
            regen_t = float(p.get("regen_max_torque", 0.0)) * p["gear_ratio"]
            regen_p = float(p.get("regen_max_power", 0.0))
        rate = float(d.get("steering", {}).get("rate_limit", 0.0))
        r = d["resistance"]
        return cls(
            name=d["name"], m=float(i["mass"]), Iz=float(i["mass"]) * (l * l + w * w) / 12.0,
            lf=lf, lr=wb - lf, h_cg=float(i["com_height"]), track=float(g["wheel_tread"]),
            width=float(g["wheel_tread"] + g["left_overhang"] + g["right_overhang"]),
            r_wheel=float(g["wheel_radius"]), steer_max=float(g["max_steer_angle"]),
            mu=float(t["friction"]), lat_stiff_max_load=float(t["lat_stiff_max_load"]),
            lat_stiff_value=float(t["lat_stiff_value"]), long_stiff_value=float(t["long_stiff_value"]),
            friction_vs_slip=[list(map(float, q)) for q in t["friction_vs_slip"]],
            axle_torque_max=float(axle), power_max=float(p["motor_peak_power"]),
            brake_torque_max=float(d["brakes"]["max_brake_torque"]),
            drag=0.5 * r["air_density"] * r["drag_coefficient"] * r["frontal_area"],
            roll=float(r["rolling_coefficient"]),
            diff=p.get("differential", {}).get("type", "open"), drive_axle=p["drive_axle"],
            brake_distribution=d["brakes"].get("distribution", "dynamic_axle_load"),
            wheel_speed_max=wheel_v, regen_torque_max=regen_t, regen_power_max=regen_p,
            steer_rate_max=rate if rate > 0 else None)
