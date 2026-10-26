"""A single-track car with a driven, spinning rear axle, for testing the drift laws offline.

The tires are PhysX's kind (Isaac's vehicle SDK): force linear in slip, the combined force
capped at mu Fz, with an optional fall-off past the cap (`drop`, the share of grip lost when
the linear force is twice the cap), which is what makes a sliding axle let go. The rear axle
has its own wheel speed, driven by the motor's torque (power-limited) against the tire's
longitudinal force, so drive torque past the rear's grip spins it and takes its lateral
grip. The front axle only steers and brakes. Load moves between the axles with longitudinal
acceleration. ISO frame: x forward, y left, z up.
"""
import math
from dataclasses import dataclass, field

G = 9.81


@dataclass
class Car:
    mass: float = 1600.0
    iz: float = 2600.0
    a: float = 1.35
    b: float = 1.35
    h: float = 0.5
    wheel_radius: float = 0.35
    axle_inertia: float = 2.4           # kg m^2, both rear wheels and the driveline
    power: float = 330e3                # W at the wheels
    max_drive_torque: float = 6000.0    # N m at the rear axle
    brake_front: float = 0.6            # share of braking on the front axle
    mu_front: float = 1.0
    mu_rear: float = 0.9
    drop: float = 0.15
    c_lat: float = 17.0                 # cornering stiffness, N/rad per N of load
    c_long: float = 10.0                # N per unit slip, per N of load
    drag: float = 0.5 * 1.2 * 0.35 * 2.2  # N per (m/s)^2


@dataclass
class Sim:
    car: Car = field(default_factory=Car)
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 27.8
    vy: float = 0.0
    r: float = 0.0
    omega: float = None                  # rear axle speed, rad/s
    ax: float = 0.0

    def __post_init__(self):
        if self.omega is None:
            self.omega = self.vx / self.car.wheel_radius

    def _cap(self, fx, fy, mu, fz):
        cap = mu * max(fz, 0.0)
        f = math.hypot(fx, fy)
        if f <= cap or f == 0.0:
            return fx, fy
        over = min(1.0, f / cap - 1.0)
        k = cap * (1.0 - self.car.drop * over) / f
        return fx * k, fy * k

    def step(self, steer, accel, dt):
        c = self.car
        L = c.a + c.b
        fzf = c.mass * (G * c.b - self.ax * c.h) / L
        fzr = c.mass * (G * c.a + self.ax * c.h) / L
        vx = max(self.vx, 0.5)
        # front: steering, braking
        vxf = vx * math.cos(steer) + (self.vy + c.a * self.r) * math.sin(steer)
        vyf = -vx * math.sin(steer) + (self.vy + c.a * self.r) * math.cos(steer)
        fy_w = -c.c_lat * fzf * vyf / max(abs(vxf), 0.5)
        fx_w = min(0.0, c.mass * accel) * c.brake_front
        fx_w, fy_w = self._cap(fx_w, fy_w, c.mu_front, fzf)
        fxf = fx_w * math.cos(steer) - fy_w * math.sin(steer)
        fyf = fx_w * math.sin(steer) + fy_w * math.cos(steer)
        # rear: driven wheel
        vyr = self.vy - c.b * self.r
        slip = (self.omega * c.wheel_radius - vx) / max(abs(vx), 1.0)
        fxr = c.c_long * fzr * slip
        fyr = -c.c_lat * fzr * vyr / vx
        fxr, fyr = self._cap(fxr, fyr, c.mu_rear, fzr)
        if accel >= 0:
            torque = min(c.mass * accel * c.wheel_radius, c.max_drive_torque,
                         c.power / max(self.omega, 1.0))
        else:
            torque = c.mass * accel * (1 - c.brake_front) * c.wheel_radius
            torque = max(torque, -abs(self.omega) * 1e4)   # a brake does not reverse the wheel
        domega = (torque - c.wheel_radius * fxr) / c.axle_inertia
        self.omega = max(0.0, self.omega + domega * dt)
        drag = c.drag * self.vx * abs(self.vx)
        dvx = (fxf + fxr - drag) / c.mass + self.r * self.vy
        dvy = (fyf + fyr) / c.mass - self.r * self.vx
        dr = (c.a * fyf - c.b * fyr) / c.iz
        self.ax = dvx - self.r * self.vy
        self.vx += dvx * dt
        self.vy += dvy * dt
        self.r += dr * dt
        self.yaw += self.r * dt
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (self.vx * cy - self.vy * sy) * dt
        self.y += (self.vx * sy + self.vy * cy) * dt
