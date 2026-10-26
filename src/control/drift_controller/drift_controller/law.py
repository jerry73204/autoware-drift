"""The drift sequence: a state machine and its control laws, free of ROS.

A drift is taken at a waypoint on a turn of known radius and direction. The machine:

    CRUISE    Autoware drives; nothing is overridden. A drift is armed when the car comes
              within `trigger_distance` of the next drift waypoint at `min_entry_speed` or more.
    BRAKE     Trail-brake: a short deceleration with the wheel turned in, which loads the
              front axle and unloads the rear.
    TURN_IN   Power-over: full drive torque with the turn-in angle held, until the sideslip
              passes `enter_sideslip` (or `turn_in_timeout` passes: the drift is abandoned).
    HOLD      The drift: steering on sideslip and yaw-rate error, drive on sideslip error,
              for `hold_time` or `hold_angle` of heading change, whichever comes first.
    EXIT      The sideslip target ramps to zero over `exit_time`; the car is handed back to
              Autoware once sideslip and yaw-rate error are small.
    RECOVER   Sideslip or yaw rate past its limit (a spin starting): countersteer along the
              velocity, brake gently, until stable; then CRUISE.

Conventions: ISO vehicle frame (x forward, y left, z up). Sideslip beta = atan2(v_y, v_x),
positive when the car slides to its left. A left turn has radius R > 0 and yaw rate r > 0;
in a left drift the car points into the turn more than it travels, so its velocity points
to the right of its heading: beta < 0. Steering angle delta > 0 turns left.

The hold laws follow the front axle's slip, alpha_f = beta + a r / v - delta (positive slip
gives a force to the right, -C alpha_f to the left):

    delta = beta + a r / v + k_beta (beta - beta*) - k_r (r - r*)

The first two terms keep the front tire near zero slip (it steers the car's nose where the
front axle travels); turning further in (delta up) adds a left force at the front, a yaw
moment to the left. Too much sideslip into the turn (beta below beta* in a left drift)
calls for less turn-in, too much yaw rate likewise. Drive torque sets the rear axle's slip,
which takes lateral grip from the rear: more drive, more sideslip.

    accel = accel_hold + k_drive (|beta*| - |beta|) + integral - k_speed (v - v_hold)+,
    kept within the limits (v_hold: the speed the hold began at)

(Goh, Goel & Gerdes 2019 control rear wheel speed instead; a vehicle bridge that publishes
it can replace the drive law with that.)
"""
import math
from dataclasses import dataclass, field
from enum import Enum


class Phase(Enum):
    CRUISE = "cruise"
    BRAKE = "brake"
    TURN_IN = "turn_in"
    HOLD = "hold"
    EXIT = "exit"
    RECOVER = "recover"


@dataclass
class DriftPoint:
    """Where a drift starts, and the turn it is taken on."""
    x: float
    y: float
    radius: float           # m, > 0 left turn, < 0 right turn
    name: str = ""
    angle: float = 0.0      # rad, the bend's turn; the hold ends short of it (0: Params.hold_angle)


@dataclass
class Params:
    a: float = 1.35                     # m, CoM to the front axle
    b: float = 1.35                     # m, CoM to the rear axle
    max_steer: float = math.radians(58)
    repeat: bool = False                # the points again after the last: laps of a closed track
    trigger_distance: float = 5.0       # m from the drift point at which the sequence starts
    min_entry_speed: float = 20.0       # m/s
    brake_accel: float = -2.0           # m/s^2, the trail-brake
    brake_time: float = 0.3             # s
    turn_in_factor: float = 2.0         # x the kinematic angle wheelbase / R
    turn_in_accel: float = 8.0          # m/s^2: more than the rear can take, so it spins
    turn_in_timeout: float = 2.5        # s
    enter_sideslip: float = math.radians(10)
    target_sideslip: float = math.radians(20)   # magnitude; its sign follows the turn
    hold_time: float = 6.0              # s
    hold_angle: float = math.radians(170)        # heading change in the drift, unless the point gives its bend
    exit_lead: float = math.radians(20)          # a bend's hold ends this short of its turn, for the exit
    k_beta: float = 1.5
    k_r: float = 1.0                    # rad per rad/s
    accel_hold: float = 0.0             # m/s^2: drive past the drag spins the rear and widens the line
    k_drive: float = 20.0               # m/s^2 per rad
    k_drive_i: float = 10.0             # m/s^2 per rad s
    k_speed: float = 1.5                # m/s^2 per m/s over the speed the hold began at
    accel_min: float = -3.0
    accel_max: float = 9.0
    exit_time: float = 1.0              # s
    exit_sideslip: float = math.radians(3)
    exit_yaw_error: float = 0.15        # rad/s
    max_sideslip: float = math.radians(50)
    recover_accel: float = -2.0
    recover_sideslip: float = math.radians(4)


@dataclass
class State:
    """What the controller measures: speed, sideslip, yaw rate, pose."""
    x: float
    y: float
    yaw: float
    vx: float               # m/s, body frame
    vy: float
    r: float                # rad/s

    @property
    def speed(self):
        return math.hypot(self.vx, self.vy)

    @property
    def beta(self):
        return math.atan2(self.vy, max(self.vx, 0.1))


@dataclass
class Command:
    steer: float            # rad, front tire angle
    accel: float            # m/s^2 requested of the vehicle bridge
    override: bool          # False: pass Autoware's command through


@dataclass
class DriftController:
    points: list
    p: Params = field(default_factory=Params)
    phase: Phase = Phase.CRUISE
    t_phase: float = 0.0
    active: DriftPoint = None
    next_index: int = 0
    yaw0: float = 0.0
    integral: float = 0.0
    exit_beta0: float = 0.0
    hold_speed: float = 0.0
    log: list = field(default_factory=list)

    def _go(self, phase, s, why=""):
        self.log.append((phase.value, why))
        self.phase, self.t_phase = phase, 0.0
        if phase is Phase.HOLD:
            self.yaw0, self.integral, self.hold_speed = s.yaw, 0.0, s.speed
        if phase is Phase.EXIT:
            self.exit_beta0 = s.beta

    def _sign(self):
        return 1.0 if self.active.radius > 0 else -1.0

    def _hold_steer(self, s, beta_ref, r_ref):
        p = self.p
        v = max(s.speed, 1.0)
        d = s.beta + p.a * s.r / v + p.k_beta * (s.beta - beta_ref) - p.k_r * (s.r - r_ref)
        return max(-p.max_steer, min(p.max_steer, d))

    def step(self, s: State, dt: float) -> Command:
        p = self.p
        self.t_phase += dt
        if self.phase not in (Phase.CRUISE, Phase.RECOVER) and abs(s.beta) > p.max_sideslip:
            self._go(Phase.RECOVER, s, f"sideslip {math.degrees(s.beta):.0f} deg")
        ph = self.phase
        if ph is Phase.CRUISE:
            if self.points and (self.p.repeat or self.next_index < len(self.points)):
                w = self.points[self.next_index % len(self.points)]
                if (math.hypot(w.x - s.x, w.y - s.y) < p.trigger_distance
                        and s.speed >= p.min_entry_speed):
                    self.active = w
                    self.next_index += 1
                    self._go(Phase.BRAKE, s, w.name)
                    return self.step(s, 0.0)
            return Command(0.0, 0.0, False)
        sign = self._sign()
        wheelbase = p.a + p.b
        turn_in = sign * min(p.max_steer, p.turn_in_factor * wheelbase / abs(self.active.radius))
        if ph is Phase.BRAKE:
            if self.t_phase >= p.brake_time:
                self._go(Phase.TURN_IN, s)
                return self.step(s, 0.0)
            return Command(turn_in, p.brake_accel, True)
        if ph is Phase.TURN_IN:
            if -sign * s.beta >= p.enter_sideslip:
                self._go(Phase.HOLD, s)
                return self.step(s, 0.0)
            if self.t_phase >= p.turn_in_timeout:
                self._go(Phase.EXIT, s, "no sideslip")
                return self.step(s, 0.0)
            return Command(turn_in, p.turn_in_accel, True)
        r_ref = s.speed / self.active.radius
        if ph is Phase.HOLD:
            turned = abs(math.remainder(s.yaw - self.yaw0, math.tau))
            limit = (self.active.angle - p.exit_lead) if self.active.angle else p.hold_angle
            if self.t_phase >= p.hold_time or turned >= limit:
                self._go(Phase.EXIT, s, f"held {self.t_phase:.1f} s, {math.degrees(turned):.0f} deg")
                return self.step(s, 0.0)
            beta_ref = -sign * p.target_sideslip
            err = p.target_sideslip - abs(s.beta)
            self.integral = max(-0.3, min(0.3, self.integral + err * dt))
            # drive holds the sideslip; past the speed the hold began at it is taken back, or a
            # power-on drift gains speed and widens its line
            accel = (p.accel_hold + p.k_drive * err + p.k_drive_i * self.integral
                     - p.k_speed * max(0.0, s.speed - self.hold_speed))
            return Command(self._hold_steer(s, beta_ref, r_ref),
                           max(p.accel_min, min(p.accel_max, accel)), True)
        if ph is Phase.EXIT:
            k = max(0.0, 1.0 - self.t_phase / p.exit_time)
            beta_ref = self.exit_beta0 * k
            if (k == 0.0 and abs(s.beta) < p.exit_sideslip
                    and abs(s.r - r_ref) < p.exit_yaw_error) or self.t_phase > 4 * p.exit_time:
                self._go(Phase.CRUISE, s, "handed back")
                self.active = None
                return Command(0.0, 0.0, False)
            return Command(self._hold_steer(s, beta_ref, r_ref), p.accel_hold * k, True)
        if ph is Phase.RECOVER:
            if abs(s.beta) < p.recover_sideslip and self.t_phase > 0.5:
                self._go(Phase.CRUISE, s, "recovered")
                self.active = None
                return Command(0.0, 0.0, False)
            steer = max(-p.max_steer, min(p.max_steer, s.beta))
            return Command(steer, p.recover_accel, True)
        raise AssertionError(ph)
