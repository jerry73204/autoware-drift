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
    EXIT      The sideslip target ramps to zero over `exit_time`; then the car is handed back to
              Autoware once sideslip and yaw-rate error are small and it is within the hand-back
              limits of Autoware's trajectory, else it goes to REJOIN.
    REJOIN    Pure pursuit of Autoware's trajectory at its speed until the car is within the
              hand-back limits (Autoware's transition manager refuses autonomous mode past
              1.5 m and 0.52 rad), then CRUISE; past `rejoin_timeout` handed back anyway.
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
    REJOIN = "rejoin"


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
    # drift_mustang (sim_vehicle_spec): wheel_base 2.886 m, com_front_fraction 0.50
    a: float = 1.443                    # m, CoM to the front axle
    b: float = 1.443                    # m, CoM to the rear axle
    max_steer: float = math.radians(58)
    repeat: bool = False                # the points again after the last: laps of a closed track
    trigger_distance: float = 5.0       # m from the drift point at which the sequence starts
    min_entry_speed: float = 27.0       # m/s: a steady drift's radius is ~v^2 / a_y, so a 104 m
                                        # bend wants ~28 m/s (docs/ring-envelope.md: 28.6 for 20
                                        # deg); lap 27 entered at 25.6 and circled 70 m
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
    # The hold's line: the yaw-rate target v / R less this much per m of lateral error to
    # Autoware's trajectory (+ left) and per rad of course error (heading error + sideslip), so a
    # drift that cuts inside the bend is turned out (lap 27 circled 70 m in a 104 m bend), after
    # Goh, Goel & Gerdes 2019, drifting along a path. Clamped to path_r_max.
    k_path: float = 0.02                # rad/s per m
    k_course: float = 0.3               # rad/s per rad
    path_r_max: float = 0.15            # rad/s
    # ... and the sideslip target less this much per m inside the line (more outside), within
    # [beta_min, beta_max]: an unreachable target fights the yaw-rate term and wins (lap 27)
    k_path_beta: float = 0.0            # rad per m (off: on the model it took the drift away, lap 27 notes)
    beta_min: float = math.radians(5)
    beta_max: float = math.radians(25)
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
    handback_lat: float = 1.0            # m from Autoware's trajectory (its limit is 1.5)
    handback_yaw: float = 0.4            # rad (its limit is 0.52)
    rejoin_timeout: float = 10.0         # s
    k_rejoin_speed: float = 1.0          # m/s^2 per m/s
    rejoin_accel_max: float = 1.0        # m/s^2: a locked rear axle under full drive spins a car
                                         # that has just come out of a slide (lap 17)
    k_rejoin_damp: float = 0.1           # rad per rad/s of yaw rate off the pursuit's
    recover_yaw_error: float = 0.4       # rad/s: recovered only below it (a 105 m bend at 28 m/s
                                         # turns at 0.27)


@dataclass
class State:
    """What the controller measures: speed, sideslip, yaw rate, pose."""
    x: float
    y: float
    yaw: float
    vx: float               # m/s, body frame
    vy: float
    r: float                # rad/s

    # Autoware's trajectory as the node sees it (None: no trajectory): the car's lateral and
    # heading error from its nearest point, a pure-pursuit tire angle onto it, and its speed there.
    path_lat: float = None
    path_yaw: float = None
    path_steer: float = None
    path_speed: float = None

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
    rejoin_speed: float = 0.0
    log: list = field(default_factory=list)
    armed: bool = True                  # False: no new drift starts (Autoware in an MRM)

    def _go(self, phase, s, why=""):
        self.log.append((phase.value, why))
        self.phase, self.t_phase = phase, 0.0
        if phase is Phase.HOLD:
            self.yaw0, self.integral, self.hold_speed = s.yaw, 0.0, s.speed
        if phase is Phase.EXIT:
            self.exit_beta0 = s.beta
        if phase is Phase.REJOIN:
            self.rejoin_speed = s.speed

    def abort(self, s, why):
        """Leave a drift for RECOVER (Autoware's MRM, say); CRUISE and RECOVER stay."""
        if self.phase not in (Phase.CRUISE, Phase.RECOVER):
            self._go(Phase.RECOVER, s, f"aborted: {why}")

    def _on_path(self, s):
        """Within the hand-back limits of Autoware's trajectory (no trajectory: no limit)."""
        p = self.p
        return s.path_lat is None or (abs(s.path_lat) < p.handback_lat
                                      and abs(s.path_yaw) < p.handback_yaw)

    def _hand_back(self, s, why):
        if self._on_path(s):
            self._go(Phase.CRUISE, s, why)
            self.active = None
            return Command(0.0, 0.0, False)
        self._go(Phase.REJOIN, s, f"{why}, {s.path_lat:+.1f} m {math.degrees(s.path_yaw):+.0f} deg off")
        return self.step(s, 0.0)

    def _sign(self):
        return 1.0 if self.active.radius > 0 else -1.0

    def _drive(self, s, target, dt):
        """The hold's drive law on a sideslip target's magnitude, with its integral."""
        p = self.p
        err = target - abs(s.beta)
        self.integral = max(-0.3, min(0.3, self.integral + err * dt))
        accel = (p.accel_hold + p.k_drive * err + p.k_drive_i * self.integral
                 - p.k_speed * max(0.0, s.speed - self.hold_speed))
        return max(p.accel_min, min(p.accel_max, accel))

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
            if self.armed and self.points and (self.p.repeat or self.next_index < len(self.points)):
                w = self.points[self.next_index % len(self.points)]
                if math.hypot(w.x - s.x, w.y - s.y) < p.trigger_distance:
                    self.next_index += 1
                    if s.speed < p.min_entry_speed:  # too slow to drift: the next bend's turn
                        self.log.append(("skipped", f"{w.name} at {s.speed:.1f} m/s"))
                        return Command(0.0, 0.0, False)
                    self.active = w
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
        r_ref = s.speed / self.active.radius if self.active else 0.0
        if s.path_lat is not None and ph in (Phase.HOLD, Phase.EXIT):
            course = math.remainder(s.path_yaw + s.beta, math.tau)
            fix = -p.k_path * s.path_lat - p.k_course * course
            r_ref += max(-p.path_r_max, min(p.path_r_max, fix))
        if ph is Phase.HOLD:
            turned = abs(math.remainder(s.yaw - self.yaw0, math.tau))
            limit = (self.active.angle - p.exit_lead) if self.active.angle else p.hold_angle
            if self.t_phase >= p.hold_time or turned >= limit:
                self._go(Phase.EXIT, s, f"held {self.t_phase:.1f} s, {math.degrees(turned):.0f} deg")
                return self.step(s, 0.0)
            target = p.target_sideslip
            if s.path_lat is not None:  # + inside a left bend, so sign * lat is inside either way
                target = max(p.beta_min, min(p.beta_max, target - p.k_path_beta * sign * s.path_lat))
            beta_ref = -sign * target
            # drive holds the sideslip; past the speed the hold began at it is taken back, or a
            # power-on drift gains speed and widens its line
            return Command(self._hold_steer(s, beta_ref, r_ref),
                           self._drive(s, target, dt), True)
        if ph is Phase.EXIT:
            k = max(0.0, 1.0 - self.t_phase / p.exit_time)
            beta_ref = self.exit_beta0 * k
            if (k == 0.0 and abs(s.beta) < p.exit_sideslip
                    and abs(s.r - r_ref) < p.exit_yaw_error) or self.t_phase > 4 * p.exit_time:
                return self._hand_back(s, "handed back")
            # The drive follows the sideslip target down rather than stopping: cut at once, the
            # rear grips while the front is still countersteered and the car snaps the other
            # way (lap 17: -2.0 rad/s).
            return Command(self._hold_steer(s, beta_ref, r_ref),
                           self._drive(s, abs(beta_ref), dt), True)
        if ph is Phase.REJOIN:
            if self._on_path(s) or self.t_phase > p.rejoin_timeout or s.path_steer is None:
                why = "rejoined" if self._on_path(s) else "rejoin timed out"
                self._go(Phase.CRUISE, s, why)
                self.active = None
                return Command(0.0, 0.0, False)
            # Gently: no faster than the car came in, little drive, and the pursuit's steering
            # damped by the yaw rate it should have (lap 17 spun in REJOIN at full drive).
            target = min(s.path_speed if s.path_speed is not None else s.speed, self.rejoin_speed)
            accel = max(p.accel_min, min(p.rejoin_accel_max, p.k_rejoin_speed * (target - s.speed)))
            v = max(s.speed, 1.0)
            r_path = v * math.tan(s.path_steer) / (p.a + p.b)
            steer = s.path_steer - p.k_rejoin_damp * (s.r - r_path)
            return Command(max(-p.max_steer, min(p.max_steer, steer)), accel, True)
        if ph is Phase.RECOVER:
            if (abs(s.beta) < p.recover_sideslip and self.t_phase > 0.5
                    and abs(s.r) < p.recover_yaw_error):
                return self._hand_back(s, "recovered")
            steer = max(-p.max_steer, min(p.max_steer, s.beta))
            return Command(steer, p.recover_accel, True)
        raise AssertionError(ph)
