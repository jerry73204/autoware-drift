"""The drift sequence against the single-track model: a left bend entered at 100 km/h."""
import math

from drift_controller.law import DriftController, DriftPoint, Phase, State
from drift_controller.model import Sim


def run(radius, seconds=16.0):
    sim = Sim(vx=27.8)
    ctl = DriftController([DriftPoint(60.0, 0.0, radius, "bend")])
    dt, ctl_dt, t, next_ctl, cmd = 0.001, 0.01, 0.0, 0.0, None
    trace = []
    while t < seconds:
        if t >= next_ctl:
            s = State(sim.x, sim.y, sim.yaw, sim.vx, sim.vy, sim.r)
            cmd = ctl.step(s, ctl_dt)
            if not cmd.override:  # Autoware's stand-in: hold 100 km/h, steer along the velocity
                cmd.steer = s.beta + 1.35 * s.r / max(s.speed, 1.0)
                cmd.accel = 27.8 - s.speed
            trace.append((t, ctl.phase, s.beta, s.speed / s.r if abs(s.r) > 1e-3 else 0.0))
            next_ctl += ctl_dt
        sim.step(cmd.steer, cmd.accel, dt)
        t += dt
    return ctl, trace


def test_a_left_bend_is_drifted_held_and_handed_back():
    ctl, trace = run(74.0)
    phases = [p for p, _ in ctl.log]
    assert phases == ["brake", "turn_in", "hold", "exit", "cruise"], ctl.log
    held = [(b, rad) for t, p, b, rad in trace if p is Phase.HOLD]
    settled = [b for b, _ in held[len(held) // 3:]]
    radius = sorted(rad for _, rad in held[len(held) // 3:])[len(held) // 3]
    assert 60 < radius < 95, radius   # the drift follows the 74 m bend
    assert all(b < 0 for b in settled)  # a left drift slides to the right
    mean = sum(settled) / len(settled)
    assert abs(math.degrees(mean) + 20) < 5, math.degrees(mean)
    assert abs(trace[-1][2]) < math.radians(3)


def test_a_right_bend_mirrors_it():
    ctl, trace = run(-74.0)
    held = [b for t, p, b, _ in trace if p is Phase.HOLD]
    assert held and all(b > 0 for b in held[len(held) // 3:])


def test_a_90_degree_bend_ends_its_hold_short_of_the_turn():
    sim = Sim(vx=27.8)
    ctl = DriftController([DriftPoint(60.0, 0.0, 104.0, "bend", math.radians(90))])
    dt, t, next_ctl, cmd, yaw_at = 0.001, 0.0, 0.0, None, {}
    while t < 16.0:
        if t >= next_ctl:
            s = State(sim.x, sim.y, sim.yaw, sim.vx, sim.vy, sim.r)
            cmd = ctl.step(s, 0.01)
            if not cmd.override:
                cmd.steer = s.beta + 1.35 * s.r / max(s.speed, 1.0)
                cmd.accel = 27.8 - s.speed
            yaw_at.setdefault(ctl.phase, s.yaw)
            next_ctl += 0.01
        sim.step(cmd.steer, cmd.accel, dt)
        t += dt
    assert [p for p, _ in ctl.log] == ["brake", "turn_in", "hold", "exit", "cruise"], ctl.log
    held = math.degrees(yaw_at[Phase.EXIT] - yaw_at[Phase.HOLD])
    assert 55 < held < 75, held   # 90 deg less the exit's 20, from where the hold began
