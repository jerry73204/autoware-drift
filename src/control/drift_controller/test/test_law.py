"""The drift sequence against the single-track model: a left bend entered at 100 km/h."""
import math

from drift_controller.law import DriftController, DriftPoint, Params, Phase, State
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


def test_repeat_takes_the_points_again():
    ctl = DriftController([DriftPoint(10.0, 0.0, 100.0)], Params(repeat=True))
    far = State(500.0, 500.0, 0.0, 27.8, 0.0, 0.0)
    near = State(10.0, 0.0, 0.0, 27.8, 0.0, 0.0)
    for lap in range(3):
        ctl.step(near, 0.01)
        assert ctl.phase is Phase.BRAKE, lap
        ctl.phase, ctl.active = Phase.CRUISE, None   # as after a handed-back drift
        ctl.step(far, 0.01)
    assert ctl.next_index == 3


def test_a_bend_reached_too_slowly_is_skipped_and_the_next_still_drifts():
    ctl = DriftController([DriftPoint(10.0, 0.0, 100.0, "a"), DriftPoint(200.0, 0.0, 100.0, "b")])
    ctl.step(State(10.0, 0.0, 0.0, 10.0, 0.0, 0.0), 0.01)
    assert ctl.phase is Phase.CRUISE and ctl.log[-1][0] == "skipped"
    ctl.step(State(200.0, 0.0, 0.0, 27.8, 0.0, 0.0), 0.01)
    assert ctl.phase is Phase.BRAKE and ctl.active.name == "b"


def test_an_mrm_aborts_a_drift_to_recover_and_arms_nothing():
    ctl = DriftController([DriftPoint(10.0, 0.0, 100.0), DriftPoint(300.0, 0.0, 100.0)])
    s = State(10.0, 0.0, 0.0, 27.8, -3.0, 0.3)
    ctl.step(s, 0.01)
    assert ctl.phase is Phase.BRAKE
    ctl.abort(s, "Autoware MRM")
    assert ctl.phase is Phase.RECOVER and ctl.log[-1][1] == "aborted: Autoware MRM"
    ctl.phase, ctl.armed = Phase.CRUISE, False
    ctl.step(State(300.0, 0.0, 0.0, 27.8, 0.0, 0.0), 0.01)
    assert ctl.phase is Phase.CRUISE


def test_off_the_path_after_the_exit_it_rejoins_before_handing_back():
    import dataclasses
    ctl = DriftController([DriftPoint(10.0, 0.0, 100.0)])
    s = State(10.0, 0.0, 0.0, 27.8, 0.0, 0.0)
    ctl.step(s, 0.01)
    ctl.phase, ctl.t_phase, ctl.exit_beta0 = Phase.EXIT, 5.0, 0.0   # an exit run out
    off = dataclasses.replace(s, path_lat=3.0, path_yaw=0.2, path_steer=-0.05, path_speed=27.8)
    cmd = ctl.step(off, 0.01)
    assert ctl.phase is Phase.REJOIN and cmd.override and cmd.steer == -0.05
    on = dataclasses.replace(off, path_lat=0.4, path_yaw=0.05)
    cmd = ctl.step(on, 0.01)
    assert ctl.phase is Phase.CRUISE and not cmd.override
    assert [p for p, _ in ctl.log][-2:] == ["rejoin", "cruise"]
