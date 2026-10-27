# autoware-drift

An Autoware 1.5.0 project for one experiment: a fast rear-wheel-drive car that Autoware
drives around CARLA Town05's closed ring highway (1.5 km, bends of ~100 m) at 100 km/h, and that drifts through every turn. A
drift controller takes the car at each turn's waypoint (trail-brake, power-over, a held
sideslip, exit) and hands it back to Autoware for the next straight. The car runs in NVIDIA
Isaac Sim through [scenario-world-bridge](https://github.com/NEWSLabNTU/scenario-world-bridge)
(swb), which plays the scenario (OpenSCENARIO, scenario_simulator_v2) on Isaac. The plan is
swb's roadmap 057.

**Status** (2026-10-27): the drift controller is tested offline against a single-track model
and in Isaac on swb's flat proving ground (`drift_mustang`, 100 km/h into a 100 m bend: -15.2
deg sideslip held 6 s on 97.8 m at 109 km/h, exit, hand back); the ROS node, the launch and the
Town05 ring scenario are written. The first Town05 lap with Autoware is next.

## Layout

| package | what |
|---|---|
| `src/control/drift_controller` | the drift sequence as a state machine (`law.py`, no ROS), the ROS node between Autoware's command gate and the car (`node.py`), the single-track test model (`model.py`), Town05's ring drift points (`config/town05_ring.yaml`) |
| `src/launcher/drift_launch` | `drift.launch.xml` (what this project adds to swb's Isaac ego stack), Autoware's velocity smoother with an 8 m/s^2 lateral limit, the Town05 ring scenario |

The car, `drift_mustang` (a 1966 Mustang drift build: 1450 kg, 50:50, 330 kW rear motor,
locked rear axle, 58 deg lock, no ABS/TC/ESC), is defined once in swb's
[sim_vehicle_spec](https://github.com/NEWSLabNTU/sim_vehicle_spec); its Autoware vehicle and
sensor kit packages (`drift_mustang_description`, `_launch`, `_sensor_kit_*`) are generated
from it in swb's `autoware_sim_vehicle`, its Isaac rig and ego config
(`ego_drift_mustang.yaml`) in swb's `autoware_isaac_bridge`.

## Run (in a scenario-world-bridge checkout, on a host that renders Town05 in Isaac)

```bash
# here
just build
# in scenario-world-bridge (this project's install as an overlay)
D=$HOME/repos/autoware-drift
just isaac-world ego:=$PWD/src/swb_isaac_world/deps/autoware_isaac_bridge/src/aib/config/ego_drift_mustang.yaml
just run sim:=isaac
just ego-isaac overlay:=$D/install/setup.bash map_path:=$CARLA_MAPS/Town05 \
    vehicle_model:=drift_mustang sensor_model:=drift_mustang_sensor_kit max_velocity:=27.78 \
    velocity_smoother_param_path:=$D/install/drift_launch/share/drift_launch/config/velocity_smoother.param.yaml \
    extra_launch:=$D/install/drift_launch/share/drift_launch/launch/drift.launch.xml \
    diagnostic_graph_path:=$D/install/drift_launch/share/drift_launch/config/diagnostics/drift.yaml \
    launch_lane_departure_checker:=false launch_autonomous_emergency_braking:=false \
    launch_control_validator:=false
just scenario $D/install/drift_launch/share/drift_launch/scenarios/town05_ring_lap.xosc sim:=isaac \
    vehicle_model:=drift_mustang initialize_duration:=300
```

Run at the default rates (60 Hz physics, SSv2's 20 Hz frames): at `physics_hz:=240` and
`frame_rate:=100` the simulation ran at ~0.2x real time on server-241, and Autoware's wall-clock
monitors never let it engage (lap 1-2, 2026-10-27); on the proving ground the drift was the same
at both rates.

The last four arguments are option 1 of [docs/autoware-in-a-drift.md](docs/autoware-in-a-drift.md):
Autoware tolerates a drift. Its diagnostic graph (`config/diagnostics/drift.yaml`) judges
autonomous mode without AEB, lane departure, the MPC's control state and the trajectory shift,
and those checkers are not launched. The controller hands the car back only within 1.0 m and
0.4 rad of Autoware's trajectory (its transition manager refuses past 1.5 m and 0.52 rad),
rejoining it first by pure pursuit, and aborts a drift when Autoware is in an MRM.

`/drift/state` reports each drift (bend, phase, speed, sideslip, yaw rate); the vehicle takes
its command from `/control/command/drift/control_cmd`, which the controller fills with
Autoware's own outside a drift.

## The drift sequence

CRUISE (Autoware drives) → BRAKE (a short trail-brake with the wheel turned in) → TURN_IN
(power-over until the sideslip passes 10°) → HOLD (steering on sideslip and yaw-rate error,
drive on sideslip error) → EXIT (the sideslip target ramps to zero) → CRUISE; RECOVER when
the sideslip passes 50°. See `law.py` for the laws and their sign conventions.

## Test

```bash
cd src/control/drift_controller && python3 -m pytest -q test
```

## License

Apache-2.0.
