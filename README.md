# autoware-drift

An Autoware 1.5.0 project for one experiment: a fast rear-wheel-drive car that Autoware
drives around CARLA Town05's closed ring highway (1.5 km, bends of ~100 m) at 100 km/h, and that drifts through every turn. A
drift controller takes the car at each turn's waypoint (trail-brake, power-over, a held
sideslip, exit) and hands it back to Autoware for the next straight. The car runs in NVIDIA
Isaac Sim through [scenario-world-bridge](https://github.com/NEWSLabNTU/scenario-world-bridge)
(swb), which plays the scenario (OpenSCENARIO, scenario_simulator_v2) on Isaac. The plan is
swb's roadmap 057.

**Status**: started 2026-10-27. The drift controller exists and is tested offline against a
single-track model (on a 100 m bend: -18 deg sideslip held on a 103 m radius at 103 km/h); the car, the launch and the Isaac runs are to come.

## Layout

| package | what |
|---|---|
| `src/control/drift_controller` | the drift sequence as a state machine (`law.py`, no ROS), the single-track test model (`model.py`), and its tests |
| `src/launcher/` | (to come) the main launch: Autoware on `/opt/autoware/1.5.0`, the car, the drift controller between Autoware's control command and the vehicle |
| `src/vehicle/`, `src/sensor_kit/` | (to come) the car's Autoware packages, generated from its vehicle spec |

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
