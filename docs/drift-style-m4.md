# Drift style M4: the car's ECU and a planned slide

(Roadmap 065 M4, E4, DP3, 2026-11-01. Design: [drift style layer](superpowers/specs/2026-10-31-drift-style-layer-design.md);
M3: [drift-style-m3.md](drift-style-m3.md). Results under swb's `play_log/race_planner/style/m4/`.
sim.Plant, the ECU at sim_vehicle_ecu 760cd78, the superproject's checkout, `SIM_VEHICLE_ECU`
unset; `RP_DET=1`, three draws a run, `--jit` 0, 1e-9, 2e-9.)

**The drift car's ECU does not fight a planned slide; a road car's does, hard.** The drift car's
spec (`drift_mustang.yaml`) ships with ABS, traction control and ESC off, which is already the
"race profile" DP3 asked about. Its one remaining controller, the regen slip control (issues 009,
011), never acted in a planned slide: 0 events, and the runs with it switched off are the same to
the last digit. Switched on over the spec, traction control cuts the motor in the slide and ESC
brakes a front wheel against it: on the gravel hairpin the car's drift falls from 26 m to 7 m
(TC) or 15 m (ESC), the corner costs 0.6 to 4.9 s, and with ABS, TC and ESC together one draw in
three leaves the road. **DP3: no change** (builder's ruling, section 4).

## 1. Setup

Two hairpins where a slide is planned: wet mud and gravel, R 15 x 180 deg, 8 m wide, a 160 m
entry (`c:mud_wet:15:180:8:160`, `c:gravel:15:180:8:160`). On both the plans drift where the
car is in the corner (the free G and D slide: hairpins are drift gains in the live map). Two arms
make sure of it:

- **D forced** (`--arm always --force-drift`): every start every re-plan, and S1's gate adopts the
  fastest feasible drift plan whenever there is one, whatever it costs (an experiment's arm,
  `mode_choice_m4._force_drift`, never the planner's);
- **S2 forced** (`--arm sched --style drift_budget, eps_lap_s 1.0`): S2 with a budget that buys
  every corner, B (the floor 3 deg past the car's peak) live.

ECU profiles, the spec's calibration with fields set over it (`sim.Plant(ecu_overrides=)`, the
harness's `--ecu`):

| profile | ABS | TC | ESC | regen slip control |
|---|---|---|---|---|
| stock (the drift car's spec) | off | off | off | on |
| bare | off | off | off | off |
| tc | off | on | off | on |
| esc | off | off | on | on |
| road (the Model 3's set) | on | on | on | on |

TC and ESC at sim_vehicle_ecu's defaults (TC slip limit 0.25; ESC dead band 0.035 rad/s + 10 %,
sideslip limit 0.07 rad, 2000 N m a wheel), as the Model 3 has them.

What is counted (`mode_choice_m4._EcuObs`, observing only): at every ECU step TC's motor factor,
ESC's motor factor and wheel brake, the regen slip control's factor, ABS's; at every plant step
whether the adopted plan slides where the car is (its rear 3 deg or more past the planner's
peak) and the car's tracking of it (lateral offset, rear slip car minus plan). An event is an
intervention's start (a factor leaving 1, ESC's brake rising past 50 N m); "in a slide" counts
those that start in a planned-slide tick.

## 2. E4

Means over three draws (all three finished unless said), on autoware-drift 9619095 (issue 012's
line weight in; the runs before it are under `m4/pre012/` and read the same). D forced; S2
forced gave the same within 0.15 s and 0.5 m everywhere (on these hairpins both adopt the same
free slides), gravel road included (1 of 3 off road in both).

| course | ECU | time s | car's drift m | planned slide m | TC cuts (in a slide), s in slides | ESC brake events (in a slide), s in slides | ESC motor cut, s in slides | rear slip car - plan, mean / rms deg | lateral error in slides, rms m |
|---|---|---|---|---|---|---|---|---|---|
| mud hairpin | stock | 25.06 | 25.3 | 39.9 | 0 | 0 | 0 | -7.1 / 11.6 | 1.76 |
| | bare | 25.06 | 25.3 | 39.9 | 0 | 0 | 0 | -7.1 / 11.6 | 1.76 |
| | tc | 25.06 | 25.2 | 39.9 | 9 (0), 0 | 0 | 0 | -7.1 / 11.6 | 1.76 |
| | esc | 28.45 | 0 | 22.3 | 0 | 68 (10), 1.2 | 2.4 | -16.0 / 17.0 | 1.28 |
| | road | 27.04 | 0 | 18.2 | 15 (0), 0 | 83 (9), 1.0 | 1.8 | -19.4 / 19.5 | 1.66 |
| gravel hairpin | stock | 15.91 | 26.1 | 27.7 | 0 | 0 | 0 | -1.3 / 2.3 | 0.51 |
| | bare | 15.91 | 26.1 | 27.7 | 0 | 0 | 0 | -1.3 / 2.3 | 0.51 |
| | tc | 16.53 | 7.2 | 23.9 | 16 (6), 1.5 | 0 | 0 | -11.7 / 14.1 | 0.81 |
| | esc | 20.84 | 15.2 | 21.6 | 0 | 37 (5), 2.6 | 2.8 | -11.0 / 14.5 | 0.70 |
| | road | 19.03 (2 of 3; 1 off road) | 0 | 20.3 | 11 (0), 0 | 24 (0), 2.1 | 2.1 | -15.3 / 16.9 | 1.10 |

A planned slide here lasts 4.8 s (mud) and 2.5-2.8 s (gravel) with the stock car. The road
profile's ABS worked a wheel 9 (mud) and 5 (gravel) times a run, 20 fallback ticks on gravel. The
regen slip control started 0 times in a planned slide with the stock car and with TC; 0 to 1
under ESC, where ESC's own wheel brake makes a rear wheel slip under regen.

1. **Stock = bare**, the same in every number above: the regen slip control never acts in a
   planned slide (it reads a driven wheel slipping under regen; in a power slide the rear is
   driven, not braked), so it neither helps nor fights one.
2. **TC fights the power slide on gravel.** The planned slide is held on the throttle with the
   rear spinning past TC's 0.25 slip limit: TC cuts the motor 1.5 s of a 2.5 s slide, the rear
   falls 11.7 deg short of the plan, the drift falls from 26 to 7 m and the corner costs 0.6 s.
   On wet mud the slide is yaw more than wheelspin: TC acts only outside the slide, and changes
   nothing.
3. **ESC fights every slide.** It reads a planned slide as oversteer (yaw rate past its bicycle
   target, sideslip past 0.07 rad = 4 deg) and brakes the outer front wheel and cuts the motor:
   no drift left on mud (+3.4 s), 15 m on gravel (+4.9 s), the car 11-16 deg short of the plan's
   rear slip.
4. **All of it together** (the Model 3's set) drives no slide at all, and leaves the road once in
   three on gravel: the planner keeps asking for a slide the car refuses.

## 3. What the planner's side would need with a road ECU

Nothing the planner could do from its side: the stock car's planned slides reach a body sideslip
of 21-24 deg (gravel) and 26 deg (mud), the plans 25-26 and 30, against ESC's 4 deg limit, and on
a driven rear a slide is wheelspin past TC's limit. Either the ECU stands aside in a planned slide, or no slide is planned. The planner does not know the
car's ECU, and the drift styles assume the drift car's.

## 4. Decision point

**DP3** (builder's ruling, for the user): **no change.** The drift car's spec already turns ABS,
TC and ESC off, which is the race profile; its regen slip control does not touch a planned slide.
A race profile in sim_vehicle_ecu would duplicate the spec's own switches (`brakes.abs.enabled`,
`powertrain.traction_control.enabled`, no `brakes.esc`), and "ESC off in S3 zones only" switches
off something that is not on. Both stay open for a car whose spec turns TC or ESC on and is to
drive S2 or S3; then the zone switch (ESC and TC off in a planned slide's zone, on elsewhere)
would be the one to build, since the road profile's ESC is what keeps a road car on the road
outside a slide. Nothing changed in sim_vehicle_ecu; no pin moved.

## Rulings (builder's, M4)

1. **The spec is the ECU profile.** E4's profiles are the spec's calibration with fields set over
   it for the run (`sim.Plant(ecu_overrides=)`); the drift car runs its spec.
2. **A drift style needs the car's TC and ESC off.** With either on, S2 and S3 plan slides the car
   refuses (section 2); recorded here, not enforced in code (the planner does not see the ECU).
3. **Regen slip control stays on** with the drift styles: it never acts in a planned slide and is
   what keeps a braking rear on the low-friction roads (issues 009, 011).

## Reproduce

```
cd src/planning/race_planner
RP_DET=1 M4_OUT=<swb>/play_log/race_planner/style/m4 env -u PYTHONPATH .venv/bin/python \
    studies/mode_choice_m4.py run c:gravel:15:180:8:160 --arm always --force-drift \
    --ecu '{"tc_enabled": true}' --tag D_tc_j0 --cpus 20,21,22,23
```

(`--ecu` takes sim_vehicle_ecu Calibration fields as JSON and turns the observer on; `--ecu-obs`
turns it on with the spec's car.) Per run: `ecu_obs` in its JSON.

Video-ready (`--traj`, for `tools/arm_video.py`; each repeats its E4 run bit for bit):
`m4/video/` gravel hairpin D stock, tc, esc (24, 4.5, 14.5 m of drift), mud hairpin D stock, esc
(25, 0 m), draw j0.
