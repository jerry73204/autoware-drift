# 011: with regen slip control, the car leaves wet-mud corners S1 used to finish

**Severity:** high (a car change makes the fastest-lap style leave the road on two wet-mud test
corners it finished before; the planner is not involved). **Status:** fixed in the car,
sim_vehicle_ecu `760cd78` (aib `f298a93`, swb_isaac_world `13ed8b6`); see "Root cause" and "The
fix".

## What is wrong

sim_vehicle_ecu `f3f5dcf` (regen slip control, issue 009) changes the car on wet mud, not only
on ice. With it, S1 leaves the road on two wet-mud corners that the previous car (`d68637a`)
finishes. 009's check of the other surfaces (wet mud 33/36, same as the old car) did not
include these corners.

## What was measured

`RP_DET=1`, `--arm sched`, no style record, one run each; the planner is the same in each pair,
only the car's ECU differs (`SWB` pointing at a tree whose `src/sim_vehicle_ecu` is the named
commit):

| course | planner | ECU d68637a | ECU f3f5dcf |
|---|---|---|---|
| `c:mud_wet:25:90:8` | 0a624f2 | done 12.96 s | done 12.96 s |
| `c:mud_wet:25:90:8` | 474a9bf | done 12.96 s | done 12.96 s |
| `c:mud_wet:25:90:8:100` | da6e230 | done 15.63 s | **off road** |
| `c:mud_wet:15:90:8:160` | da6e230 | done 20.03 s | **off road** |

So the planner change of 009 (`0c61d28`) is not the cause: the same planner finishes with the
old car and leaves the road with the new one. Corners with an entry straight (100 m, 160 m) are
hit; the 5-field corner (its default entry) is not.

Roadmap 065 M3 first saw this as an S1 regression "since the rebase onto 474a9bf": the
superproject's `src/sim_vehicle_ecu` checkout moved to `f3f5dcf` during its runs, so its runs
after the rebase drove the new car.

## Where to look

- Which part of the regen slip control fires on mud: the cut on slip, the 0.25 release, the
  runaway-wheel test, or the hold (0.7 x found grip). A longer entry means more speed and a
  harder first brake.
- Whether the planner should model the hold on wet mud after all, or the ECU should not act
  there (009 found the planner's hold made snow and mud plans leave the 6 m roads).

Outputs: swb scratch runs reproduced with the table's commands; see roadmap 061 for the entry.

## Root cause

Traced (the plant's ECU each step: the regen slip control's modulator, its slip event, the axle's
grip found and the cap; scripts in swb `play_log/race_planner/mode_choice/i011/tools/`). Two
parts of the control fire on wet mud, both on a grip that was never lost:

- **The 0.25 release, on a slip alone.** Wet mud's tyre peaks at 0.20 slip and keeps its force
  past the peak (sliding share 1.0). In `c:mud_wet:25:90:8:100` the plan brakes the rear in a
  braking drift at -1.33 m/s^2 on the straight, 1600 N of regen; the rear's slip creeps to 0.25
  and holds at 0.30, the wheel 0.27 g faster than the car, the axle carrying 0.92 of what the
  brakes put on it. At s 78 (4.5 s) the release fires (no runaway: the deficit test is not
  met), cuts regen to 0.67 for 0.3 s, and the event's grip, 0.23 (what the rear was carrying),
  becomes the axle's friction found: from then on regen beyond the rear's share is held at
  0.7 x 0.23, 1096 N against the plan's 1600, the rest on the front's friction brakes, for the
  rest of the drive (forgotten at 0.001 /s). The car brakes the front where the plan braked
  the rear and runs wide; the old car follows the plan from there and finishes. Without the
  release on a slip alone (runaway only) both corners finish (15.76 s, 20.05 s); without the
  hold (grip share 100) the 160 m corner still leaves the road.
- **The runaway test out of a wheelspin.** Out of a power slide (the rear 0.1-0.47 faster than
  the road) into regen, the rear crosses the road's speed slowing 4-7 g faster than the car,
  while the car's deceleration still shows the tyre's drive of a moment ago, so the axle reads
  30-100 % short of its brakes: a runaway. The grip is then measured from the wheels' balance
  while their inertia gives up most of the torque, and held at the floor (0.10-0.13; wet
  mud's is 0.29). With the release fixed, 7 of 12 M4g wet-mud corners still had such an event
  at 170 iterations, every one of them out of a wheelspin.

Ice is not affected by either: its events are runaways from a rolling wheel (the axle 0.68 of
its brakes, the wheel 3 g faster), and the priming finds the same grip (0.2313, 0.2336, 0.2297
on three ice corners, before and after).

## The fix (the car)

sim_vehicle_ecu `760cd78` (README "Regen slip control"; calibration defaults, spec untouched):

- past the release slip a release starts only on a lost grip: the axle 30 % short of its brakes
  (`regen_runaway_deficit`, the runaway's), or the wheel still slowing 1 g faster than the car
  (`regen_slip_growth`; a road that keeps its force past the peak still locks a wheel the
  brakes ask more of than it carries: a quarter of asphalt's friction, 15 % short, the wheel
  2 g faster, planner test `test_sim`). Once released, the slip alone, as ABS;
- neither test is read until the driven wheels have been out of a drive slip for 0.1 s
  (`regen_spin_settle`).

Car-side because the car did something wrong: it cut and held regen on an axle that had not
lost its grip, from its own misreading (a slip on a late-peaking tyre, a lagging deceleration).
The planner's model (no hold on mud, issue 009's ruling) was right.

## Measured

Three cars on one planner (`446b403`; `RP_DET`): `o` sim_vehicle_ecu d68637a (before 009), `n`
f3f5dcf (009), `f` the fix. Results swb `play_log/race_planner/mode_choice/i011/` (`det/`,
`digest/`, `agg.txt`, `tools/agg.py`; `det_fixB/` a first version without the growth test,
superseded).

| | old `o` | 009 `n` | fix `f` |
|---|---|---|---|
| `c:mud_wet:25:90:8:100` S1, 5 budgets x 4 draws | 20 / 20 | 1 / 20 | **20 / 20** |
| `c:mud_wet:15:90:8:160` S1, 5 budgets x 4 draws | 11 / 20 | 1 / 20 | **11 / 20** |
| the same, no budget, 10 draws | 10 / 10, 5 / 10 | 0 / 10, 0 / 10 | **10 / 10, 5 / 10** |
| ice single (12 a replicate, 20) | 0.00 (see below) | 10.90 | **10.95** (+0.05, se 0.23: 0.2 se) |
| ice sched (10) | 0.00 | 10.30 | **10.30** (+0.00) |
| packed snow, single 150/170/200 | 36 / 36 | 36 / 36 | 36 / 36, all 36 bit for bit as `o` |
| loose snow | 36 / 36 | 36 / 36 (-0.40 s) | 36 / 36 (-0.46 s; 14 as `o`) |
| wet mud | 33 / 36 | 33 / 36 | 33 / 36 (+0.005 s; 31 as `o`) |

- The 160 m corner is a coin flip on the old car too (5 of 10 draws without a budget, the
  fix's runs the same draws and times, 20.87-20.92 s); the fix gives the old car back there.
  The 25 m corner finishes in every run, as on the old car.
- Ice: 334 of 360 runs bit for bit as `n`, 3 outcomes changed; per corner the same as `n`
  but R 9 x 180 at 6 m single 9 of 20 (8). 009's ice gain kept.
- The old car on ice with today's planner finishes nothing: the planner holds regen on ice
  (issue 009) and that car does not, issue 009's "L1 told regen stops at the rear's grip, the
  car not". Not a baseline; M4e's 9.55 was the old car with the old planner.
- The wet-mud loss (R 15 x 180 at 6 m, all three cars) is issue 012's.
- Haruna / Tsukuba `RP_DET` single: 76.28 / 74.33 s, plan and plant digests 9524862c /
  9160b94d and d7ca9f95 / 82bd3c28, equal to M4g's.
- Tests: sim_vehicle_ecu 100 passed (4 new: a slip whose axle carries its brakes, a wheel down
  from a wheelspin, ice out of a wheelspin, a lock-up on a road that keeps its force); aib 60
  passed, generated files up to date; planner suite with the fix **205 passed** (load 9-19;
  the first version, without the growth test, failed `test_sim`'s slick-road test).

## Left

- In a deep drift on wet mud (R 9 and R 15 x 180 at 15 m, the rear at 0.3 slip and 24 deg of
  sideslip) the axle's longitudinal grip is the friction circle's remainder; a real lock-up,
  released, but the grip found (0.14-0.17) is held for the drive as if it were the road's.
  Outcomes unchanged in the runs above; a hold that knows the lateral load is left.
