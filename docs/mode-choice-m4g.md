# Grip or drift, model in the loop: ice with the car's braking fixed (roadmap 061 M4g)

Written 2026-11-01. M4f (`docs/mode-choice-m4f.md`) root-caused ice's last losses to the car:
the drift car brakes on ice through its rear axle only (issue 009). The user ruled for the car
(option 1): regen limited by the rear's grip and slip-controlled as a real EV's brake module
blends it, the planner's axle model following. Results: swb
`play_log/race_planner/mode_choice/m4g/` (`det/` the repeatable runs, `det_noprox/` the
first batch before L2's front fix, `digest/` Haruna and Tsukuba, `repro/` traced runs,
`isaac/` the Isaac run, `tools/` the scripts and `agg.py`, the tables below).

## What was built

**The car** (sim_vehicle_ecu `f3f5dcf`, `ev` powertrain; README "Regen slip control"; from the
brake module's own estimates, never the simulator's):

- **Slip control on regen.** The regen beyond the driven axle's share of the brakes (the brake
  distribution's) is cut while a driven wheel slips, by the modulator ABS and TC use; the
  friction brakes take what it gives up, split by the distribution (static 50:50 on
  drift_mustang). Regen within the axle's own share is left alone (cutting it would only hand
  it to the same axle's friction brakes). It releases past 0.25 slip (ABS's) or when a driven
  wheel runs away: past 0.06 slip, slowing 3 g faster than the car, and its axle carrying 30 %
  less than the brakes put on it. A spool's slip is its two wheels' mean.
- **The axle's grip found.** In a slip event the axle's tyre force is measured from the wheels'
  own balance (brake and regen torque and the driveline's drag, less what the wheels' inertia
  gives up), the event's highest kept as the axle's friction; never held below what the axle
  carries without slipping, forgotten at 0.001 /s.
- **Regen held.** From then on regen beyond the axle's own share stays within 0.7 of that grip
  (`regen_grip_share`).

On ice at -1.8 m/s^2 from 13 m/s it finds 0.227-0.243 (ice's peak 0.265) within 0.1 s and the
rear rolls (slip under 0.02); without it the rear turned backwards (-50 m/s peripheral). At
-2.1 the car decelerates at 2.10 with every wheel rolling; at -2.3 and beyond the rear's own
friction share (no ABS, static split) is past its grip and locks it, as before.

**ABS stays off** in the drift spec: not needed. An ABS-on spec (local override, not
committed) did not save any of the snow losses below, and ABS's 0.25 release is 5 times ice's
peak slip.

**The planner** (autoware-drift):

- `axle.dynamics`: where the road is `held` (slick enough for the motor's full regen to slip
  the rear, `axle.regen_slip_mu` 0.42 on drift_mustang, and falling off past its peak as ice
  does, sliding at most 0.75), regen is held within `car.regen_grip_share` of the rear's grip
  at the planned friction, smoothly: rear = max(min(regen, 0.7 mu Fz_r), (1 - share) B),
  front = B - rear. A switch on the stage's parameters; where no node is held
  (`axle.regen_hold_car`, in live.Driver and both trackers) the model is the old one
  expression for expression.
- L2 (`tracker`): the front wheels' proximal term 0.3 m/s (`W_PROX_FRONT_HELD`, 1.0 elsewhere)
  where the hold applies.
- The study: on a held road the car starts the corner knowing the rear's grip, as a car that
  has braked on that ice before the corner does (`sim.Plant.regen_grip_found`: -1.8 m/s^2 for
  0.6 s on the course's start, then a fresh start with only the grip kept; `ecu_primed` in the
  result JSON). `SIM_VEHICLE_ECU` points the plant at another sim_vehicle_ecu checkout; the
  result JSON records the ECU (`ecu`, `ecu_regen_slip`) and the planner's hold (`regen_hold`).

## Method

As M4e: repeatable runs (`RP_DET=1 RP_ITERCAP`), ice's 12 corners, budgets 150, 160, 170, 185
and 200 iterations, last-bit draws `--jit` k x 1e-9; single 20 replicates (k 0-3), sched 10
(k 0-1), against M4e's `78` (main's planner, the old car) on the same (budget, draw). Welch se
of the difference of the means, under 2 se read as noise. Other surfaces: packed snow, loose
snow and wet mud, single, budgets 150 / 170 / 200 (one draw each), new against main's planner
on the old car (`b`) run the same way. Every run's configuration (det, itercap, jit, the ECU,
the hold, the priming) checked in its JSON by `tools/agg.py`. Load 4-20 from other users; no
number depends on it.

## Results

**Ice, single start** (finished of 12 a replicate):

| | finished | 6 m roads | 15 m roads | failed re-plans | lost |
|---|---|---|---|---|---|
| M4e `78` (old car) | 9.55 (sd 0.69, 8-11; 191 / 240) | 76 / 120 | 115 / 120 | 19 % | L1 32, L2 17 |
| **M4g** | **10.90** (sd 0.72, 10-12; 218 / 240) | **108 / 120** | 110 / 120 | 15 % | L1 22, L2 0 |
| difference | **+1.35, se 0.22 (6.1 se)** | | | | |

**Ice, sched:**

| | finished | 6 m roads | 15 m roads | failed re-plans | lost |
|---|---|---|---|---|---|
| M4e `78` | 9.80 (sd 0.92, 8-11; 98 / 120) | 40 / 60 | 58 / 60 | 15 % | L1 13, L2 9 |
| **M4g** | **10.30** (sd 0.48, 10-11; 103 / 120) | 50 / 60 | 53 / 60 | 14 % | L1 16, L2 1 |
| difference | +0.50, se 0.33 (1.5 se: noise, not worse) | | | | |

Per corner (single, of 20; sched of 10):

| corner | M4e single | M4g single | M4e sched | M4g sched |
|---|---|---|---|---|
| R 9 x 90, 6 m | 17 | 20 | 9 | 10 |
| R 9 x 90, 15 m | 19 | 20 | 9 | 10 |
| **R 9 x 180, 6 m** | **0** | **8** | 0 | 0 |
| R 9 x 180, 15 m | 17 | **10** | 10 | **4** |
| R 15 x 90, 6 m | 11 | 20 | 9 | 10 |
| R 15 x 90, 15 m | 20 | 20 | 10 | 10 |
| **R 15 x 180, 6 m** | 15 | **20** | 5 | **10** |
| R 15 x 180, 15 m | 20 | 20 | 10 | 9 |
| R 25 x 90, 6 m | 20 | 20 | 10 | 10 |
| R 25 x 90, 15 m | 20 | 20 | 10 | 10 |
| R 25 x 180, 6 m | 13 | 20 | 7 | 10 |
| R 25 x 180, 15 m | 19 | 20 | 9 | 10 |

- **The car's braking is ice's lever**: single +1.35 corners of 12 (6.1 se), the 6 m roads from
  63 to 90 %, L2's losses gone (17 to 0 single, 9 to 1 sched). R 9 x 180 at 6 m, lost in every
  run since M4e, finishes 8 of 20.
- **One corner is worse: R 9 x 180 at 15 m** (single 10 of 20 against 17, sched 4 of 10
  against 10). Every loss is L1's (6-9 failed re-plans) at the hairpin's entry (s 80-98), with
  L2 falling back 11-70 ticks: the plans now brake later and harder (the fronts' brakes), so
  the car reaches the hairpin hotter, where the windows from the car's state already needed
  177-1549 iterations in M4f. M4d's "left" (the hairpin's windows at the rear's peak), now on
  the 15 m road too. The 6 m road's R 9 x 180 gained more (0 to 8 single).

**Haruna and Tsukuba** `RP_DET` single on the new car and planner: **76.28 and 74.33 s, plan
and plant digests equal to M4e's**: the hold is not in the model on a dry road and the regen
slip control never acts there (no slip event, no cap, in either lap: traced).

**Other surfaces** (single, 150 / 170 / 200; the planner without the hold there, the car with
its control):

| | old car (`b`) | M4g | lap time, both finished |
|---|---|---|---|
| packed snow | 36 / 36 | 36 / 36 | -0.015 s mean (3 of 36 runs differ, within -0.70..+0.08) |
| loose snow | 36 / 36 | 36 / 36 | -0.40 s mean (34 of 36 differ: the plans run the rear at 0.22-0.26 slip near loose snow's 0.3 peak, the 0.25 release fires and the car holds; -7.44..+1.16) |
| wet mud | 33 / 36 | 33 / 36 | -0.018 s mean (24 of 36 differ; the same corner lost, R 15 x 180 at 6 m) |

## How it got there (the losses on the way)

- **The crude cap's 15 m losses (M4f)**: its model's hard `ca.fmin(regen, cap)`. L2's RTI
  linearises at the kink and HPIPM stopped at its minimum step (status 2) every other tick, the
  LQR fallback's +0.5 m/s^2 drive in between: 136, 97, 33 and 340 fallback ticks in the four
  lost runs (M4e's 0-3). With a smooth min the same four finished (failures 0-17).
- **A first ECU on fixed slip thresholds** (release 0.10): fired on Haruna and Tsukuba (rear at
  0.10-0.19 braking hard, a spool's inner wheel in a turn, a wheel down from a wheelspin),
  laps 76.53 and 74.57 s. Hence: only the regen beyond the axle's share, a spool's mean slip,
  and a runaway (slip, deceleration and the axle's deficit together).
- **The grip from the car's deceleration** lagged the tyre: once 0 at an event's start, held
  regen at 730 N where the plan had 1130. Hence the wheels' own balance.
- **The car not knowing the grip at the corner's start**: the corner's first braking was its
  probe while the plan assumed the hold; 4 of 12 at 150 iterations (M4e 8). Hence the study's
  priming (a car on ice has braked on it).
- **L2's front wheels** with W_PROX 1.0: on ice the fronts now brake in turns at a front past
  its 3 deg lateral peak, where their force balance is flat in the wheel's speed; the
  predicted front-left speed swung 5-10 m/s tick to tick, HPIPM failed every other tick: the
  15 m roads 46 of 59 (`det_noprox/`, 119 runs). At 0.3 m/s the two traced losses finished with
  no failed QP.
- **The hold on snow and mud too** (every road below 0.42): packed snow 28 of 36, loose snow 24
  of 36 (against 36 and 36; mud 36 against 33): the plans used the fronts in a braking drift
  (planned sideslip -9 to -16 deg on the straight) that left the 6 m roads, L2's losses, the
  fronts locking at 0.4-0.5 slip. With ABS on (local spec override) the same. Those surfaces
  keep their force past the peak, the old plans braked them through the rear alone within its
  grip and finished every corner, and the car does not slip there to need the hold: so the
  hold is in the model only where the road also falls off (ice).

## Isaac

See the roadmap entry (061 M4g): Haruna, gravel, single start, on the new car.

## Gate

| criterion | M4g | |
|---|---|---|
| ice single and sched not worse pooled | single +1.35 (6.1 se), sched +0.50 (1.5 se) | met |
| ice 6 m roads better | single 76 -> 108 of 120, sched 40 -> 50 of 60 | met |
| ice 15 m roads | single 115 -> 110 of 120, sched 58 -> 53 of 60 (R 9 x 180 at 15 m) | **not met for that corner** |
| Haruna / Tsukuba digests 76.28 / 74.33 s | equal (plan and plant digests) | met |
| packed / loose snow, mud not worse | 36 / 36 / 33 finished as before; laps 0.02-0.40 s faster | met |
| suite, sim_vehicle_ecu tests | see the roadmap entry | |

## Rulings

- **The car's control acts only on regen beyond the driven axle's share.** Below it, cutting
  regen only moves the same braking to the same axle's friction brakes. Cost if wrong: a rear
  locked by its own friction share (requests past ~2.1 m/s^2 on ice with the static split) is
  ABS's to save, and the drift car has none.
- **ABS stays off** (not needed; not authorised to change the spec). The rear's own share at
  the static split locks it at -2.3 m/s^2 on ice; the planner's rows keep the rear on its rising
  branch.
- **The planner holds regen only on roads that fall off (ice)**, not on every road below
  0.42. Cost if wrong: on snow or mud, once the car has slipped its rear under regen (loose
  snow's plans brush its 0.25 release) it holds and the plan does not know: there the car
  brakes the fronts more than planned (outcomes unchanged in 108 runs, laps a little faster).
- **The study primes the car's ECU on held roads.** A car on ice finds its rear's grip at its
  first stop; a one-corner course has no earlier stop. Cost if wrong: on a course entering ice
  from a dry road, the first stop on the ice is the car's probe and the plan's assumption is
  ahead of the car by that stop (the planner does not read the car's found grip: left).
- **The regen hold share 0.7 and the thresholds** (0.25, 0.06 / 3 g / 30 %) are calibration
  defaults, not in the spec; they kept Haruna and Tsukuba bit for bit.

## Left

- R 9 x 180 at 15 m on ice: the hairpin's windows from a hotter entry (M4d's left, now on the
  15 m road).
- The planner reading the car's found grip (the plant has it, `ecu.powertrain.axle_mu`; aib
  would report it as it does the friction estimate) instead of assuming it on ice.
- The hold on snow and mud: plans that use the fronts in a braking drift there leave the 6 m
  roads.
- R 9 x 180 at 6 m: 8 of 20 single, 0 of 10 sched.
