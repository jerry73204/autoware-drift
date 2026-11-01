# 013: the class-zone chain's ice zone stops most runs that reach it

**Severity:** high (blocks roadmap 061's gate item "the true-map chain finishes": with issue 012
fixed, the runs that get past the chain's mud and loose-snow zones stop in its ice zone).
**Status:** fixed in the planner (L1's rear brake within what the rear carries sliding on a held
road's first 50 m, `window.recover_gains`; see "Root cause" and "The fix"): ice-zone stops 15 of
19 runs reaching it to 3 of 19. Found in issue 012's fix and 061's gate re-score,
`docs/mode-choice-m4i.md`, roadmap 061 M4i.

## What is wrong

The class-zone chain (`studies/mode_choice_m4.py run zones`, 8 m wide), zone 4: ice from s 744
to 952, a 180 deg corner from s 803 to 933, entered from 44 m of asphalt (zone 3, gravel, ends at
s 700). Repeatable runs (`RP_DET=1 RP_ITERCAP` 150-200, last-bit `--jit` draws), autoware-drift
with issue 012's fix, the car at sim_vehicle_ecu 760cd78:

| | runs reaching zone 4 | stopped there |
|---|---|---|
| issue 012's fix, single / sched / always, 5 draws each | 11 | **7** (s 773-944) |
| the same, the car's ECU at f3f5dcf (single, 6 draws) | 6 | 4 (the 4 draws compared with 760cd78: the same outcome, s to 0.1 m) |
| the same, the car's ECU at d68637a (before issue 009's regen slip control) | 6 | 6 |
| the same, the ECU primed with ice's grip at the start (0.2313, the ice corners' prime) | 6 | 5 |
| main (1f0725e), `LINE_MU = 0` (a study patch: the line at 0.2 m on ice too) | 7 | 7 |
| M4h (95b902e, the old car), repeatable and wall clock | | 5 of 20 runs stopped on ice |

On main without the fix few runs reach zone 4 (issue 012 stops them in zones 1-2); the 2 that did
passed it.

## What the runs show

`zones__single_r1_d170j5q50` (swb `play_log/race_planner/mode_choice/i012/trace/`, `zq_d170.npy`
L2 every tick, `.traj.json` the plant's wheels):

- The car enters the ice at s 744 at 20.3 m/s, on the plan's speed, the plan braking at about
  -2 m/s^2 from the zone's edge (ice's grip 2.6 m/s^2).
- From s 748 to 760 the rear wheels are at 4.5-7.4 times ice's peak slip (slip -0.23 to -0.37;
  the peak at 0.05), the fronts near theirs: the car brakes on its rear and yaws (-3 deg against
  the plan's -1.3 to -1.9).
- L2 steers against it, the fronts go to 2.5-3.2 peaks, and the car runs to the outer edge
  (n -3.07 at s 775, the plan at -2.65), 0.4-1.2 m/s above the plan's speed from s 770 on; it
  leaves the road at s 816-851 in the corner, or later in it.
- `zones__single_r1_d200j8f2` (issue 012's fall-off form, the same weights on ice): the same, the
  rear at 4 peaks at s 747, L2 at 16-30 deg against the plan's 2-3 and lifting off the brake
  (request -0.4 to +0.7 at s 755-770), the car 1-2 m/s fast through the corner, off at s 846.

The priming does not reach the ice: the ECU's grip found is raised to what the rear carries
whenever it brakes unslipping, so a hard braking on the asphalt before the zone forgets it.

## Where to look

- The ECU's regen slip control on a rear that crosses from asphalt onto ice while braking: the
  release needs the axle 30 % short of its brakes or a runaway, and ice keeps 0.7 of its peak
  sliding, at that limit. The car without the control (d68637a) is worse, f3f5dcf the same.
- The plan: L1 brakes from the ice's edge at the hold's rate (issue 009: regen within 0.7 of the
  rear's grip), which the car reaches only once its ECU has found the grip; the corner suite
  primes the ECU for that (M4g), the chain cannot.
- L2 at the yaw: steering 16-30 deg where the plan steers 2, its fronts past their peak (M4f's
  mechanism), and the brake lifted.

## Reproduce

```
cd src/planning/race_planner
RP_DET=1 RP_ITERCAP=170 env -u PYTHONPATH .venv/bin/python studies/mode_choice_m4.py run zones \
    --arm single --jit 5e-9 --tag d170j5 --traj          # off_road @851.6
```

## Root cause (2026-11-02, autoware-drift 55c6270, the car at sim_vehicle_ecu 760cd78)

Every run repeatable (`RP_DET=1 RP_ITERCAP`, `--jit` draws), the plant's ECU traced every 60 Hz
step (wheel slips, regen, its cap and the axle's grip found, the brakes), L2 every tick and every
adopted plan, s 680-1000. Results and scripts in swb `play_log/race_planner/mode_choice/i013/`
(`trace/` main, `oracle/`, `r100/` `r110/` `r120/` the forms not shipped, `fix/` and `fixtr/` the
fix, `base/` main's extra draws, `ice/`, `other/`, `digest/`, `traj_runs/`, `tools/`, `pool.txt`).

**L1 plans the transition and the car arrives on it.** The plan accelerates to 21 m/s on the 44 m
of asphalt, brakes there at -4 to -7.7 m/s^2 to the ice's edge and enters the ice at 19.5-20 m/s
braking at -2.4 (0.92 of ice's mu g), enough for the corner by its model. The car is at the
boundary on the plan's speed (0.1-0.3 m/s off) and line (0.1 m). Not too fast.

**The rear goes past its peak from the car's regen at the step, and stays there from the plan's
brake.** `zones` single d170j5 (s 744-765):
- The ECU brakes with the last road's grip until a slip shows it ice's: its regen cap is 0.7 of
  the axle's friction found, here 0.469 (found on the gravel, raised on the asphalt to what the
  axle carried), 2139 N against the ice rear's peak of about 1730; with none found (other draws)
  the motor's full 2812 N. The rear slips at once (-0.25 at s 748).
- The regen slip control then cuts regen to the rear's own share of the brakes (the static
  50:50 split), and no further: below it, cutting regen hands the braking to the same axle's
  friction brakes, and the car has no ABS (the spec). At the plan's -2.4 that share is 1500 N:
  within the rear's peak (0.88 of it) but past what ice's tyre carries sliding (0.7 x 1730 = 1210).
  So the rear does not come back: slip -0.33 to -0.46 for 0.8 s with regen at the share and the
  modulator at 0, the car yaws (-3 deg against -1.4), until L2 lifts off (s 763). From there the
  car is 0.4-0.5 m outside the plan near the 8 m road's edge, L2 steers into the turn and gives up
  the brake, the car runs 1-2 m/s fast and leaves the road in the corner.
- On the plant alone (straight ice at 19.8 m/s, a request held; `tools/latch.py`): with the ECU's
  grip stale (0.469) the rear slips and comes back in 0.27 s at a request whose rear share is
  1.00 of the sliding force (-2.0 m/s^2), 0.40 s at 1.06, 0.60 s at 1.09, and stays locked from
  1.12 (-2.2) on (slip -0.38 to -1.1 for the whole 1.5 s); with ice's grip found (0.23, the corner
  suite's prime) every request to -2.4 rolls (slip within 0.04, no event).

**Single start**: the same (single runs stop there 7 of 10 reaching it). **The ice corners**
start on ice with the ECU primed (M4g): no step and no probe, the plan's -2.4 rolls on the rising
branch. That is why they pass and the chain does not.

**Not the ECU's control as such**: telling the car ice's grip at the step (`I013_ORACLE`, axle_mu
set to 0.2313 on the first ice node) does not stop the event: in the same step the ECU raises its
grip to what the axle carries unslipped (the asphalt braking's 2700 N, 0.38), as it must on a road
it does not know. ECU f3f5dcf gives the same outcomes (issue text above), d68637a (no control)
worse. The car cannot know the road changed until it slips; what decides is whether the plan's
brake lets a slipped rear come back.

## The fix (the planner)

`window.recover_gains` and `recoverable_rear_rows` (a hold car only, `axle.regen_hold_car`: a
course with a held node): on a held road's first `PROBE` (50) m after a node that is not, the
rear tyres' braking rows are within `RECOVER` (1.1) of their sliding force (sliding share x
planned mu x Fz; 1.1 at the planned grip 0.92 is 1.01 of the true road's, where the plant's slip
comes back in 0.3 s). One parameter more per stage (the gain, 0 elsewhere); the drive rows and
every other node as before. A course that starts on a held road has no bound there (its car has
braked on it: the studies prime the ECU). Test `test_window.py::
test_a_held_roads_first_metres_brake_the_rear_within_what_it_carries_sliding`.

With it the plan enters the ice at -1.8 to -1.9 m/s^2, braking harder on the asphalt before
(d170j5: the rear slips to -0.22 at s 746 and rolls again by s 752, the car within 0.1-0.2 m and
0.1 m/s of the plan through the corner).

## Result (repeatable; the true-map chain, 10 draws an arm: M4i's 5 and 5 more)

| | main | the fix |
|---|---|---|
| chain: stopped in the ice zone, of the runs reaching it (pooled) | **15 of 19** | **3 of 19** (-0.63, se 0.13: 4.9 se) |
| single / sched / always | 7 of 10 / 5 of 5 / 3 of 4 | 2 of 10 / 0 of 5 / 1 of 4 |
| chain finished | 0 of 30 | 2 of 30 (single d170j2 158.1 s, always d150j7 157.8 s) |
| ice corners (12 x 5 draws x 3 arms) | 58 / 59 / 60 of 60 (M4i) | the same 180 runs bit for bit |
| wet mud, loose snow, gravel, packed snow (one corner each) | | bit for bit (no held node: the window is the old one) |
| Haruna / Tsukuba `RP_DET` single and sched | 76.28 / 74.33 s | the same, digests 9524862c / 9160b94d, d7ca9f95 / 82bd3c28 |

## Not the fix

- **The bound on every held node** (`r100`/`r110`/`r120`): at `RECOVER` 1.0 the plans are
  infeasible (the regen hold's own cap is the sliding force, 0.7 = 0.7: any braking past it
  violates the row); at 1.1 the chain's ice stops 2 of 11 but the ice corners 44 of 60 single
  (R 9 x 90 at 6 m, R 9 x 180 at 6 and 15 m lost: they need ice's full braking); at 1.2 corners
  56 of 60, chain 4 of 11. A car that has found the grip does not need it.
- **The ECU told ice's grip at the step** (above).

## Left

- The 3 runs still lost in the zone (single d185j1, d200j6; always d185j1): the probe slips and
  recovers, but the car's yaw in it (1 deg) takes it 0.3-0.5 m outside a plan that runs near its
  outer edge (n -2.6 to -2.7, the car to -3.0 to -3.5), the edge rows then rule L2: it steers 11-58 deg into the turn
  with the fronts locking under the brake (d185j1), or lifts off the brake for 50 m and the car
  slides to 20 deg (d200j6). L2's and the edge margin's, as issue 012's chain stops.
- `PROBE` is a length: a plan that does not brake in a held road's first 50 m probes later,
  unbounded. Every held entry on the chain is braked into.
- The chain past the ice (M4i finding 3), now first: loose snow R 9 x 90 (zone 5) 11 of 30 runs,
  zone 0 on the multi-start arms 10 of 20, the packed hairpin 3.
