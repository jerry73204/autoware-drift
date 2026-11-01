# 013: the class-zone chain's ice zone stops most runs that reach it

**Severity:** high (blocks roadmap 061's gate item "the true-map chain finishes": with issue 012
fixed, the runs that get past the chain's mud and loose-snow zones stop in its ice zone).
**Status:** open (found in issue 012's fix and 061's gate re-score, `docs/mode-choice-m4i.md`,
roadmap 061 M4i; not root caused).

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
