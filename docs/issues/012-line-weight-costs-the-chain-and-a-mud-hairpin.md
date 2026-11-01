# 012: since L2's low-friction line weight (issue 007's fix) the zone chain stops on its snow zones and a mud hairpin spins

**Severity:** high (blocks roadmap 061's gate: the true-map class-zone chain, finished by every
multi-start run in M4c, is finished by none on 95b902e). **Status:** open (found in 061's gate
re-score, `docs/mode-choice-m4h.md`, roadmap 061 M4h; not fixed there).

## What is wrong

The class-zone chain (`studies/mode_choice_m4.py run zones`, 8 m wide, ten corners in zones of
swb classes) on the true map, autoware-drift 95b902e (issue 007's line weight, 14a79e7, and
issue 008's slip rows, 9771e96/6cf5108):

| | runs | finished | where they stop |
|---|---|---|---|
| M4c (`m4c2`, before 007 and 008), wall clock | sched 2, always 2 | **4 of 4** (153.0-154.7 s) | |
| 95b902e, repeatable (`RP_DET`, `RP_ITERCAP` 150/170/185/200, last-bit start draws) | single 4, sched 4, always 4 | **0 of 12** | s 1074-1076 (zone 5, loose snow R 9 x 90) 5; s 1204-1208 (zone 6, packed snow R 25 x 180) 3; ice (zone 4) 3; s 1746 (zone 8, mud) 1 |
| 95b902e, wall clock (load 4-10) | single 2, sched 2 (+ est 2), always 2 | **0 of 8** | packed-snow hairpin s 1206-1209 3; loose snow s 481; mud hairpin s 256; ice 2; mud s 1742 |
| 95b902e with issue 007's fix off (`tracker.LINE_MU = 0`, a study-only patch, M4e's), the same 12 repeatable draws | single 4, sched 4, always 4 | **4 of 12** (single 1, sched 1, always 2; 153.5-154.3 s) | ice 4 (single 3); s 100 (zone 0, asphalt declared uncertain) 4 |

Of the runs that reached s 1050: with the line weight **9 of 9 stopped** (8 in zones 5-6, 1 in
the mud of zone 8); without it **0 of 4** (all finished). (The four s 100 stops without it are another matter, the same draws finishing
with the weight: issue 007's fix wins zone 0 and loses zones 5 and 6.)

**A mud hairpin too.** `mud_wet:15:180:6` (a corner suite case, not re-run on mud after issue
007): with the line weight it leaves the road at s 82.3 in every arm and draw (single 3 budgets,
sched and always at 170; the same s every time), the car's sideslip running from 23 to 59 deg
against the plan's 23 as its speed falls from 6.5 to 1.3 m/s; with the weight off
(`LINE_MU = 0`) it finishes in all 3 draws (18.14 s, car 24 deg against 20). Mud corners: 11 of
12 in every arm (M4b/M4c 12 of 12).

## What the runs show (traces in swb `play_log/race_planner/mode_choice/m4h/chaindet/traj/`,
`chain/traj/`)

- **Packed-snow hairpin (R 25 x 180, s ~1205).** The plan slides 8-14 deg; the car stays at
  3-5 deg with the steering at its lock (58 deg) from s ~1195, its speed on the plan's, and
  drifts out of the plan 1-2.5 m over 25 m until it leaves the 8 m road (`always_r1_d200j8`:
  n 1.79 against the plan's 0.98 at s 1198, 2.75 against 1.86 at s 1201; `single_r1` wall
  clock: 3.1 m against 0.6). Re-plans succeed (0-6 failed of ~95). L2 does not rotate the car
  into the plan's slide; with the line at packed snow's 1.0 m scale (0.2 m x 1 / 0.2), a metre of
  line error weighs what 0.2 m did on asphalt.
- **Loose snow R 9 x 90 (s ~1063-1075).** The plan slides 15-21 deg; the car loses its slide
  (14 deg, then -1 deg at s 1064 against the plan's 20), swings across the road and leaves it on
  the other side (`sched_r1_d170j5`: n -2.8 at s 1061, +2.5 at s 1074 against the plan's -1.3).
- No runaway slide (issue 007's pattern): the car's sideslip stays at or under the plan's.

## Cause (suspected, not proven)

Issue 007's fix made L2's line weight scale as 1 / mu below mu 1 (packed snow 1.0 m, loose
snow 0.8, mud 0.69). It was checked on the corner suites (15 and 6 m wide, entered from a
straight at the corner's own speed), not on the chain. On the chain the car comes into the snow
zones carrying the previous zone's state and must rotate into a planned slide; with the line
loose, L2 gives up line before it asks for the rotation, and the 8 m road's margin (the car's
centre leaves it at 3.55 m) is used up. The same mechanism as M4e's "6 m roads: L2 leaves the
road 0.4-3.8 m off the plan" on ice. Issue 011 (the new ECU, f3f5dcf) also leaves wet-mud corners, by another cause (the car); its
old-car count, wet mud 33 of 36, fits this corner's loss (not checked by name). M4f found ice's 6 m losses not the line weight
but a front past its peak with no lateral authority left; on the packed-snow hairpin the front
is at its lock, so the two may meet here. Not tried here: a line scale bounded by the road's margin
(the edge's distance), xi's scale on low friction, the scale tied to what the car can correct in
the horizon (issue 007's own suggestion).

## Reproduce

```
cd src/planning/race_planner
RP_DET=1 RP_ITERCAP=170 .venv/bin/python studies/mode_choice_m4.py run zones --arm sched \
    --jit 5e-9 --tag d170j5          # off_road @1075.1 (95b902e)
```
and the same with `tracker.LINE_MU = 0` set before the run (study patch): see M4h's `chaindet/`
results tagged `l0` (`line_mu` 0.0 in each result JSON).
