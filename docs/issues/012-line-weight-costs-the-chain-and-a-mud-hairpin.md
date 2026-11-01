# 012: since L2's low-friction line weight (issue 007's fix) the zone chain stops on its snow zones and a mud hairpin spins

**Severity:** high (blocks roadmap 061's gate: the true-map class-zone chain, finished by every
multi-start run in M4c, is finished by none on 95b902e). **Status:** fixed (L2's loosening in
full only where the surface falls off past its peak, `tracker.LINE_SLIDING`; see "Root cause"
and "The fix"). Found in 061's gate re-score, `docs/mode-choice-m4h.md`, roadmap 061 M4h. The
chain's other stops are not this issue: its ice zone (issue 013) and its packed-snow hairpin
(M4f's mechanism, the plan's front far past its peak; below).

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

## Root cause (2026-11-01, on main 1f0725e with the car at sim_vehicle_ecu 760cd78)

Every run repeatable (`RP_DET=1 RP_ITERCAP`, last-bit `--jit` draws), L2 traced every tick (its
prediction, references, the cost's terms by kind at its solution, the free width) with the
plant's wheels from `--traj`. Logs: swb `play_log/race_planner/mode_choice/i012/` (`trace/`,
`base/` main, `q/` the fix, `fix/` and `fall/` earlier forms, `iso/`, `prime/`, `ecu/`, `bisect/`,
`room/`, `digest/`).

**Reproduced on main** (the planner and the car have both moved since 95b902e): `mud_wet:15:180:6`
off the road at s 82.3 in all 5 runs (single 150/170/200, sched, always), finished with
`LINE_MU = 0` (18.98 s). The chain (single, sched, always, 5 draws each): 1 of 15 finished; of the 11 runs that reached
zone 1 (mud), 6 stopped there and 2 more in zone 2 (loose snow), both surfaces whose tyre keeps its force past
the peak (sliding share 1.0).

**What the line weight does on a road that keeps its force sliding.** Issue 007's runaway needs
a rear that loses force past its peak (packed snow, sliding 0.8): L2 buys line with rotation, the
rear falls off, the slide runs away. Where the tyre keeps its force (wet mud, loose snow: 1.0)
rotation is a safe handle, and the loosened line (0.69 m on mud, 0.8 on loose snow) only lets
the car drift from the plan:
- `mud_wet:15:180:6` (6 m, the car's centre at most 1.95 m off the centre line): main and
  `LINE_MU = 0` run tick for tick alike to s 55 (0.1 m apart); from the turn-in the loose line
  lets the car run 1.0-1.1 m outside the plan (s 64-68: n 0.40 and 1.40 against the plan's
  -0.65 and 0.26; with the 0.2 m line 0.69 and 0.48). At s 72 the car is at its edge (n 1.85-1.95):
  the edge rows (1e4 a m^2) now rule, L2 cannot countersteer (that sends the car over the edge),
  keeps 13-33 deg of steering into the turn, the front at 3-8 peaks, the slide grows from the
  plan's 23 deg to 55 as the speed falls from 6.4 to 1.5 m/s. With the 0.2 m line the car keeps
  0.4-0.6 m of the plan, its front also at 2-2.7 peaks at s 64, L2 countersteers (-16 deg at
  s 75) and catches the slide (24 deg against 19).
- The chain's mud hairpin (zone 1, R 9 x 180 at 8 m, `zones` single d170j5, stalled s 268.7):
  the car under-rotates (25 deg falling to 1.6 against the plan's 32 to 16, s 227-240); with the
  line at 0.69 m nothing holds the rotation (the sideslip term alone), the car crosses its 8 m
  road from the plan's outer edge (n 2.7) to the inner one (-2.8) and stalls there, 300 fallback
  ticks.

**But the 0.2 m line is not right on these roads either.** With the loosening off on mud and
loose snow (sliding 1.0, `LINE_SLIDING` (0.8, 1.0)), `mud_wet:9:180:15` leaves the road in every
arm (5 of 5 runs, s 64; main finishes all): L2 steers 28-43 deg against the plan's 22-25 to hold
0.4 m of predicted line error at its horizon's end, the front past its peak, the car slows to
3.7 m/s against the plan's 6.3 and slides to 33 deg against 20: issue 007's loop, on mud.

So the two failures are one balance: on a road that keeps its force sliding, a line loosened in
full gives up the line until the edge decides (6 m and 8 m roads); the asphalt line drives the
front past its peak in a tight hairpin (R 9 at 15 m). Swept (single start, mud's and loose
snow's 24 corners, `ITERCAP` 170): the share of 007's loosening kept at sliding 1.0 of 0 and 0.25
loses R 9 x 180 at 15 m, 0.67 and 1 (main) lose R 15 x 180 at 6 m, **0.5 finishes all 24**.

**Not M4f's mechanism.** In the mud corner the front past its peak is a consequence (L2 steering
into the turn at the edge), not the plan's: the plan steers 15-26 deg with the front at its peak.
The chain's packed-snow hairpin (R 25 x 180 at 8 m, s 1205), where the issue suspected the two
meet, is M4f's: the plan itself steers 41-43 deg there (its front at 7-11 peaks, slip angle 35-49
deg against packed snow's 6) with 3.6 deg of slide, the car follows that steering within 1 deg,
runs 1 m out of the plan and L2 reaches the lock (58 deg) with no lateral authority left
(`trace/zq_d185.npy`). The line's weight there is issue 007's, unchanged by the fix. The same
corner alone (`c:snow_packed:25:180:8`) finishes in every draw with any line weight (main, the
fix, `LINE_MU = 0`; 4 draws each), as does the chain's loose-snow R 9 x 90 alone
(`c:snow_loose:9:90:8`): on the chain they are lost to the state the car arrives in.

## The fix

`tracker.LINE_SLIDING = (0.8, 1.2)`: the share of issue 007's loosening kept runs from 1 at a
sliding share of 0.8 or less (packed snow, ice: issue 007's weights to the bit) to 0 at 1.2:
0.5 on mud and loose snow (1.0), 0.75 on gravel (0.9). Line scale 0.2 m x (1 + f (max(1,
LINE_MU / mu) - 1)): wet mud 0.44 m (0.69 before), loose snow 0.5 (0.8), gravel 0.27 (0.29),
packed snow 1.0 and ice 0.75 as before; asphalt 0.2 m and the constant-weight QP as before.
Test `test_the_line_gives_way_in_full_only_where_the_surface_falls_off`; issue 007's test now on
a road that falls off (sliding 0.8).

## Result (repeatable, `RP_DET`, the car at 760cd78)

| | main (1f0725e) | the fix |
|---|---|---|
| `mud_wet:15:180:6` | off the road at s 82.3, 5 of 5 | finished 5 of 5 (25.8-28.6 s; `LINE_MU = 0` 18.98: the car slides past the plan's, but holds) |
| wet mud's 12 corners (single 150/170/200, sched, always at 170) | 33 / 11 / 11 | **36 / 12 / 12**; time on corners both finish +1.4 % (median +0.4 %) |
| loose snow's 12 | 36 / 12 / 12 | 36 / 12 / 12; -0.2 to -2.0 % (median -0.1 to +0.4 %) |
| gravel's 12 | 36 / 12 / 12 | 36 / 12 / 12; 0.0 to +0.2 % |
| packed snow, ice | | bit for bit (plan and plant digests of `snow_packed:25:90:15`, `ice:15:180:6` equal); issue 007's corners finish |
| Haruna, Tsukuba (`RP_DET`, single) | 76.28 s, 74.33 s | the same, digests 9524862c / 9160b94d and d7ca9f95 / 82bd3c28 |
| the chain, true map (single, sched, always; 5 draws each) | 1 of 15 finished; zones 1-2 (mud, loose snow) stop 8 of the 11 runs reaching them | 0 of 15 finished; zones 1-3 stop **0 of 11** runs reaching them; then ice (zone 4) 7 of 11 (issue 013), loose snow R 9 x 90 3 of 4, packed hairpin 1 of 1 (M4f's) |

The chain's zone 0 (asphalt declared uncertain) stops 4 of 15 runs on main and on the fix alike, the same draws (d185, d200 for sched and always, s 98.8: the plan slides 15 deg at
16 m/s on the uncertain zone's 0.7 friction, the car on the true asphalt does not rotate and
runs wide at the lock): the same stop on 95b902e with the old car (`bisect/`), a last-bit
outcome (M4h saw it only in its `LINE_MU = 0` draws). Not this issue.
