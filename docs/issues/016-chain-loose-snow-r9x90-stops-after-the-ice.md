# 016: the class-zone chain stops in its loose-snow R 9 x 90 (zone 5), which finishes alone

**Severity:** high (blocks roadmap 061's gate item "the true-map chain finishes": with issue 013
fixed, zone 5 is the chain's first stopper, 11 of 30 runs). **Status:** fixed in the planner (the stage arriving at snow or mud brakes the rear within
that road's grip, `window.recover_gains`; issue 005's cut warm start decided per window,
`live.Driver.cut_warms`; see "Root cause" and "The fix"). Found in issue 013's verification
(roadmap 061, Issue 013). The packed-snow hairpin is not this cause (below).

## What is wrong

The class-zone chain (`studies/mode_choice_m4.py run zones`, 8 m wide), zone 5: loose snow from
s 996 to 1096, a 90 deg right-hander R 9 from s 1056 to 1076, entered from 44 m of asphalt after
the ice zone (s 744-952). Repeatable runs (`RP_DET=1 RP_ITERCAP` 150-200, last-bit `--jit`
draws), autoware-drift 82f0b69 (issue 013's fix), the car at sim_vehicle_ecu 760cd78:

| | runs | stopped in zone 5 |
|---|---|---|
| issue 013's fix, single / sched / always, 10 draws each (swb `i013/fix/`) | 30 | **11** (single 7, sched 3, always 1; s 1069.7-1077.9) |
| main before 013 (`i013/base/`, `i012/q/`) | 30 (4 reach zone 5) | 3 |
| the corner alone at 8 m, `c:snow_loose:9:90:8` (M4i, issue 012) | every draw | 0 |

M4i: the plan's flick (its sideslip 15, 3, 8 deg) is not followed; the car arrives 4 deg against
15 and then slides 25 deg against 11. Since the corner alone finishes, something the chain brings
into the zone is suspect: the entry state, the ECU's grip found and held from the previous roads,
the plan carried across the road change, issue 013's recover rows, the flick seed.

The packed-snow hairpin (zone 6, 3 stops on 013's fix) is checked for the same cause.

## Reproduce

```
cd src/planning/race_planner
RP_DET=1 RP_ITERCAP=150 env -u PYTHONPATH .venv/bin/python studies/mode_choice_m4.py run zones \
    --arm single --jit 7e-9 --tag d150j7       # off_road @1071.5 on 82f0b69
```

## Root cause (2026-11-02, autoware-drift afd0cc5, the car at sim_vehicle_ecu 760cd78)

Repeatable runs, the plant's ECU every 60 Hz step and L2 every tick, every L1 solve logged
(start state, status, iterations), s 940-1250. Results and scripts in swb
`play_log/race_planner/mode_choice/i016/` (`base/` afd0cc5, `diag/` solves traced, `alone/` the
corner alone, `cw/` `fx1/`-`fx3/` `fa/` forms not shipped, `fb/` the fix, `fbtr/` its packed
hairpin traced, `corners_*/`, `digest_*/`, `tools/`).

**Two things the chain brings into the zone and the corner alone does not.**

1. **The car holds regen on the snow; the model says it never does.** Every one of the 10
   single draws: the plan brakes at -4 to -7 m/s^2 on the 44 m of asphalt up to its node at
   s 996, the snow's first. The plant's road changes at s 995.5 (its nodes 1 m, nearest), inside
   the plan's last asphalt stage, and L2 eases the request from that stage's to the next's
   across it, so the car meets the snow asking -1.9 to -2.1 m/s^2; the ECU blends regen first,
   2650-2810 N on a rear that carries about 1900 there. The rear slips (-0.15), the regen slip
   control fires (s 996.0-996.5 in all 10), reads the grip during its own release (0.111-0.17,
   the snow's planned 0.23), and holds regen at 0.7 of it (645 N) for the whole zone: on loose
   snow the rear works at slip 0.1-0.4, above the reapply slip, so the grip found never rises
   again. The model (`axle.held`) holds regen only on ice: on snow "the car never slips it and
   so never holds". So the plan's rotation by the rear's regen (the flick, sideslip 20-23 deg
   from s 1024) is not there: the car reaches 14 deg, crosses the plan, L2 runs to the lock.
   The corner alone starts on the snow: no event until the corner, full regen (1100-1380 N,
   rear slip -0.25 to -0.41) through the approach, the car at 14 deg against 20 and in.
2. **No plan from the road change on.** The windows from s 976-1044 need 150-390 IPOPT
   iterations against the cap's 150-200 (offline re-solves of the traced windows; re-planned
   on the plan from the plan before, the car exactly on it, still 146-243 at s 980-1012, with
   or without the road change before them (snow there instead: the same), with or without the
   hold model, the slip rows, a shorter window: not the road change itself; the corner alone's
   windows take 12-47). Each cut solve restarts from the adopted plan, staler each time,
   because issue 005's cut warm start is off on any course with ice somewhere: 8 s without a
   new plan (s 980 to the stop) in the base draws.

1 alone (the cut warm start forced on, `cw/`): 1 of 3 runs passes the zone. 2 alone (the
arrival bound, `fa/`): zone 5 stops 7 -> 2 of 10, but the mud hairpin (zone 1) 3 new stops.
Both (`fb/`): below.

**The packed-snow hairpin** (zone 6) is not this cause: with the fix the car enters it with
regen 1015 N, no slip event, the grip found 0.66 (unknown, as on the corner alone); it stops
the way M4f and issue 012 describe, the plan steering 40-43 deg (its front far past the peak),
L2 at the lock from s 1197, 1-2 m outside the plan.

## The fix (the planner)

- `window.recover_gains`: the stage arriving at a road where regen can slip the rear and the
  model does not hold it (planned mu below `axle.regen_slip_mu`, not `held`: snow, mud) from
  one where it cannot: its rear braking rows take the gain mu_i / mu_i+1 - 1, the rear within
  the new road's grip in that stage. A hold car only (a course with a held node, as issue
  013's rows), no other stage; a held road's arrival is issue 013's (the bound on its arrival
  stage too cost the chain's ice zone 2 of 4 draws, `fx3/`). Test
  `test_window.py::test_the_stage_arriving_at_snow_brakes_the_rear_within_the_snows_grip`.
- `live.Driver.cut_warms(i0)`: a cut solve warms the next unless its window meets ice
  (`low`, rear_slip_low's roads); was: never on a course with ice anywhere (issue 005's ruling
  came from the ice corners, all ice). Test
  `test_live.py::test_a_cut_solve_warms_the_next_unless_its_window_meets_ice`.

Every course without a held node, and the ice corners (all ice: no arrival, no window off
ice), plan as before bit for bit; only the class-zone chain changes.

## Result (repeatable; the true-map chain, 10 draws an arm: issue 013's)

| stops by zone (30 runs) | afd0cc5 | the fix |
|---|---|---|
| zone 0 (asphalt declared uncertain) | 0 | 2 (sched, always d185j1) |
| zones 1-3 | 2 | 0 |
| ice (zone 4) | 13 | 10 |
| **loose snow R 9 x 90 (zone 5), of the runs reaching it** | **10 of 15** | **4 of 18** (-0.44, se 0.16: 2.8 se) |
| packed-snow hairpin (zone 6), of the runs reaching it | 2 of 5 | 11 of 14 |
| later (mud, zone 8) | 1 | 0 |
| finished | 2 (single d170j2 158.1 s, always d185j6 157.0 s) | 3 (single d200j6 155.9 s, sched d150j3 155.8 s, always d170j2 152.5 s) |

Zone 5 by arm: single 7 of 8 -> 1 of 7, sched 2 of 3 -> 3 of 7, always 1 of 2 -> 0 of 4. The
car now enters the snow with regen 1000-1170 N and no slip event in every traced draw. The
chain's first stopper is the packed-snow hairpin (M4f's mechanism, above), then ice.

afd0cc5 (its 0.85 s start budgets) moved the chain from issue 013's runs: zone 0 stops 10 -> 0,
ice 3 -> 13 (the multistart arms: sched 5, always 6); not this issue's.

Unchanged, bit for bit (same result, time, re-plans, digests): one corner each of ice
(`c:ice:9:90:6`), wet mud (`15:180:6`), loose snow (`9:90:6`), gravel (`15:180:15`), packed
snow (`25:180:15`), single and sched, d170j5; Haruna / Tsukuba `RP_DET` single and sched
76.28 / 74.33 s, digests 9524862c / 9160b94d, d7ca9f95 / 82bd3c28. Planner suite 209 passed
(before the cut-warm test, which passes alone).

## Left

- The packed-snow hairpin (zone 6): M4f's mechanism, the plan's front far past its peak.
- The transition windows' 150-390 iterations (the cause not found: not the road change, the
  hold model, the slip rows or the window's length); the cut warm start carries them across
  now, a cap of 150 still cuts them.
- The ECU reads loose snow's grip at 0.135 of 0.23 when it does slip there (it measures during
  its own release, and the rear never drops under the reapply slip to raise it): car-side, not
  touched (the plan now avoids the slip).
- Rulings: planner side, not the car (the planner's model assumes no hold on snow; the plan
  can keep that true, the car cannot know the road changed until it slips); the arrival bound
  on non-held roads only (on ice's arrival it cost 2 of 4 draws); the cut warm start by window.
