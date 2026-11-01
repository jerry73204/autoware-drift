# Grip or drift, model in the loop: the gate re-scored on the current car (roadmap 061 M4i)

Written 2026-11-01. M4h (`docs/mode-choice-m4h.md`) re-scored roadmap 061's gate on 95b902e and
the old car and did not pass it: issue 012 (L2's low-friction line weight) stopped the class-zone
chain and a mud hairpin. M4i re-scores the gate after issue 012's fix, on autoware-drift
**96d781b** (main: M4g's hold on ice, 065 M2-M3's S2, issue 012's `LINE_SLIDING`) and the
current car, sim_vehicle_ecu **760cd78** (issue 009's regen slip control with issue 011's fix).

Results: swb `play_log/race_planner/mode_choice/m4i/` (`digest/` the dry courses under `RP_DET`,
`ice/` and `packed/` the corner suites, `chain/` and `wrong/` the class-zone chain, `traj_runs/`
the video runs) and `i012/q/` (wet mud, loose snow, gravel and the true-map chain on the same
code: issue 012's final form, run as a study parameter equal to the default; the three chain
draws repeated in `m4i/chain/` came out the same to the bit). Every result has its
configuration in its JSON (`det`, `itercap`, `jit`, `tag`, `ecu`, `regen_hold`, `ecu_primed`).

## Method

- **Pinned inputs.** The planner from a worktree of autoware-drift main (each run asserts it
  imported that tree); swb's inputs from a frozen copy (`surface_classes.yaml`,
  `drift_mustang.yaml`, the course data) with sim_vehicle_ecu a worktree at 760cd78 (the
  superproject's checkout, unmoved during the session; another session was working on
  sim_vehicle_ecu). Check: Haruna and Tsukuba `RP_DET` single 76.28 and 74.33 s, plan and
  plant digests equal to M4e's and M4h's (9524862c / 9160b94d, d7ca9f95 / 82bd3c28).
- **Repeatable runs only** (as M4d-M4h): `RP_DET=1 RP_ITERCAP`, each budget a draw, with a
  last-bit start (`--jit`). Ice: 5 budgets (150, 160, 170, 185, 200) x 3 arms x 12 corners,
  `--jit 9e-9` (draws M4e and M4h did not use); wet mud, loose snow, gravel: 3 arms at 170 and
  single at 150 and 200 (`--jit 5e-9`); packed snow: the same; the true-map chain: 5 draws an arm
  (150/7, 160/9, 170/5, 185/6, 200/8); the 10 % wrong maps: 8 seeds x 3 arms at 170. Outcomes
  pooled per draw, mean and Welch standard error; a difference under 2 se is noise.
- **No wall clock.** The rule is a 1-min load under 6 with nothing else of mine running; the
  host was at 12-80 the whole session (other users' CARLA and Python, another session's Rust
  builds). The dry courses' `RP_DET` runs are M4h's to the bit (digests above), so M4h's
  wall-clock laps (morning window, load 3.9-7.5) are of the same plans and stand; the corner
  suites and the chain are judged on repeatable runs, as in M4h.

## Gate

| item | target | M4h (95b902e, old car) | M4i (96d781b, car 760cd78) | |
|---|---|---|---|---|
| R6 Haruna | sched within 0.1 % of single | wall clock sched +0.063 % (se 0.033 %), 6 laps an arm; `RP_DET` sched = single bit for bit | `RP_DET` sched = single bit for bit (76.28 s, 1.00 solves a re-plan, digests M4h's); always 77.07; wall clock not re-run (load) | **met** (M4h's laps, same plans) |
| R6 Tsukuba | the same | wall clock equal means, one sched lap of six off at s 1820 (issue 005's place, load 6.2) | `RP_DET` sched = single bit for bit (74.33 s); always 73.98 | **met on time**; M4h's lost lap is the clock's |
| low friction: stops, multi-start against single | fewer stops and failed re-plans | ice 9.52 / 9.58 / 9.14 of 12, failed re-plans 19 / 15 / 15 %; mud 11 / 11 / 11 (issue 012); gravel, loose snow 12; packed 12 | **ice 11.60 / 11.80 / 12.00 of 12** (5 draws each, sd 0.55 / 0.45 / 0), failed re-plans **15.3 / 10.4 / 10.2 %**; **mud 36 of 36 / 12 / 12**; loose snow, gravel 36 / 12 / 12; packed snow 35 of 36 / 12 / 12 | **met** (sched - single on ice +0.20, se 0.32, 0.6 se: equal on stops, fewer failed re-plans) |
| low friction: time against single | no worse | mud -0.42 %, gravel -0.32 %, ice +0.05 %, loose snow +1.43 % (one corner) | sched / always on corners both finish: mud -0.26 / -1.26 %, gravel -0.58 / -0.48 %, ice +0.40 / +0.80 % (58 corners, medians 0.00), packed -0.73 / +1.49 %, loose snow +1.54 / +1.54 % (medians 0.00; all of it R 9 x 180 at 6 m, +16.8 %: two F adoptions, the car 27 deg against the plan's 33) | met but loose snow's one corner (as M4c, M4h) |
| sched at least as good as always, fewer solves | | ice 9.58 against 9.14; solves 2.00 / 2.35 | ice 11.80 against 12.00 (-0.20, se 0.20, 1.0 se); solves a re-plan ice 2.00 / 2.22, mud 2.16 / 2.24, loose 2.17 / 2.23, gravel 1.07 / 2.24, packed 2.01 / 2.22, chain 2.72 / 3.26, Haruna 1.00 / 3.28 | **met** |
| the true-map chain finishes | (M4c: 4 of 4) | **0 of 20** | **0 of 15** (5 draws an arm). Stops: zone 0 (asphalt declared uncertain) 4, the multi-start arms' d185 and d200 draws; ice (zone 4) **7 of the 11 runs reaching it** (issue 013); loose snow R 9 x 90 (zone 5) 3 of 4; packed-snow hairpin (zone 6) 1 of 1. Zones 1-3 (mud, loose snow, gravel): 0 of 11 (main before the fix: 8 of 11) | **not met (issue 013 first)** |
| 10 % zones wrong: sched stops no more than always | | every run stops; at the wrong zone sched 3 of 3, always 2 of 2 | every run stops (24 of 24); at the wrong zone (seeds 1 and 6: the ice zone misdeclared) sched 2 of 2, always 2 of 2, single 2 of 2; the other 18 in true-mapped zones, 16 of them in the ice zone (issue 013) | met by the letter, **not testable** while the true map stops in zone 4 |
| the chain's mud hairpin, mapped right | | repeatable 0 of 36; wall clock 10 of 56 | repeatable 0 of 35 (true map 11, wrong maps 24); wall clock not run | met on repeatable runs |

## Findings

### 1. Issue 012 fixed: the line gives way in full only where the tyre falls off

Root cause and fix in `docs/issues/012-line-weight-costs-the-chain-and-a-mud-hairpin.md`. On a
road whose tyre keeps its force sliding (wet mud, loose snow) issue 007's loosened line lets the
car drift from the plan until the edge rows take over and force the rotation at the edge; the
0.2 m line drives the front past its peak in a tight mud hairpin (007's loop). Half of the
loosening on those roads (`LINE_SLIDING` (0.8, 1.2)) finishes both: wet mud's 12 corners in every
arm (main 11), the chain's mud and loose-snow zones 0 of 11 stops (main 8 of 11), packed snow
and ice to the bit, Haruna and Tsukuba to the bit.

### 2. Ice: the corners are no longer a problem; the chain's ice zone is

On the current car (issue 009's regen slip control) ice's 12 corners finish 11.6-12.0 of 12 in
every arm (M4h's old car 9.1-9.6); 3 runs of 180 lost (R 15 x 180 at 6 m twice, R 9 x 180 at 15 m once). The extra starts change
nothing on stops (sched +0.20, always +0.40 against single, both under 2 se) and fail a third
fewer re-plans.

The chain's ice zone, entered braking at 19-20 m/s straight off the asphalt, stops 7 of the 11
true-map runs that reach it and 16 of the 17 wrong-map runs that reach it truly mapped (issue 013,
new): the rear wheels at 4.5-7.4 times ice's peak slip for half a second as the car brakes onto
the ice, the car yaws, L2 steers 16-30 deg against the plan's 2 and lifts off the brake, and the
car runs 1-2 m/s fast through the corner. The ECU at f3f5dcf gives the same outcomes, at d68637a
(no regen slip control) worse (6 of 6); an ECU primed with ice's grip at the start the same (the
priming is forgotten in the asphalt braking before the zone). Not root caused.

### 3. The chain's other stops

- **Zone 0** (asphalt declared uncertain, s 98.8): the sched and always arms' d185 and d200 draws,
  on main and on the fix alike, and on 95b902e with the old car (the same draw): the plan slides
  15 deg at 16 m/s on the zone's lowered friction (0.70), the car on the true asphalt does not
  rotate (2-5 deg) and leaves at the lock. M4h saw it only in its `LINE_MU = 0` draws: a last-bit
  outcome of the uncertain zone's drift plan. Not filed as an issue yet (one cause, the plan
  drifting on a friction the road does not have, is the uncertain zone's design question).
- **Loose snow R 9 x 90** (zone 5, s 1071-1075): the plan's flick (its sideslip 15, 3, 8 deg) is
  not followed; the car arrives 4 deg against 15 and then slides 25 deg against 11. The same
  corner alone at 8 m (`c:snow_loose:9:90:8`) finishes in every draw and line weight: the state
  the car brings out of the ice zone.
- **Packed-snow hairpin** (zone 6, s 1209): M4f's mechanism, not the line weight: the plan steers
  41-43 deg (its front at 7-11 peaks) with 3.6 deg of slide, the car follows and L2 reaches the
  lock 1 m out of the plan. Alone (`c:snow_packed:25:180:8`) it finishes in every draw.

## Verdict

**061 does not close.** Every corner-suite criterion is met with margin on the current car (ice
11.6-12.0 of 12 in every arm, wet mud 12 of 12 after issue 012, sched as good as always with
fewer solves, R6 bit for bit as M4h's met laps). It fails where it failed in M4h, the
robustness the reframed gate puts first: the true-map chain finishes 0 of 15, and the wrong-map
criterion cannot be judged. Issue 012 no longer stops it; the ice zone does.

Blockers, by weight:
1. **Issue 013**, the chain's ice zone entered braking from asphalt: 7 of 11 true-map runs, 16 of
   17 wrong-map runs reaching it truly mapped. The rear past its peak at the ice's edge; not root caused (the ECU's regen
   slip control at the asphalt-ice step, L1 braking at the hold's rate before the car has found
   the grip, or L2 at the yaw).
2. **Zone 0 on the multi-start arms** (4 of 10 runs, the same two draws on every code since
   95b902e): a drift plan on the uncertain zone's lowered friction over true asphalt.
3. **The chain past the ice**: loose snow R 9 x 90 (3 of 4) and the packed hairpin (1 of 1, M4f's
   front-scrubbing plans): both finish alone; few runs reach them.
4. **The wrong-map criterion** waits on 1.
5. **Wall clock** not re-measured (load 12-80 all session): R6's laps are M4h's; a latency
   effect on the corners or the chain's mud hairpin (M4h finding 5) is unseen.

## Rulings

- **Named M4i**: M4h's successor, the same method; M5 is the GPU probe. Cost if wrong: a name.
- **No wall-clock runs** (load 12-80; the rule is under 6): R6 rests on M4h's laps, valid
  because the `RP_DET` plans and plant are M4h's to the bit. Cost if wrong: a timing change from
  code that only shows under load (none in the asphalt path: digests equal).
- **Wet mud, loose snow, gravel and the true-map chain taken from issue 012's final runs**
  (`i012/q/`, the same code as 96d781b: the study parameter equals the default; three chain
  draws re-run in `m4i/chain/` came out the same to the bit). Cost if wrong: none found.
- **Issue 012's half share chosen from a sweep** (0, 0.25, 0.5, 0.67, 1 on mud's and loose snow's
  24 corners, single at 170): only 0.5 finished all 24. Cost if wrong: a tuned constant; the
  principled form (the line scaled by what the road's room and the tyre's fall-off allow in the
  horizon) is untried. `mud_wet:15:180:6` finishes in 25.8-28.6 s against 18.98 with the 0.2 m
  line: held, not well driven.
- **Issue 013 filed, not root caused**; the zone-0 stop not filed (a design question of the
  uncertain zone, described above).

## Representative runs (for `tools/arm_video.py`)

`m4i/traj_runs/<key>.traj.json`, all `RP_DET`, the code above:

| course | result name | single | sched | always |
|---|---|---|---|---|
| wet mud R 15 x 180, 6 m (issue 012's corner) | `c_mud_wet_15_180_6__<arm>_r1_d170j5` | 28.63 s | 28.63 | 25.78 |
| ice R 15 x 180, 6 m | `c_ice_15_180_6__<arm>_r1_d170j9` | 22.58 | 22.57 | 22.68 |
| the chain, true map, furthest draw an arm | `zones__single_r1_d185j6`, `zones__sched_r1_d160j9`, `zones__always_r1_d160j9` | off road s 1208.8 (packed hairpin) | off road s 845.6 (ice) | off road s 1072.5 (loose snow) |

Haruna and Tsukuba: M4h's `traj_runs/haruna__<arm>_r1_dig`, `tsukuba__<arm>_r1_dig` (the same
plans and plant to the bit).
