# Drift style M2: S2 `drift_budget`, the signed drift map and its first closed loop

(Roadmap 065 M2, 2026-11-01. Design: [drift style layer](superpowers/specs/2026-10-31-drift-style-layer-design.md)
4.3, section 8 call 3. Results under swb's `play_log/race_planner/style/m2/`.)

**S2 runs.** A `drift_budget` record now drives the scheduled planner: the lap budget
`eps_lap_s` is spent offline by a knapsack over the lap's corners, priced from a new signed drift
map; the gate adopts a bought corner's drift when the live window says it costs no more than its
price plus 0.3 %; a lex-ε gate (`eps_window`) serves classes without fall-off; an account is
debited by the map's price, once a corner, never credited. L1 stays minimum time. S1 is bit for
bit 061's arm (Haruna 76.28 s, Tsukuba 74.33 s, digests unchanged).

**The main finding (E2): a drift that loses time has no valley of its own.** Over 152 cells
(8 classes, R 9-60 m, 90/180 deg, widths 6-15 m), every drift the free window found from any start
was faster than grip (signed loss -0.08 to -0.97 %). Where a drift does not pay, every drift seed
slid back to grip. So "drift where it costs a little" cannot be bought from the existing seeds: it
needs the drift held. `forced.py` holds it (the rear past its peak + 3 deg, rounded up to a degree,
from turn-in to the corner's middle) using the window's own held-slip rows with a negative bound,
no new solver rows and nothing loosened. Forced, a drift costs 0.02-0.05 s a corner on asphalt
(+0.27 to +0.69 % of the corner section), up to 0.23 s on packed snow. A whole Haruna lap of them
is predicted at 0.23 s (0.30 %).

**Closed loop (E3, sim.Plant, RP_DET): the budget holds, the drift does not grow.** All 36 S2 runs
that finished stayed within budget + 0.3 % of S1 (Haruna +0.05 to +0.39 s, Tsukuba -0.05 to
+0.19 s). No run added a stop, spin or off-road against its S1. But on asphalt the plant barely
slides (0-28 m a lap, rear at most 5.2 deg past its peak), and the live gate measured the forced
drifts at 2-4 times their map price, so most bought corners were never entered. On the gravel,
grass, dirt and packed-snow corners S2 was **faster** than S1 (-0.04 to -0.08 s): it takes drifts
that the map says are free and that S1's 0.3 % switch threshold refuses.

## What was built (autoware-drift, `src/planning/race_planner`)

| piece | file | what |
|---|---|---|
| signed map | `budget.py` `LossMap`, `race_planner/drift_losses.json` | per class and cell (R, turn, width): drift - grip time over the study's corner section (s; < 0 a gain), the share of the corner's zone the drift slides, and whether it is forced. A corner is priced conservatively: the cells bracketing its radius and turn, the largest loss and the least share; none outside the measured radii by more than 1.25x, none if a bracketing cell has no drift |
| knapsack | `budget.knapsack` | exact 0/1 over the loss in ms; value = drift metres (share x zone length); a gain corner is taken free and credits nothing. Per run; per lap on a circuit driven as laps (`MultiStart(lap=)`, which `serve.py` passes) |
| account | `budget.Account` | the budget, the knapsack's corners reserved, each corner debited once at entry by its price (> 0 only) |
| lap plan | `budget.S2Lap` | each corner: `budget` (bought), `lex` (eps_window set, every probable class flat, not bought), or S1's |
| gate | `multistart.MultiStart._choose_s2` | `budget`: the fastest drift plan entered when it costs at most price + 0.3 % of the grip plan's window time on two re-plans, left beyond that + `exit_hyst` on two; `lex`: among plans within (1 + eps) of the fastest, the one that drifts most (D_k: metres `drift_past_deg` past the peak in the corner's zone; faster on a tie), while the account can pay; left outside (1 + eps + h) on two; else `_choose_s1`, today's gate moved unchanged |
| forced drift | `forced.py`, slot `B` | a seed process with one window a floor angle; into a bought corner (and while its own plan still slides there) B solves the D seed with the floor on. Built only when S2 is the style at start |
| schedule | `MultiStart._replan_scheduled` | S2 adds H and D into a corner it may buy or lex (outside AHEAD), and B into a bought one |
| refusals | `live.Driver.accept_style`, `MultiStart` | S2 without the scheduled starts refused by name (`'name': drift_budget needs the scheduled starts ...`); S3 still `NotImplementedError` |
| offline | `studies/drift_optimality.py` `s2-signed`, `s2-forced`, `drift-losses` | the grid, its forced arm, the map |
| harness | `studies/mode_choice_m4.py` | the plant's drift (metres and share 3 deg past the peak on the true surface, per corner); S2's plan, debits and gate records in the result; `pareto`, `s2-report` (the one-class courses `<course>@<class>` and `--traj` are d7b6dac's, landed beside this) |

Tests: `test_budget.py` (8: lookup bracket and refusal, alias, width, the knapsack exact against
brute force and monotone, the account), `test_s2_gate.py` (15: the gate's rules, hysteresis, one
mode change at the edge, account limits, S1's chooser untouched, `S2Lap.build`, and two driver
runs: a bought corner scheduled with D and B, B's plans drift; nothing bought with no budget),
`test_forced.py` (3: refusals, rounding, the floor holds and costs and opens to the free window),
`test_style.py` updated (S2 refused without the schedule, the runtime switch test now uses S3). Full suite:
181 passed (17 min, load 7-20).

## E1 again: S1 is bit for bit

`RP_DET=1`, `--arm sched --digest`, this branch, no record and the `fastest` record:

| course | plans / trajectory digest | lap | 95b902e (M1's table) |
|---|---|---|---|
| Haruna | `9524862c` / `9160b94d` (both arms) | 76.28 s | same |
| Tsukuba | `d7ca9f95` / `82bd3c28` (both arms) | 74.33 s | same |

`play_log/race_planner/style/m2/s1id/`.

## E2: the signed map

`s2-signed`: each cell solved as the drift study does (ds 2, 360 m, grip 1, tol 1e-6), with the
live slip caps (front 1x, rear 3x the peak on classes without fall-off): held (grip) from cold and
from the best grip plan, free from cold, from the held plan, from Ds and, into a hairpin, F. A drift
is a plan whose rear runs 3 deg past its peak in the corner's zone (multistart's). `s2-forced`:
every cell with no drift valley, solved from Ds and F with the floor on.

| class | cells | drift valley (all gains) | gain % | forced (no valley) | floor held | forced loss % | forced loss s |
|---|---|---|---|---|---|---|---|
| asphalt | 28 | 12 | -0.48 to -0.09 | 16 | 16 | +0.27 to +0.69 | +0.02 to +0.05 |
| mud_wet | 28 | 28 | -0.97 to -0.16 | 0 | - | - | - |
| snow_loose | 28 | 28 | -0.88 to -0.14 | 0 | - | - | - |
| gravel | 28 | 14 | -0.70 to -0.31 | 14 | 13 | -0.28 to +0.56 | -0.02 to +0.05 |
| snow_packed | 10 | 4 | -0.34 to -0.08 | 6 | 6 | +0.13 to +0.93 | +0.02 to +0.23 |
| sand | 10 | 10 | -0.83 to -0.12 | 0 | - | - | - |
| grass | 10 | 3 | -0.30 to -0.26 | 7 | 7 | +0.15 to +0.73 | +0.02 to +0.11 |
| dirt | 10 | 4 | -0.40 to -0.09 | 6 | 6 | -0.09 to +0.20 | -0.01 to +0.03 |

On asphalt the valleys are the hairpins (R 9-25 x 180 deg), every 90 deg and the R 40-60 hairpins
have none. Four forced cells came out faster than grip (dirt R 9, 15, 60 x 90; gravel R 25 x 90 at
15 m, -0.28 %): there the free solves missed a drift valley the floor found. One gravel cell
(R 9 x 90, 15 m) was infeasible with the floor and has no price. Ice is not run (R5) and never
priced. The asphalt valleys agree with the old map (R 9 x 180 at 8 m: -0.39 % here, 0.40 % gain
there).

**The curve** (`pareto`, the knapsack on each course's corners as the planner sees them; the
budget in s, and as ε % of S1's lap or, for the one-class courses, the quasi-steady lap). Drift
metres are the floor's stretch on a forced corner, the valley's own on a gain one:

| course | 0 s | 0.05 s | 0.1 s | 0.2 s | 0.5 s and up | ε 0.3 % | ε 1-3 % |
|---|---|---|---|---|---|---|---|
| Haruna (asphalt, 76.28 s) | 0 s, 24 m, 1.6 % | 0.026 s, 52 m, 3.4 % | 0.081, 94 m, 6.1 % | 0.168, 131 m, 8.6 % | 0.227, 149 m, 9.8 % | 0.197, 143 m, 9.3 % | 0.227, 9.8 % |
| Tsukuba (asphalt, 74.33 s) | 0, 35 m, 1.7 % | 0.047, 121 m, 5.6 % | 0.090, 160 m, 7.5 % | same | same | same | same |
| zone chain (mixed) | 0, 93 m, 4.4 % | | 0.096, 276 m, 13.2 % | 0.158, 277 m, 13.3 % | same | same | same |
| Haruna @ gravel | 0, 64 m, 4.2 % | | 0.075, 178 m, 11.7 % | 0.157, 236 m, 15.5 % | same | same | same |
| Haruna @ snow_packed | 0, 20 m, 1.3 % | | 0.096, 70 m, 4.6 % | 0.182, 88 m, 5.8 % | 0.368, 128 m, 8.4 % | 0.368 | 0.368 |
| Haruna @ grass | 0, 20 m, 1.3 % | | 0.086, 76 m, 5.0 % | 0.195, 118 m, 7.8 % | 0.256, 135 m, 8.8 % | same | same |
| Haruna @ dirt | 0, 128 m, 8.4 % | | 0.048, 198 m, 13.0 % | same | same | same | same |
| Haruna @ mud_wet, snow_loose, sand | every corner gains: 280 / 263 / 214 m (18 / 17 / 14 %) at 0 s | | | | | | |

(predicted loss s, drift metres, share of the course.) The share grows with the budget until every
corner is bought, which happens at 0.1-0.5 s: the cheapest whole lap of drifts is 0.3 % or less of
its time on every class. ε of 0.3 % or more buys everything. The full table: `pareto.md`.

## E3: S2 in closed loop

sim.Plant (the ECU in the loop), `--arm sched`, `RP_DET=1` (no clock: solves uncut, adoption at the
fixed 1 s latency), one run each. Load 7-40 (other users); irrelevant without the clock.

| course | S1 | S2 0.02 / 0.05 s | S2 0.1 s | S2 0.2 s | S2 0.5 s | plant drift S1 → S2 |
|---|---|---|---|---|---|---|
| Haruna | 76.28 | 76.47 (+0.19) | 76.33 (+0.05) | 76.40 (+0.12) | 76.67 (+0.39) | 0 → 17, 0, 0, 28 m |
| Tsukuba | 74.33 | 74.52 (+0.19) | 74.28 (-0.05) | 74.28 (-0.05) | 74.28 (-0.05) | 0 → 0 |
| c:asphalt:25:90:8 | 6.62 | 6.62 / 6.62 | | 6.62 | | 0 → 0 |
| c:gravel:15:90:8 | 9.23 | 9.14 (-0.08) / same | | same | | 0 → 12 m (8.6 %) |
| c:snow_packed:15:90:8 | 15.74 | 15.67 (-0.07) / same | | same | | 19 → 20 m |
| c:grass:15:90:8 | 9.52 | 9.52 / 9.48 (-0.04) | | 9.48 | | 0 → 0 |
| c:dirt:25:90:8 | 8.51 | 8.45 (-0.06) / same | | same | | 0 → 0 |
| c:mud_wet:25:90:8 | 12.96 | same | | same | same (eps_window 0.01 too) | 17 → 17 m |
| c:snow_loose:15:180:8 | 19.14 | same | | same | same | 34 → 34 m |
| Haruna @ mud_wet, @ snow_loose | off road at s 269, every arm | | | | | |

(time s, vs S1; corner courses timed over the corner's span.) `e3.md` has every run and, per
bought corner, the live gate's last loss against its allowance.

What it shows:

1. **The budget holds**: every finished S2 run is within budget + 0.3 % of S1 (R3's first half).
   No added stop, spin or off-road: the Haruna-geometry mud and loose-snow laps leave the road at
   s 269 in S1 as in S2 (061's low-friction limit at Haruna's speeds, not S2's).
2. **The drift does not grow with the budget on the plant** (R3's second half: not met). On
   Haruna 3, 5, 8, 10 corners are bought at 0.05-0.5 s, but 0-2 are entered: the live window prices
   a forced asphalt drift at 0.05-0.10 s where the map says 0.026-0.029 s (Haruna s 48: 0.085
   against an allowance of 0.049; s 1071: 0.085-0.093; s 1411: 0.051-0.097). The map is priced at
   the study's settings (grip 1, ds 2, the section's exit) and the live window at the planner's
   (grip 0.92, ds 4, 260 m, its end speed), and on asphalt the difference is the whole price.
3. **The plant follows an asphalt drift only partly.** With 25 D and 11 B plans adopted (0.5 s) the
   car slid 28 m, its rear at most 5.2 deg past the peak; Tsukuba at 0.1 s adopted 28 B plans and the
   rear never passed 2.95 deg. L2 tracks the path, not the slide (today's weights; S3 is the style
   that changes them).
4. **Where a drift is nearly free, S2 is faster than S1.** The map buys gain corners at zero cost;
   S1 switches only above 0.3 %. Gravel, grass, dirt and packed snow corners: -0.04 to -0.08 s.
5. **The lex gate did not act**: on the flat classes every corner is a gain, so all are bought;
   `eps_window 0.01` runs equal `eps_lap_s 0.5`. Its rules are covered by the unit tests only.

## Decision points

- **DP0** (user, 2026-11-01): passed for S2's classes (not ice; asphalt held per design); M2 started
  after 061 M4e. Ice stays open on 6 m roads; S2 never prices or buys ice.
- **DP1** (builder's ruling, after E2): **S2 everywhere, offline budget, with the forced drift.**
  Drift is cheap everywhere the floor holds it (asphalt 0.02-0.05 s a corner; every class's whole
  lap <= 0.37 s), so asphalt is not budget-only by cost. But no costly drift exists without the
  floor, so the design's "D where the loss fits" needs `forced.py` (B) live. The lex gate stays for
  classes without fall-off and has nothing to do while every corner there gains.
- **DP2** (after E3, M3's; preliminary): **tune**. Before adopting, (a) price the map at the live
  settings (grip 0.92, ds 4, the live window), since the study's price is a third to a half of
  what the window measures on asphalt; (b) look at L2 on a planned asphalt slide (a separate
  question from S3's heavier weights, but the same mechanism).

## Rulings (builder's, M2)

1. **Forced drift.** A drift S2 buys where there is no valley is the window held 3 deg (`drift_past_deg`)
   past the rear's peak, rounded up to a whole degree, from turn-in to the corner's middle, by the
   held-slip rows with a negative bound (`forced.py`). Hard, not elastic: a floor the car cannot
   hold makes B infeasible, and the gate takes grip (the D seed falls back to grip there anyway).
   It only adds a requirement: no invariant of design section 5 moves.
2. **Price.** The worst bracketing cell's loss and least share; no price outside 1.25x the measured
   radii or where a bracketing cell has no drift. Per corner, the worst probable class.
3. **Allowance.** A bought corner's drift: at most max(price, 0) + 0.3 % of the grip plan's window
   time; left beyond that + `exit_hyst`. The account is debited at entry by the price, gains not
   credited; a lex corner without a price debits the window's measured loss, and only when
   `eps_lap_s` is set does the account limit the lex gate.
4. **Budget per run** on an open course; per lap when the planner is told the lap (`serve.py` on a
   circuit). A style switched to S2 at run time gets a fresh account and no B (B's windows are built
   at the start).
5. **S2 needs the scheduled starts**; refused by name otherwise.
6. **Gain corners are bought free.** S2's corner with a negative price is entered at a tie
   (allowance 0.3 %), where S1 needs 0.3 % faster: this is why S2 beats S1 on near-free corners.

## Video-ready runs

`play_log/race_planner/style/m2/video/` (each `.json` with its `.traj.json`, `--traj`'s, for
`tools/arm_video.py`; RP_DET: each re-run gave its E3 run's plan and trajectory
digests bit for bit, S2's as S1's):

| pair | S1 | S2 |
|---|---|---|
| Haruna, dry asphalt | `haruna__sched_r1_s1` 76.28 s, no slide | `haruna__sched_r1_s2b0.5` 76.67 s, 28 m of slide (s 1195, 1411) |
| gravel R 15 x 90, 8 m | `c_gravel_15_90_8__sched_r1_s1` 9.23 s, grip | `c_gravel_15_90_8__sched_r1_s2b0.05` 9.14 s, 12 m drift |
| packed snow R 15 x 90 | `c_snow_packed_15_90_8__sched_r1_s1` 15.74 s | `c_snow_packed_15_90_8__sched_r1_s2b0.05` 15.67 s |
| Tsukuba (S2 only) | the e3 run | `tsukuba__sched_r1_s2b0.1` 74.28 s, 28 forced plans adopted, no slide on the plant |

## Left

- M3 proper: the map priced at the live settings; S2 with L2 that holds an asphalt slide; E3 three
  repeats and timed runs on an idle host (p99 adoption <= 1.0 s is unmeasured: the det runs have no
  clock); the lex gate in a closed loop where a flat-class corner is not a gain (none in the map
  today).
- E3 on the zone chain (061's off-road at s 1236 in every arm stands in the way).
- Mode changes per corner counted per corner (the run records have them; today's report counts
  per run).
