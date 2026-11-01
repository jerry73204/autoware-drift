# Drift style M3: S2 live, priced as the gate sees it

(Roadmap 065 M3, 2026-11-01. Design: [drift style layer](superpowers/specs/2026-10-31-drift-style-layer-design.md)
4.3, section 8 call 3; M2: [drift-style-m2.md](drift-style-m2.md). Results under swb's
`play_log/race_planner/style/m3/`. Every closed-loop number here is sim.Plant with the ECU at
sim_vehicle_ecu d68637a, the superproject's checkout, `SIM_VEHICLE_ECU` unset.)

**S2's drift now grows with its budget on the plant.** On Haruna the car drives 5, 36, 36, 40 m
of drift a lap (rear 3 deg past its real peak; 11, 45, 45, 51 m past the planner's) at budgets of
0, 0.1, 0.2, 0.5 s, against 0 m for S1, and the lap's mean loss stays inside the budget plus 0.3 %
at every budget. M2's map did not get there for three reasons, all found and fixed: the map was
priced at the study's settings and geometry, not the gate's (the live gate measured 2-5 times the
price, so it refused most bought corners); the forced floor was 3 deg past the planner's peak,
which is only 1.9 deg past the car's real one on asphalt; and the gate carried a drift from one
corner into the next without its entry check or its debit (issue 010). L2 was not the cause: it
holds a sustained planned slide within about 1 deg. R6 is not shown: the forced starts raise L1's
iterations a re-plan by about 30 % on Haruna, and no timed run was possible (load 15-180 all day).

## 1. The price, at the live settings

M2 priced a drift on the study's synthetic corner (grip 1, ds 2, 360 m, tol 1e-6, read 50 m past
the exit). The live gate compares two window plans at the planner's settings. Pricing a cell as the
gate sees it (`drift_optimality.py s2-live`, `live_positions`): a live.Driver on the section (its
window length, which is the braking distance from the course's top speed plus a corner, 200-312 m;
its slip caps only where a surface has no fall-off; `v_end` its L0 ceiling), the car on the
course's grip plan (`grip_plan`: H's windows chained), and from 45, 75 and 105 m before the turn-in
the live starts G, D (Ds and the library's Dc), B (the floor) and H, classified as the gate does.
The price is the largest loss over the three positions (the gate needs the drift inside its
allowance on two re-plans in a row anywhere on the approach), the share the least.

What it found:

| | study (M2) | live settings, flat cell | the course itself, live |
|---|---|---|---|
| asphalt R 25 x 90, 8 m | 0.024 s | 0.076 s (16 deg floor) | 0.08-0.10 s (`c:asphalt:25:90:8:160`) |
| asphalt R 25 x 90, 7 % downhill | | 0.057 s (14 deg floor) | |
| Haruna hairpins (R 10-13, 158-185 deg) | +0.029 s | R 9-15 x 180: -0.01 to -0.09 (gains) | -0.003 to -0.058 s (gains) |
| Haruna s 1071 (R 37, 33 deg kink) | 0.026 s (from 90 deg cells) | 45 deg cells: 0.03-0.11 s | 0.15-0.93 s |
| Haruna s 1195 (R 19, 133 deg) | 0.029 s | | 0.060-0.088 s |
| Haruna s 1411 (R 12, 152 deg) | 0.029 s | | 0.114-0.148 s |
| mud R 15 x 90, 8 m | -0.034 s (gain) | +0.030 s | +0.041 s (`c:mud_wet:15:90:8:160`) |

So the settings explain about a factor of two on a flat asphalt corner (most of it the deeper
floor, section 2, and the longer live window that counts the slow exit), and a course's own
geometry (Haruna is 6-8 % downhill all the way, its corners run into each other, the kink is taken
at 30 m/s) explains the rest, in both directions: the downhill hairpins are gains live and losses
in the map. A cell map cannot carry that. **Course prices** (`mode_choice_m4.py course-prices
<course>`, `budget.CoursePrices`, `MultiStart(course_prices=)`, the harness's `--prices`): each
corner of a course priced offline once, as above, from the course's own grip plan; S2 uses a
corner's course price where it has one and the map elsewhere (Haruna's s 48 lies too near the
start to be priced from 45 m back, and kept the map's). This is the offline budget DP1 chose, per
course, as L0 is per course.

The live map itself (`s2-live` over M2's 152 cells and 56 cells at 45 deg, `drift-losses-live`):
see section 6.

Noise: the live window's tol 1e-3 makes a position's loss vary by about +-0.04 s (dirt R 25 x 90:
-0.039, +0.046, +0.086 at the three positions), so the largest of three is conservative by up to
that much. Mud's R 15 x 180 at 15 m priced -2.6 s: its grip plan stalled at one position.

## 2. The forced floor, past the car's peak

`forced.py` holds the rear `drift_past_deg` past the peak. M2 took the peak at the planner's
friction (grip 0.92 x the map's). On asphalt the planner's tyre peaks at 10.85 deg, the car's at
12.1 deg (the peak grows with load transfer, 1 + 0.6 mu): the floor was 14 deg, 1.9 deg past the
car's real peak, and the plant's drift measure (3 deg past the real peak) counted none of it. The
floor is now at the map's friction (`MultiStart._floor`: `c.mu / grip`): 16 deg on asphalt, 19 on
gravel. S2 only; S1 never builds B. The harness now reports the plant's drift both ways
(`plant_drift_m` past the real peak, `plant_drift_m_planner` past the planner's, by which a plan is
called a drift).

## 3. L2 and the planned slide (item 2)

Traced every tick of Haruna at 0.5 s (M2's arm) and the gravel corner: the plan's rear slip at the
car's s against the car's (scratch traces, summarised here). Per bought corner where the plan was 3
deg or more past the planner's peak:

| corner | planned past, m | car - plan rear slip, mean (p5, p95) deg |
|---|---|---|
| s 1195, a sustained B slide, 28 m | 27 | -0.5 (-1.2, +1.3) |
| s 971, s 503 | 7-10 | -0.9 to -1.4 |
| s 867, s 1411 | 15-20 | -2.1 (to -5) |
| hairpins s 276, 376, 600: short spikes | 4-10 | -3.9 to -5.4 |
| gravel R 15 x 90, a free G slide to 18 deg | 24 | car 17.3 deg against 17.9 |

L2 holds a sustained planned slide within about 1 deg. It lags short spikes (4-10 m, a few tenths
of a second: the car's yaw cannot follow), and once (s 1411, M2's arm) it dropped a slide after the
car fell 2.6 m/s behind the plan's speed, steering out of it. Weight changes on the whole lap (the
slide's scales x0.5, so x4 weight; the line's 0.2 to 0.5 m) moved single corners both ways (s 275:
-3.9 to -1.0 deg; s 503: -1.4 to -8.0) and changed no outcome consistently. So the "5.2 deg at
most" of M2 was the plans (asked 14-18 deg, 2-6 deg past the real peak, mostly as short spikes)
and the measure (section 2), not L2. **No L2 change**; S1 is untouched.

## 4. The gate across corners (issue 010)

The gate's mode spans corners (S1's rule: no corner ahead, every plan keeps the mode), but S2's
entry and debit were per corner and only from grip. A drift entered at one corner carried into the
next bought one was neither checked against that corner's allowance nor debited: on Haruna at 0.5 s
three of five drift corners went unpaid (account 0.058 s spent against 0.146 s driven), and a B
plan over its allowance was kept by the exit band. Fixed: `_choose_s2` records the corner its drift
was entered for; a drift carried into another bought or lex corner is kept only if it meets that
corner's entry rule, then debited (`why` "... carried"), else left on two re-plans.
[Issue 010](issues/010-s2-drift-carried-into-the-next-corner-unpaid.md).

## 5. E3: S2 in closed loop, three draws

sim.Plant, `--arm sched`, `RP_DET=1`, three draws a run (`--jit` 0, 1e-9, 2e-9: a last-bit start;
the closed loop is chaotic, S1's Haruna lap is 76.28 / 76.67 / 76.53 s over the draws). Corner
courses `c:<class>:<R>:<turn>:<width>:160`: a 160 m entry (`course`'s new fifth field), since on the
study's 60 m entry the gate saw a bought corner once and could never enter it (two re-plans needed:
M2's asphalt corner showed loss 0.020 against 0.052 allowed, once). S2 on course prices. Losses are
against S1's draw with the same jit; "within" per draw and for the means.

| course | budget s | time s (mean) | vs S1: mean, spread | within budget + 0.3 % | plant drift m: mean (range); planner's | bought / entered | spent s |
|---|---|---|---|---|---|---|---|
| Haruna | S1 | 76.49 | | | 0 (0-0); 6 | | |
| | 0 | 76.62 | +0.13, 0.20 | 2 of 3; mean yes | 5 (0-9); 11 | 6 / 5-6 | 0 |
| | 0.1 | 76.71 | +0.21, 0.24 | 2 of 3; mean yes | 36 (18-54); 45 | 7 / 6-7 | 0.088 |
| | 0.2 | 76.71 | +0.21, 0.24 | yes | 36 (18-54); 45 | 8 / 6-7 | 0.088 |
| | 0.5 | 76.95 | +0.45, 0.38 | yes | 40 (30-54); 51 | 9 / 7-8 | 0.236 |
| | 1.0 | 76.95 | +0.45, 0.38 | yes | 40 (30-54); 51 | 9 / 7-8 | 0.236 |
| asphalt R 25 x 90 | S1, 0-0.1 | 9.50 | 0 | yes | 0 | 0 | |
| | 0.2 | 9.50 | -0.00, 0.20 | yes | 6 (0-16); 15 | 1 / 1 | 0.102 |
| dirt R 25 x 90 | S1, 0, 0.05 | 12.24 | 0 | yes | 0 | 0 | |
| | 0.1, 0.2 | 12.17 | -0.07, 0.33 | yes | 28 (11-53); 34 | 1 / 1 | 0.086 |
| grass R 15 x 90 | S1, 0-0.1 | 13.80 | 0 | yes | 0 | 0 | |
| | 0.2 | 13.96 | +0.16, 0.04 | yes | 15 (15-15); 18 | 1 / 1 | 0.146 |
| gravel R 15 x 90 | S1, 0 | 13.00 | | | 0 | | |
| | 0.05-0.2 | 12.82 | -0.18, 0.04 | yes | 0; 0 | 1 / 1 | 0.045 |
| packed snow R 15 x 90 | S1, 0-0.2 | 23.63 | 0 | yes | 20 (20-20); 21 | 0 (unpriced) | |
| mud R 15 x 90 (160), R 25 x 90 (100) | every arm, S1 too | off road 3 of 3 | | | 8-9 | lex: 1 entered, 0.041 / 0.032 | |

(`e3.md`, `e3.json`; Haruna's s 1071 kink, at 0.93 s, is never bought at 1 s with the rest.)

1. **The budget holds** for every mean, and for every draw at 0.2 s and above; at 0 and 0.1 s one
   Haruna draw in three is over by 0.04-0.06 s, inside S1's own draw spread (0.39 s). R3's 0.3 %
   (0.23 s on Haruna) is smaller than the plant's chaos, so a single draw cannot show it; three
   draws' means can.
2. **The drift grows with the budget**: Haruna 5 -> 36 -> 40 m (saturates once the costly corners
   are bought; the kink is never affordable); each corner course goes from 0 to its drift at the
   budget its course price fits (asphalt at 0.2 s, dirt at 0.1 s, grass at 0.2 s). Haruna's growth
   comes mostly from s 1195 (9-36 m) and s 1411 (12-20 m), the forced asphalt corners M2 could not
   enter.
3. **Gravel** is 0.18 s faster than S1 with its corner bought, but slides 0 m past either peak:
   the plans entered are B's and G's, the car's rear stays at 17 deg, 1.5 deg past its real peak.
   **Packed snow**'s corner is unpriced (no drift at 45 m back): S2 = S1.
4. **No added stop, spin or off-road.** Mud leaves the road in every arm, S1 too. This is new with
   the rebase onto 474a9bf (issue 009's 0c61d28): before it, S1 finished `c:mud_wet:25:90:8:100`
   3 of 3 (15.60 s) and `c:mud_wet:15:90:8:160` 2 of 3; after, 0 of 3 each (the other courses' S1
   runs are bit for bit the same before and after). So the lex gate's closed loop (item 4) ran,
   entered and debited its corner (0.041 / 0.032 s), but its effect on a finished corner is not
   measured.
5. **Mode changes** at most 1 a corner in every S2 arm.
6. **L1's cost (R6), unclocked**: per re-plan, the slowest start's IPOPT iterations, p95 over the
   run: Haruna S1 150-158, S2 158-229 (2.5-2.9 starts a re-plan against 1.0, B the slowest; the
   400-iteration cap hit in one draw at 0.1-1.0 s); corner courses +20-160 %. At M4d's 5.8 ms an
   iteration that is about 0.9 s for S1 and 1.0-1.3 s for S2. No timed run: the host's load was
   15-180 all session (other users), so adoption p99 and L1 p95 are not measured.

S1 bit for bit, after the rebase onto 474a9bf: Haruna `9524862c` / `9160b94d` 76.28 s, Tsukuba
`d7ca9f95` / `82bd3c28` 74.33 s, no record and `fastest` (`s1id/`, `s1id_rebased/`).

## 6. The live map

`race_planner/drift_losses.json` is rebuilt from `s2-live` (`drift-losses-live`): M2's 152 cells
and 56 at 45 deg (8 classes, R 9-60, 45/90/180 deg, 6-15 m on the width classes), each priced at
the live settings as in section 1, the floor past the car's peak. It is the price for any corner a
course has no price for. The E3 runs above were made with course prices; Haruna's s 48, the one
corner without one, used M2's map then.

| class | cells | priced | gains | gain s | losses | loss s | forced |
|---|---|---|---|---|---|---|---|
| asphalt | 38 | 36 | 8 | -0.09 to -0.01 | 28 | +0.056 to +0.186 | 25 |
| mud_wet | 38 | 30 | 12 | -2.64 to -0.02 | 18 | +0.019 to +0.090 | 9 |
| snow_loose | 38 | 30 | 11 | -0.48 to -0.03 | 19 | +0.025 to +0.108 | 8 |
| gravel | 38 | 36 | 10 | -0.13 to -0.06 | 26 | +0.031 to +1.608 | 25 |
| snow_packed | 14 | 13 | 1 | -0.09 | 12 | +0.103 to +5.013 | 12 |
| sand | 14 | 12 | 6 | -0.23 to -0.03 | 6 | +0.035 to +0.090 | 2 |
| grass | 14 | 11 | 2 | -0.11 to -0.03 | 9 | +0.126 to +0.248 | 8 |
| dirt | 14 | 14 | 3 | -0.08 to -0.02 | 11 | +0.067 to +0.132 | 9 |

Against M2's map: forced asphalt drifts cost 2-4 times more (the deeper floor and the live
window); the hairpins (180 deg, R 9-15) stay gains; **the flat classes (mud, loose snow, sand) now
have losses** (90 deg and 45 deg corners, +0.02 to +0.11 s), so the lex gate has corners to act
on (item 4, section 5's mud runs). Outliers come from a grip plan that stalled at one position:
packed snow R 9 x 90 +5.0 s, gravel +1.6 s, mud R 15 x 180 at 15 m -2.6 s. A large price only
stops a purchase; a false gain is bought free but still entered only within 0.3 % live, so neither
loosens the budget. Cells with None had no drift at one of the three positions.

## Decision point

**DP2** (builder's ruling, for the user): **tune, then adopt.** S2's live gate now does what R3
asks on the plant (budget held on the means, drift growing with the budget, nothing added to stops
or off-roads) with course prices, the floor past the car's peak and issue 010 fixed. Two things
stand before adoption: (a) R6: B adds iterations a re-plan (p95 +30 % on Haruna, the cap reached);
a timed E3 on an idle host must show L1 p95 <= 0.9 s, or B needs its own iteration budget; (b) the
lex gate's closed loop on a flat class (mud) is unmeasurable while S1 itself leaves the road there
after 0c61d28. Dropping the live gate (offline budget only) is not indicated: the gate is what
refuses the corners whose live cost exceeds their price, and it is why the budget holds.

## Rulings (builder's, M3)

1. **Price as the gate measures.** A drift's price is the live windows' loss (the gate's settings,
   from three approach positions, the largest), not the study's section time.
2. **Course prices before the map.** A course priced offline uses its own corners' prices; the map
   (now at the live settings) prices the rest.
3. **The floor is past the car's peak** (the map's friction), not the planner's.
4. **A drift is paid per corner** (issue 010): carried into a bought corner it must meet that
   corner's allowance and is debited there.
5. **R3 by means over three draws**, per-draw results reported beside them: the plant's chaos
   (0.4 s on Haruna) is larger than R3's 0.3 %.
6. **No L2 change for S2.** L2 holds a sustained slide; short spikes lag by the car's yaw, not the
   weights.

## Left

- Timed E3 on an idle host (R6, adoption p99).
- B's iterations: its own iteration budget or a warm start from the last B plan on the next corner.
- The lex gate on a flat-class corner that finishes (mud leaves the road in S1 since 0c61d28;
  tell issue 009's owner).
- Packed snow: priced None at one position; a finer position set or the map's price there.
- Course prices for Tsukuba and the zone chain (061's off-road at s 1236 still stands).
