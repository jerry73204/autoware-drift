# Grip or drift, model in the loop (roadmap 061 M4)

Written 2026-10-30. M3b's scheduler picks L1's starts from the window ahead; M4 drives it on
`sim.Plant` behind the ECU (sim_vehicle_ecu) with L2 (the NMPC) tracking, against the single start
(060) and every start always on, on a true map and a wrong one. First, part A: the asphalt rule
tightened (user, 2026-10-30: "we don't pursue drift start for most dry asphalt roads"; approved
"tighten the asphalt rule").

## A. The asphalt rule tightened

**Rule.** On a class that falls off past its peak (asphalt, concrete, gravel, dirt, grass,
packed snow: sliding share below 0.95), a corner gets D (and with it H) only where the drift
map's gain for it (its R, turn and width, as the map is read: a paying cell no wider and turning
no less than 30 deg further, up to 1.25 x its radius) clears the switch threshold, 0.3 %
(`RULES["gain_falloff"]`), not the noise floor (`gain_min`, 0.05 %). Classes without fall-off
(mud, sand, loose snow) keep 0.05 %. H on low friction (the fallback) is unchanged. The camera's
veto reads the map the same way. On asphalt only the R 9 cells clear it (0.32-0.48 % at 6-15 m),
so hairpins up to R ~11 m; gravel's R 9 hairpins (0.44-0.61 %) and its 15 m-wide R 15 (0.35 %);
packed snow's R 9 hairpin (0.34 %).

**Chain check** (`studies/mode_choice_m3b.py`, no plant, three repeats each; cores 8-11 pinned,
host load 4.9-6.3 on 32 cores; results swb `play_log/race_planner/mode_choice/m3b/tightened/`):

| course | config | result (r1/r2/r3) | time (s) | vs single | solves / re-plan | G only | overruns > 1.0 s |
|---|---|---|---|---|---|---|---|
| Haruna | single | done x3 | 76.90 x3 | | 1.00 | 100 % | 0/0/0 |
| Haruna | sched (M3b) | done | 76.85 | -0.065 % | 2.13 | 43 % | 0 |
| Haruna | sched, tightened | done x3 | 76.85 x3 | -0.065 % x3 | 1.73/1.73/1.81 | 59/59/51 % | 1/1/0 |
| Tsukuba | single | done / stopped / done | 72.70, -, 72.70 | | 1.00 | 100 % | 1/4/1 |
| Tsukuba | sched (M3b) | done | 72.75 | +0.138 % | 1.51 | 75 % | 3 |
| Tsukuba | sched, tightened | done / stopped / done | 72.70, -, 72.85 | 0.000 / - / +0.206 % | 1.00 | 100 % | 1/4/4 |

- Haruna: D now goes into four of seven hairpins (R 9.9, 10.6, 11.0, 11.2 m: within 1.25 x the
  R 9 cell), not into R 11.4, 12.0, 13.0, 18.8; a third fewer solves, the same time in every
  repeat (the drift plan at the first hairpins is the time difference, as in M3b).
- Tsukuba: no corner clears the threshold (its hairpins are R 15-18), so the scheduled run is
  060's G alone throughout: bit for bit the single start's plans (tested in M3b). Its +0.206 % in
  r3 and the stop in r2 are the single start's own run-to-run noise: G's solves cut at the 1.0 s
  budget near t 60 s (overruns 1-4 a run in both configs); in r2 both configs stopped there,
  "solves failed", three cut solves in a row. Over repeats the single start itself spans
  72.65-72.70 (M3b and here).
- R6: met on Haruna (-0.065 %, three of three); on Tsukuba the scheduled arm is the single start
  by construction, so R6 holds up to 060's own noise (0-0.2 % between runs, one stop in three at
  load ~5): an R3 budget matter of 060's, not the scheduler's.
