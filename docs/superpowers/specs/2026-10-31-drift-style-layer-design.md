# Drift Style: a Layer That Chooses the Solvers

**Status**: proposed, 2026-10-31. Not yet approved. Roadmap: carla-scenario-bridge
`docs/roadmap/065-drift-style-layer.md`.

**Builds on**: the live planner design (`2026-10-29-live-race-planner-design.md`, revision 3:
L1, L2, section 4.4a's starts and gate, section 5's failure table, section 10). Evidence:
`docs/drift-style.md` and `docs/drift-style-notes/` (competition practice, autonomous drifting,
style layers in other stacks), `docs/mode-choice-m4b.md` (current state).

## 1. Intent

The user, 2026-10-31: "We pursue for laps with shortest time. Later, we may request for strong
performance like D1 GP ... The D1 would require drift on every turn and the car has to maintain
speed. The other is to chase for fast lap but we may change to drift turns in the case that the
drift trades off no or little time." And: "the behavior should be adjustable. The solver can go
fastest lap, drift for little trade off or always making drift if possible even on straight
roads." And: "The style or behavior is like a layer we can decide the combinations" of the
solvers and starts we already have.

So style is a **layer above the existing solvers**. It picks which formulation each existing
piece runs. It does not add a new solver stack.

## 2. Goals, non-goals, anti-goals

**Goals**
- Three styles on one planner, chosen by one record:
  - **S1 `fastest`**: the shortest lap. Today's planner, unchanged.
  - **S2 `drift_budget`**: a fast lap that takes a drift where it costs no or little time,
    up to a budget the user sets.
  - **S3 `drift_max`**: drift wherever the car physically can, corners and straights, keeping
    speed: D1-style.
- A style change takes effect at the next re-plan, without a stop.
- Each style measured on the plant against its own criteria (section 7).

**Non-goals**
- Tandem (lead/chase). Judged scoring (DOSS-style) as a product. A "D1 look" (body angle)
  as a requirement: reported, not required.
- A GPU, a new solver, or a learned policy.

**Anti-goals** (a failure even if the style "works")
- Any style loosening a safety invariant (section 5).
- S1 drifting from today's plans: S1 must be bit-identical to 061's scheduled arm.
- Planned slides on ice in S3.

## 3. Why a layer, not a weight

A drift reward added to L1's lap-time cost cannot set the style. On this non-convex problem a
weighted sum cannot reach every trade-off; no single weight holds across re-plans; a reward on
slip pays the plan to sit on the slip cap or spin (061's issue 002 failure); and a weight
change moves the plan between valleys, breaking warm starts. Every source that defines a drift
well (D1's rulebook, Stanford's controllers, TUM's orchestration command) defines it as
**constraints and a task chosen upstream, with the optimiser left to go as fast as it can
inside them**. Our planner was already built that way for grip or drift: seeds enumerate the
modes, IPOPT refines each. The style layer is a different selection and constraint set over the
same machinery. **The L1 objective stays minimum time in every style.**

## 4. The design

### 4.1 The layer

```
                      STYLE RECORD  (fastest | drift_budget | drift_max)
                      validated at load; read each re-plan; a change applies at the next
                      re-plan, never within 25 m of a committed drift zone
   .-----------------+----------------+-----------------+----------------+---------------.
   v                 v                v                 v                v               |
OFFLINE / L0     SCHEDULE         L1 WINDOW         GATE              L2 TRACKER        |
drift_map.json   schedule.py      window.py         MultiStart        tracker.py        |
S1 gains         S1 4.4a table    S1,S2 today's     S1 >0.3% x2,      S1,S2 today's     |
S2 + signed      S2 + D where     rows              grip on tie       S3 x3-5 on beta,  |
   losses           loss fits     S3 + elastic      S2 lex-eps,       r in zones; soft  |
S3 + zone plan      the budget       slip floor,       lap account    slip floor;       |
   + M library   S3 D every zone, speed floor,      S3 zone floors    fallback: grip,   |
                    M straights,     swing rate,       first, then    then stop         |
                    F S-bends,       no-return         fastest                          |
                    G + H always                                                        |
   '-----------------+------> seeds in parallel, one core each, cut ~0.9 s -----> adopt  |
                                                                                        |
 INVARIANT (no field; a style may only tighten): grip share g1 | edge margins (widen     |
 only) | slip caps where no fall-off (front <= 1x, rear <= 3x peak) | ice rear at peak |  |
 friction at the lower quantile | estimator lowers only, holds through a slide | H as     |
 grip fallback | L0 end-speed bound | stopping-distance stop | section 5 table |          |
 FREE_STAGES 12 m  <--------------------------------------------------------------------'
```

### 4.2 What each style changes, layer by layer

| layer | S1 fastest | S2 drift ≤ budget | S3 drift max |
|---|---|---|---|
| offline / L0 | corner library, `drift_map.json` (gains) | + **signed losses** per cell (forced-drift solves) | + **zone plan** (where, which way, switch points, speed reference) + **M library** (manji seeds) |
| schedule | 4.4a table | + D (F into hairpins) where mapped loss ≤ remaining budget | D every zone, M on straights, F at S-bends, G and H always |
| L1 objective | min time | min time | min time |
| L1 rows | slip caps where no fall-off | same | + elastic: signed rear-slip floor, speed floor, swing-rate bound, no-return penalty |
| gate | switch on > 0.3 % twice, grip on tie | **lex-ε**: keep plans within (1+ε) of fastest, adopt most drift; hysteresis; debit lap account | plans meeting zone floors first, fastest of them; else any feasible, H last |
| L2 | today's | today's | ×3–5 weight on β, r in zones; soft slip floor; fallback recover to grip, then stop |
| ECU | as configured | as configured | traction control / ESC profile that does not cut a planned slide (roadmap 065 DP3) |

### 4.3 S2 in detail

- **Where the decision is made.** The cheapest asphalt drifts we have measured (Haruna's
  R 9.9–11.2 m hairpins) cost 0.09 s a lap in total, which per window is the size of the 0.3 %
  solver noise. The live gate cannot tell that cost from noise. So on asphalt S2 is decided
  **offline**: the drift map stores a signed loss per cell, and the user's budget `eps_lap_s`
  (seconds per lap) is spent on the drifts with the most drift per second lost (a knapsack
  over the lap's corners, planned before the lap).
- **Live gate.** Among the re-plan's feasible seed plans, keep those with
  T_k ≤ (1+ε)·T_min and adopt the one that drifts most (faster on a tie). Enter a drift after
  two re-plans inside, leave after two outside (1 + ε + h), h = 0.3 %. ε below the noise floor
  is refused.
- **On low friction** (no fall-off past the peak) drift already pays 1–2 % in most corners, so
  S2 is S1 plus the marginal corners.

### 4.4 S3 in detail

- **Zone plan first.** Drift direction is discrete; a gradient cannot choose it. The zone plan,
  built offline (`zones: auto` from geometry, width, class, edge type, or a file), sets every
  zone's direction and switch points: the planner's version of a D1 course map. Straights get a
  manji (alternating zones) only if `straights: true` and the width minus the edge margins
  leaves room for the sweep.
- **L1 rows in zones** (all elastic, so an unholdable zone degrades to grip at a cost, never to
  infeasibility; none in the first 12 m):
  - `sign_z · α_rear ≥ α_peak(class) + slip_floor_past`
  - `v ≥ (1 − speed_floor) · v_ref(s)`, v_ref from L0's profile or the previous G plan
  - `|dβ/dt| ≤ swing_rate_max` in switch intervals
  - a penalty on the angle easing off inside a zone (D1's "angle returns" deduction)
- **Seeds.** New **M** (manji): an alternating slip profile from an offline library by
  straight length × speed band, built by continuation like the corner library. F at S-bends.
  G and H on every re-plan (H is the fallback, G gives v_ref). Three to four solves a re-plan.
- **L2.** RTI linearises along the plan, so no separate drift-equilibrium model. Heavier weights
  on β and r in zones and a soft slip floor, so L2 does not "correct" a planned drift to grip.
  After three misses: recover to grip along H, then stop.
- **Drift measure** for every style and test: the rear axle ≥ 3° past its tyre's peak
  (`modes.REAR_PAST`). Body β is a diagnostic.

### 4.5 The style record

```yaml
style:
  name: fastest            # fastest | drift_budget | drift_max
  eps_lap_s: 0.0           # S2: seconds per lap the user gives for drift (preferred)
  eps_window: 0.0          # S2 fallback without a map: refused < 0.003 or > 0.03
  exit_hyst: 0.003         # S2: leave outside 1 + eps + h on two re-plans
  drift_past_deg: 3.0      # S2's D_k and S3's floor/measures; S1 keeps modes.REAR_PAST
  zones: auto              # S3: auto | path/to/zones.yaml
  straights: true          # S3: manji on straights
  slip_floor_past_deg: 3.0 # S3
  speed_floor: 0.10        # S3: v >= (1 - this) * v_ref
  swing_rate_max: 2.0      # S3: rad/s in switch intervals
  width_use: 0.67          # S3: share of drivable width a manji may sweep
  classes: [mud, sand, snow_loose, gravel, dirt, grass, snow_packed]  # asphalt after DP7; never ice
  l2_slip_gain: 1.0        # S3: 3-5
  # ecu_mode: added at roadmap 065 DP3, with an ECU interface (none today)
```

Validated at load; a field that breaks an invariant is **refused by name**, never clamped
silently (the repo's rule). Invariants are not fields. S1's default record must give today's
plans bit for bit.

## 5. Invariants (every style)

Grip share g1; edge-type margins (a style may widen only); the slip caps on classes without
fall-off; ice's rear held at its peak; friction at the lower quantile; the estimator lowers
only and holds through a slide; H as grip fallback; L0's end-speed bound; the stopping-distance
stop; design section 5's failure table; `FREE_STAGES` 12 m.

## 6. Risks

1. **Speed and angle are not independent on this car.** One acceleration request drives the
   rear or brakes 50:50. Stanford's speed-keeping drifts used an independent brake. S3's speed
   floor and slip floor will conflict where the car lacks surplus rear torque (fast
   straights). Roadmap 065 DP5.
2. **Asphalt drifts sit on the tyre's falling branch**, the regime that lost ice in 061 M4b.
   Test order: mud and sand first, asphalt last, never ice.
3. **L1 budget** with M seeds and long drift straights is unmeasured (p95 0.91 s today).
4. **061's gate is not passed** (M4b). S2 and S3 build on it.
5. **Every threshold above is a proposal** to be swept.
6. Tyre heat and wear are unmodelled; S3 is not free on tyres.

## 7. Acceptance criteria

| style | criterion |
|---|---|
| all | a bad record refused at load, naming the field; style changes apply without a stop, never inside 25 m of a committed zone |
| S1 | bit-identical plans to 061's scheduled arm; Haruna R6 0.000 % (three repeats) |
| S2 | plant lap-time loss ≤ budget + 0.3 %; drift share grows with budget; no added stops, spins or off-road vs S1; ≤ 1 mode change per corner; p99 adoption ≤ 1.0 s |
| S3 | rear past the floor for ≥ 80 % of zone length; zone speed ≥ (1 − speed_floor) × S1's; switches within a set distance of the plan; zero spins and off-road; LQR fallback ≤ 1 %; L1 p95 ≤ 0.9 s with M |

## 8. Calls made (builder check, 2026-10-31)

Settled by the author, not yet confirmed by the user:

1. **S1 bit-identical with S3 available.** Each seed process compiles two `Window` variants at
   start, plain and S3 (elastic rows). S1 always runs the plain one, today's code path; E1 diffs
   against the same commit with no style record.
2. **Live style change.** A new stdin message `{"style": {...}}` to `serve.py`, validated as at
   load, applied at the next re-plan subject to the 25 m rule.
3. **S2 budget.** Knapsack-picked corners become committed D corners: D adopted if feasible and
   within its budgeted loss + 0.3 %. The live lex-ε gate runs only on classes without fall-off,
   with `eps_window`. The lap account is debited by the map's predicted loss; gains are not
   credited. On an open course the budget is per run.
4. **Elastic rows are an exact (L1) penalty**, weighted above the multiplier bound so a floor
   binds whenever it is feasible; slack reported per zone; weight swept in E5. (Flag: this is
   the one place S3 carries a weight; it is a constraint penalty, not a drift reward.)
5. **Which seeds carry S3's rows**: D, M, F. G and H stay plain. v_ref is L0's profile when
   present, else G's last plan; S3's speed criterion uses the same reference.
6. **`drift_past_deg`** feeds S2 and S3 only; S1's classifier keeps `modes.REAR_PAST`.
7. **ECU**: today `tc_enabled` true, `esc_enabled` false, `abs_enabled` true
   (`sim_vehicle_ecu` calibration, from the spec); the planner cannot set it at run time. E4 is
   mostly about traction control. No `ecu_mode` field until DP3.
8. **"Committed drift zone"** in S1/S2: the adopted plan's slide in the chosen corner as
   `MultiStart._slides_in` defines it; 25 m as `AHEAD`.
9. **Refuse, never clamp**: S2/S3 fields set to non-defaults under `fastest` are refused too.
10. **Asphalt** stays out of S3's default classes until DP7.

## 9. Left to the builder

Zone-plan construction details, the knapsack solver, the M library's continuation path, L2
weight values within 3–5, the swing-rate penalty form, file layout. Thresholds come from the
experiments in roadmap 065.
