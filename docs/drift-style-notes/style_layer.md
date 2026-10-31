# A style layer over the live race planner: choosing and parameterising L1/L2 per driving style

(Research notes, 2026-10-31. Stack facts are from this repo's own docs: the live race planner
design (`docs/superpowers/specs/2026-10-29-live-race-planner-design.md`, sections 4, 4.4a, 5, 10),
`docs/drift-optimality.md`, `docs/mode-choice-methods.md`, `docs/mode-choice-m4b.md` and swb's
roadmap 061. External claims carry links; "[abstract]" or "[snippet]" marks a claim checked only
against an abstract or a search summary, not the full text. The three styles are: S1 fastest
lap, S2 drift wherever it costs at most epsilon in time, S3 drift everywhere possible, straights
included, keeping speed (D1-style).)

## Q1. Which formulation of the preference: weighted sum, epsilon-constraint, lexicographic, mode constraints, or drift-reference tracking?

### Takeaway
Do not put a weighted "time + w x drift reward" into L1's NLP. On a non-convex problem a weighted
sum cannot reach every trade-off, its weight has no units a user can set, and a reward on slip
pays the car to spin. Each style fits a different formulation. S1 is today's minimum time. S2 is a
**lexicographic choice with a tolerance, made across seeds in the gate**: keep the plans within
(1+epsilon) of the best time, then take the most drift among them. The NLP does not change.
S3 is **minimum time subject to a signed slip floor and a speed floor in drift zones**: a
mode-dependent constraint, seeded inside its own valley, with L2 tracking the plan's sideslip.

### Cited findings
- **Weighted sums miss non-convex parts of a Pareto front.** A point in a non-convex region has
  no supporting hyperplane, so no weight selects it. Das and Dennis (1997) made the point in "A
  closer look at drawbacks of minimizing weighted-sums of objectives for Pareto set generation",
  and their remedy was normal-boundary intersection
  ([Springer reference list](https://link.springer.com/article/10.1023/A:1010035730904)) [snippet].
  Another paper finds that "even using an infinite number of weights … the weighted-sum approach
  cannot fully cover the Pareto front" on a test problem
  ([arXiv:2203.02697](https://arxiv.org/pdf/2203.02697)) [snippet].
- **A weighted sum and a lexicographic cascade agree only under convexity, and only at one
  instant.** For convex priority-ordered MPC, a weighted sum reproduces the lexicographic optimum
  only when the weight lies in a slice of a normal cone. With squared-hinge penalties "no finite
  weight is exact whenever the limiting multiplier is nonzero", and "no single weight is valid
  across the sampled ticks of an episode"
  ([Hajieghrary et al., arXiv:2608.12520](https://arxiv.org/abs/2608.12520)) [abstract].
- **Lexicographic MPC costs one solve per priority level.** The cascade "requires L+1 sequential
  programs per MPC cycle" ([arXiv:2608.12520](https://arxiv.org/abs/2608.12520)) [snippet].
  Sequential prioritised MPC solves "a sequence of single objective NLPs for each level in a
  decreasing order of priority", with hard constraints that keep the earlier levels' optima
  ([prioritised OC paper](https://typeset.io/pdf/a-simple-formulation-for-fast-prioritized-optimal-control-of-r4ixmw0o.pdf)) [snippet].
- **Lexicographic order with a tolerance (lexicographic epsilon).** Candidates are filtered one
  objective at a time. A stage keeps those within epsilon_i of its best,
  Pi_i = {pi in Pi_(i-1) | c_i(pi) <= c_i* + epsilon_i}, and the last objective is minimised over
  what survives. Strict lexicographic order is impractical with sampled candidates, so costs
  within epsilon count as equal
  ([Multi-objective kinodynamic planning, arXiv:2607.15508](https://arxiv.org/pdf/2607.15508)) [snippet].
- **Epsilon-constraint MPC has run online with epsilon adapted to the terrain.** An "adaptive
  epsilon-constraint method integrated into a Model Predictive Control (MPC) framework" moves
  its bounds "based on terrain descriptors to explore the Pareto front in real time". It reports
  94 % navigation success against MPPI and a genetic-algorithm baseline
  ([Jerome, Kulathunga, Nascimento, RA-L, arXiv:2605.21188](https://arxiv.org/abs/2605.21188)) [abstract;
  which objective is constrained, the solver and the timing are not in the abstract].
- The augmented epsilon-constraint method (Mavrotas 2009) builds a payoff table of each
  objective's optimum first, then sets the constraints' right-hand sides from it
  ([GAMS epscm](https://gams.com/52/gamslib_ml/libhtml/gamslib_epscm.html)).
- **Drift as reference tracking.** Drift controllers track a sideslip reference. One NMPC tracks
  a fixed 30 deg sideslip together with lateral path error
  ([arXiv:2405.10859](https://arxiv.org/pdf/2405.10859)) [snippet]. One minimum-lap-time drift
  planner outputs sideslip, yaw rate and speed along its line, and the tracker penalises
  sideslip error ([arXiv:2608.00113](https://arxiv.org/pdf/2608.00113)) [snippet]. A drifting
  MPC paper says path tracking and drifting pull against each other
  ([circular-drift MPC](https://bohrium.dp.tech/paper/arxiv/658785aa8864da1b1a886af3748ddcf12e66ad5557c7accb3a017cdb955053a0)) [snippet].
- **No source found quantifies how a sideslip-reference weight trades lap time against drift.**
  A PoliMi study of human drivers uses drift-angle RMS as one style descriptor and finds a
  Pareto-like trade-off between lap time and steering effort
  ([PoliMi](https://re.public.polimi.it/retrieve/f134a092-63f8-4206-a80d-c45aabad6412/10-10-03-0026.pdf)) [snippet].
- **Our own facts on the drift measure.** Body sideslip is the wrong measure of a drift on this
  car. Gripping at the limit on asphalt takes 10-15 deg of rear slip and 6-12 deg of sideslip,
  and on snow 5 deg of sideslip is already past the peak. "the rear axle against its peak can"
  tell grip from slide (`drift-optimality.md`, "Sideslip is the wrong test here").
- **The handbrake-turn study needed a "proper yet simple cost function"** before the optimiser
  used the technique. That suggests the drift basin is small and the cost has to carry the
  intent (`mode-choice-methods.md`, citing [Southampton](https://eprints.soton.ac.uk/382093/)).

### Inferences
- **Weighted sum (time + w x drift).** Pros: one NLP, IPOPT warm starts keep working, no extra
  solve. Cons: (a) w has no meaning the user can set, and its effect changes with corner,
  surface and speed. The certificate result above says no single weight holds across ticks even
  in the convex case. (b) The grip and drift valleys lie on a non-convex front, so the "drift
  where it costs <= X" points may be unreachable for every w. (c) A reward on |rear slip| or
  |beta| without an upper bound pays the plan to slide past the slip_rows cap or to spin. It
  needs the cap anyway, and then the reward pushes every zone onto the cap. (d) Changing w
  moves the optimum continuously, so a warm start from the old w may sit in the wrong valley.
  This is the same seeding problem as grip and drift. Verdict: acceptable only as a small
  regulariser inside S3, not as the style knob.
- **Epsilon-constraint inside the NLP (max drift s.t. T <= (1+eps) T*).** It needs T* first,
  which means two sequential solves inside a 1 s budget whose p95 is already 0.91 s. T* can come
  from the G solve of the same re-plan (sequential, so no), from the previous re-plan, or from
  L0's v_max(s). It also replaces the time objective with a drift objective, so the plan's
  time is only bounded, and IPOPT's convergence on a "max slip" objective is untested here.
  Verdict: not for L1 online. It suits offline work, for example building L0's per-corner drift
  map, which needs the loss of a forced drift.
- **Lexicographic epsilon over the seeds (S2).** It uses what already exists. Each seed's plan
  has a time T_k and a drift measure D_k, so the gate keeps {k : feasible and T_k <= (1+eps)
  min T} and adopts the argmax of D_k, with grip on a tie. This is arXiv:2607.15508's filter
  applied to L1's discrete candidates. It costs no extra solve and no NLP change, and warm
  starts are untouched because each seed keeps its own last plan. It is exact only over the
  valleys that were seeded, which is acceptable because the seeds already define the modes
  (`mode-choice-methods.md`: the field enumerates discrete choices before the solver).
- **Mode-dependent constraints (S3).** In each drift zone the window adds, per node, a rear slip
  floor of sign_z x alpha_rear >= k_min x alpha_peak(class) and a speed floor of
  v >= (1 - delta) x v_grip(s), where v_grip comes from the G plan or L0. The objective stays
  minimum time. Fixing the sign per zone keeps the constraint smooth and turns a disjunction
  (|alpha| >= floor) into one valley, which the seed then supplies. The constraint is
  infeasible for a grip start, but IPOPT accepts infeasible starts, and a D seed or a manji
  seed starts inside it. The slip floor must sit below the slip_rows cap: alpha_peak < floor
  <= 3 alpha_peak on classes without fall-off.
- **Drift-reference tracking in L1 (|alpha - alpha_ref|^2).** It makes the plan copy a
  hand-written profile instead of finding the fastest way to hold the drift, and the reference
  has to be generated somewhere. Better in L2, where the reference is the L1 plan's own slip and
  sideslip (Kapania, as section 4.5 already says).

### Gaps
- No paper found that measures a weighted-sum and an epsilon-constraint style knob side by side
  on a non-convex vehicle NLP with IPOPT warm starts.
- Not found: the full text of Das and Dennis 1997, or arXiv:2605.21188's solver and timings.

## Q2. How do AV and racing stacks let a behaviour layer select cost or constraint sets?

### Takeaway
Every stack found picks configurations upstream and parameterises the solver through limits
and enable flags, not through an online weight search. TUM's A2RL stack sends an orchestration
command (maximum velocity, overtaking permission, following distance) and scales acceleration
limits per cell with a GripMap. TRI's NMPC exposes "a designated maximum fraction of the
friction circle". Autoware turns modules on or off from presets and gives each its own
parameter file. Weight-based style knobs exist (an aggressiveness value mapped to weights),
but in comfort-oriented driving, not at the handling limit. No racing stack found has a
user-facing "style" for drifting.

### Cited findings
- **TUM, A2RL 2025.** Layers are offline (raceline, GripMap, map), sense, plan (target and
  emergency trajectories), act (Tube MPC plus low-level PID) and orchestration (a state machine
  and watchdog). "the state machine generates and distributes an orchestration command within
  the core stack", carrying the maximum velocity, overtaking permission, following distance and
  pit requests. The GripMap scales the limits per Frenet cell, a_x,max(s,n) = theta_ij x
  a~_x,max(v, a_z), and lower grip off the raceline makes the car "behave more cautiously". The
  sampling planner has "a fixed-time horizon of 4 s" and six weighted cost terms, with weights
  not published. The safety ellipse around opponents scales with speed "to accommodate varying
  risk levels". The TMPC was retuned to keep lateral deviation smaller, which "increases the
  safety margin". The paper reports grip variations "of 50 %"
  ([Hoffmann et al., arXiv:2602.08571](https://arxiv.org/html/2602.08571)).
- **TUM, IAC.** The full software stack is in Betz et al., *Journal of Field Robotics* 2023
  ([arXiv:2205.15979](https://arxiv.org/abs/2205.15979)) [abstract only; its behaviour-planning
  section was not read]. A TUM planner "generates an action set of multiple drivable
  trajectories", from which an adjacent behaviour planner picks
  ([catalyzex listing, Stahl](https://catalyzex.com/author/Tim%20Stahl)) [snippet].
- **TRI GR Supra NMPC.** The cost "penalizes exceeding a designated maximum fraction of the
  friction circle" at each axle. Track bounds and sideslip are soft constraints. The horizon is
  20 points covering 120 m. CasADi and IPOPT solve it, capped at 50 iterations and warm-started
  from the previous solution, with a mean solve time of 31.3 ms and 15.7 iterations
  ([Dallas, Thompson, Goh, Balachandran, arXiv:2304.12263](https://ar5iv.labs.arxiv.org/html/2304.12263)).
- **TRI transient drifting.** An NMPC with a coupled-slip tyre model and wheel-speed dynamics,
  for deep rear saturation, which "smoothly transition[s] from dynamic, non-equilibrium drifting
  to grip driving" (Goh, Thompson, Dallas, Balachandran, *Vehicle System Dynamics* 62(10),
  2024, pp. 2590-2613;
  [record](https://bohrium.dp.tech/paper/arxiv/2a28f15814cf14ed9abe5f5c27722e72d1a78e53588d1663fa20a21e8e819664)) [abstract].
  In the 2024 TRI/Stanford tandem drift, each car "solves and re-solves an optimization
  problem up to 50 times per second", and the chase car's task is to "drift alongside the lead
  vehicle while proactively avoiding a collision"
  ([TRI blog](https://medium.com/toyotaresearch/stanford-engineering-and-toyota-research-institute-achieve-worlds-first-autonomous-tandem-drift-131fcb9a76a9)).
  The drift is given to the controller as a task. It is not a lap-time decision.
- **Stanford MARTY.** A figure-eight drift was planned as a nonlinear optimisation
  ([Goh thesis](https://purl.stanford.edu/th820zb4174)). An augmented NMPC "expands the
  operational domain … to track drift transitions", shown by tracking figure-eights
  ([Stanford thesis](https://purl.stanford.edu/mz751hw3069)) [snippet]. A figure-eight is the
  closest published analogue of a linked direction change (manji).
- **Autoware behavior_path_planner.** A manager activates "the relevant scene module in response
  to the specific situation", orders the active modules and merges their paths. Modules are
  turned on in `default_preset.yaml` (`launch_<module>_module`) and each has its own
  `.param.yaml`, with changes taking effect on the next launch
  ([Autoware docs](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_planner/)).
- **Apollo's** planner is documented as scenario, then stage, then task, with tasks configured
  per scenario. The README fetch failed (404), so this is not verified here.
- **Unimore (A2RL).** A thesis describes heuristic corridor selection from other agents'
  predictions, followed by an MPC planner and controller
  ([Bondi thesis](https://unitesi.unimore.it/retrieve/b0d5375a-4123-4399-a1d3-03a9bac0ed05/Bondi.Simone.pdf)) [snippet].
  No PoliMOVE planner paper was found.
- **Style as cost weights (road driving).** An aggressive style weights "terms relating to the
  time required to complete certain maneuvers" more and jerk or g-force less
  ([US patent 10606270](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10606270)).
  An ACC example uses jerk weight 0.05 and a 1.0 s gap for aggressive, against 0.2 and 3.0 s
  for conservative ([project page](https://sarkaranimesh.github.io/MotionPlanning_projects)).
  LanguageMPC maps instructions such as "I am in a hurry" to MPC weights, inverse optimal
  control learns weights from a driver's data
  ([review, arXiv:2512.11944](https://arxiv.org/pdf/2512.11944)), and multi-objective RL takes
  a continuous preference vector at run time
  ([arXiv:2505.05223](https://arxiv.org/html/2505.05223v2)) [snippets].
- **D1's own definition of S3.** The 2024 rulebook asks for speed "as fast as possible in any
  part of the course" and notes that angle and speed trade against each other. Solo runs are
  scored on angle and speed by the DOSS system, plus a course line the judges set in advance
  ([D1GP](https://d1gp.co.jp/02_what/whats_index-e.html);
  [2024 rulebook PDF](https://entry.d1gp.co.jp/download_files/Data_4.24-01縲占ｦ丞援譖ｸ2024%20闍ｱ隱樒沿縲・English_2024_P28-31_D1謗｡轤ｹ蝓ｺ貅.pdf)) [snippet].
  The FIA publishes a drifting-event guideline
  ([FIA](https://www.fia.com/sites/default/files/guideline_for_drifting_event_-_wmsc051218_-_clean_0.pdf)) [not read].

### Inferences
- The pattern that transfers: **an orchestration record, set at the top and read by the layers**.
  At TUM it carries v_max, permissions and distances, and in Autoware it carries enable flags
  and per-module parameters. We already have the pieces it would set: the seed schedule
  (`schedule.py`, `drift_map.json`), the gate's thresholds, the window's constraint rows
  (`window.slip_rows`), g1, and L2's weights.
- TRI's "maximum fraction of the friction circle" corresponds to our g1. TUM's GripMap theta
  corresponds to our per-class mu at the lower quantile. Both are **safety parameters, not
  style parameters**, and both stacks keep them outside any aggressiveness knob.
- D1 scoring gives S3 its spec: zones with a line, a minimum angle, and speed held high.
  It maps onto the slip floor, the speed floor, and track-margin rows per zone. A free
  "maximise angle" objective does not fit it.

### Gaps
- No published racing stack found exposes drift intensity as an operator setting.
- Apollo's scenario configuration was not verified this session.
- Not found: PoliMOVE's or Unimore's planner papers with parameter details.

## Q3. How should the seed schedule, the gate and L2 change per style?

### Takeaway
S1 is today's design, unchanged. S2 changes two pieces: the gate becomes the lexicographic
epsilon filter, and the schedule adds D wherever the drift map's *loss* is at most epsilon,
not only where its gain clears 0.3 %. S3 changes the window (signed slip floor and speed floor
per zone), adds linked-transition seeds (manji, flick) for straights and S-bends, adds an
offline zone and direction plan (the D1 "line"), and makes L2 weight the plan's slip and
sideslip more heavily. L2's RTI already linearises along whatever L1 plans, so it needs no
drift-equilibrium model, only weights and a sideslip-rate reference through transitions.

### Cited findings (stack facts; external sources as in Q1 and Q2)
- Today's gate: feasible plans only; switch mode only when a plan is more than 0.3 % faster on
  two re-plans; grip on a tie; scored at the lower friction quantile. 0.3 % is the solver's
  noise (p75 at mu 0.45), and a 0.1 % gate "took a noise-sized gain and gave it back"
  (design section 10).
- Today's schedule adds D only where the map's drift gain for the corner clears 0.3 %, on
  classes that fall off past the peak, up to the measured cell's radius (design section 10;
  061 M4b). Haruna's scheduled arm reproduces 060's plans bit for bit (0.000 %, M4b).
- On asphalt a drift wins only in hairpins of 180 deg and R <= 25 m, by at most 0.4 %, and never
  in 90 deg corners. On mu 0.3 it wins in nearly every corner, by up to 1.6-1.8 %. A drift seed
  from scratch takes 230-280 iterations at p95, and the corner library's seeds reach the
  optimum most often (design 4.4a and section 10; `mode-choice-methods.md`).
- A drift's tracking cost: TUM reports 1.1 m lateral error and a 0.06 rad sideslip overshoot
  while tracking a drift ([arXiv:2606.27914](https://arxiv.org/html/2606.27914), cited in
  `mode-choice-methods.md`).
- TRI's transient-drift NMPC models wheel speed and coupled slip to hold deep rear saturation
  through transitions (Goh et al. 2024, Q2).

### Inferences: per-style changes
**Gate.**
- S1: unchanged (> 0.3 % twice, grip on a tie).
- S2: candidates C = {k feasible : T_k <= (1 + eps) x min_j T_j}, all scored at the lower
  friction quantile as today. Adopt argmax D_k over C, where D_k is the drift measure (below).
  On a tie, take the faster plan. For hysteresis, enter drift when a D plan stays in C on two
  re-plans, and leave when it falls outside (1 + eps + h) on two re-plans, with h about the
  noise floor of 0.3 %. eps below the noise floor makes no sense, because the gate cannot tell
  a 0.1 % loss from noise. So eps is clamped to at least 0.3 %, and the useful range is
  0.3-3 %.
- S2 also needs a **lap-level budget**: an eps per window lets every window lose eps, so the lap
  loses eps. That is the intended semantics if eps is "per lap, relative". If the user means
  "at most X seconds per lap", a running account (seconds spent so far against X) has to
  narrow eps for the rest of the lap. A knapsack over the corners (drift the ones with the best
  drift per second lost) can be planned offline from `drift_map.json` once it stores losses.
- S3: no time gate between modes. The window is solved only in drift mode inside zones, and the
  gate's job shrinks to feasibility and fallback. If no drift seed is feasible, adopt any
  feasible plan, H last. The safety order of section 5 stays.

**Drift measure D_k** (for S2 and for tests). The distance-weighted share of window nodes where
the rear axle is past k x alpha_peak(class), with k about 1.2, or the integral of
(|alpha_rear| / alpha_peak - 1)+ over s. It is keyed to the tyre's peak per class, as
`drift-optimality.md` requires, and never to beta alone.

**Schedule.**
- S1: today's table.
- S2: add D (and F) where the map's drift *loss* is <= eps for the corner's cell, so
  `drift_map.json` needs signed results (loss where drift is slower), not only the cells where
  it wins. Leave out corners where a forced drift is infeasible (ice: no drift pays, and the map
  may show infeasible).
- S3: every zone the zone plan marks (below) gets D. A sequence of zones with alternating
  direction gets an **M (manji) seed**: a sinusoidal-sign slip profile with sign changes at the
  zone plan's switch points, built offline per straight length and speed band by continuation,
  as the corner library is. F seeds at S-bends. G still runs as the fallback and as v_grip's
  source for the speed floor.

**Zone plan (S3 only, offline or L0).** It decides where to drift and which way (the D1 "line"),
using the track's edge types (guardrail, terrain, run-off), width and class. It must be fixed
before the solver, because direction is the discrete choice gradients cannot make
(`mode-choice-methods.md`: a flick has a zero-slide ridge between its two directions).

**Window (L1).**
- S1/S2: unchanged.
- S3: per-node rows inside zones:
  - a slip floor, sign_z x alpha_rear >= k_min x alpha_peak;
  - a speed floor, v >= (1 - delta) x v_grip(s);
  - optionally, a sideslip-rate bound through the switch points.
  Each row is soft with a large L1 penalty (an elastic row). An infeasible zone then degrades
  to grip with a cost, and the plan does not fail. The objective stays minimum time. A small
  weight on (alpha_rear / alpha_peak - target)^2 inside zones may be added only to shape the
  plan within the floor and the cap.

**L2.**
- S1/S2: today's weights. The plan's sideslip and yaw rate are already references (4.5).
- S3: raise the sideslip and yaw-rate weights inside zones. Add the slip floor as a soft
  constraint so L2 does not "correct" a drift back to grip. RTI linearises along L1's plan, so
  a non-equilibrium drift needs no separate equilibrium linearisation. Linked transitions need
  the wheel-speed and coupled-slip dynamics that TRI found necessary (Goh 2024). Check that
  L2's model holds them. The LQR fallback is valid only near the plan, which in a manji is
  itself transient. After three misses, S3's fallback should be "recover to grip, then stop",
  not "stop along the plan".

### Gaps
- Not known: whether L1's model and IPOPT converge on long constant-drift straights at Haruna
  speeds within 0.9 s. No measurement exists, and the M seed and the slip-floor rows are new.
- Not known: tyre heat and wear in S3. The design lists heat (2-15 % more per drift) but has no
  wear model.
- No source found for manji or linked-drift planning by optimisation beyond figure-eights.

## Q4. What stays invariant across styles, and what does a concrete style record and test plan look like?

### Takeaway
Style may change the objective, the gate and the seeds. It may never change the safety
pieces: g1, edge-type margins, the slip_rows caps, the friction quantile and the estimator's
lower-only rule, H as the grip fallback, the stopping-distance stop, and section 5's failure
table. A style record of about ten fields, clamped against those invariants, covers S1-S3.
S1 has to reproduce today's plans bit for bit.

### Cited findings
- TUM keeps grip scaling (GripMap) and margins as separate inputs from the orchestration
  command, and tightened its controller for a larger safety margin
  ([arXiv:2602.08571](https://arxiv.org/html/2602.08571)). TRI keeps the friction-circle
  fraction as a cost-side limit ([arXiv:2304.12263](https://ar5iv.labs.arxiv.org/html/2304.12263)).
- Our invariants, from the design:
  - L1 at grip share g1 = 0.92, L2 up to 1.
  - Margins by edge type.
  - On classes without fall-off, the front axle within its peak and the rear within 3x it. The
    free plan plowed the front at 9-21x its peak, and every arm stopped at the mud hairpin, 6 of
    6. Bounded, none stopped (M4b, issue 002).
  - Friction at the lower quantile.
  - The estimator only lowers friction and holds it through a slide (060 M6: an estimate that
    rose mid-slide crashed).
  - Section 5's fallbacks.

### Inferences: the style record (sketch)

```yaml
style:
  name: fastest | drift_budget | drift_max      # S1 | S2 | S3
  eps_time: 0.0          # S2: relative loss allowed per window, clamped to [0.003, 0.03]; S1: 0
  eps_lap_s: null        # S2 alternative: seconds per lap, a running account narrows eps
  drift_k: 1.2           # D_k counts rear slip past drift_k x alpha_peak(class)
  slip_floor_k: 1.5      # S3: sign_z x alpha_rear >= slip_floor_k x alpha_peak; must be < slip_rows cap (3.0)
  speed_floor: 0.10      # S3: v >= (1 - speed_floor) x v_grip(s)
  zones: auto | file     # S3: zone plan (where, direction); auto = from edge type/width/class
  straights: true        # S3: allow zones on straights (manji); false = corners only
  allow_classes: [asphalt, gravel, dirt, grass, packed_snow, mud, sand, loose_snow]  # never ice for S3
  l2_slip_weight: 1.0    # multiplier on L2's sideslip/yaw-rate weights (S3: e.g. 3-5)
```

**Clamps from the invariants:**
- slip_floor_k < the slip_rows cap on its class.
- speed_floor never lifts the speed above the plan's own v_max bound or the stopping distance.
- No zone where the edge is a guardrail and the width is below a set minimum (to be measured).
- g1, the margins, the quantile and the estimator are not in the record at all.

**What each style toggles:**

| piece | S1 fastest | S2 drift <= eps | S3 drift max |
|---|---|---|---|
| L1 objective | min time | min time | min time |
| L1 rows | slip_rows | slip_rows | + signed slip floor, speed floor in zones (elastic) |
| seeds | schedule (gain >= 0.3 %) | + D where map loss <= eps | D every zone, M on straights, F at S-bends, G as fallback |
| gate | > 0.3 % twice, grip on tie | lex-eps: within (1+eps), max D_k, hysteresis h | feasibility and fallback only |
| offline | corner library, drift map | + signed losses in drift map | + zone plan, manji library |
| L2 | plan references | same | heavier slip/yaw weights, soft slip floor |
| invariants | g1, margins, slip caps, quantile, estimator, H, stop | same | same |

**Test criteria:**
- **S1 (regression).** Plans identical to the M4b scheduled arm on Haruna and Tsukuba (0.000 %),
  and R6 as today.
- **S2.**
  - (a) Offline, on the drift map's corners: the adopted plan's predicted loss against
    the S1 plan is <= eps + the noise floor in >= 95 % of re-plans.
  - (b) On sim.Plant, the realised lap-time loss is <= eps + 0.3 %.
  - (c) The drift share D grows monotonically with eps over {0.3, 1, 2, 3} %.
  - (d) No added stops, spins or off-road against S1.
  - (e) At most one mode change per corner, and no flip-flop.
  - (f) p99 adoption latency <= 1.0 s.
- **S3.**
  - (a) The rear stays past slip_floor_k x alpha_peak for >= 80 % of zone length on the plant.
  - (b) Mean speed in zones >= (1 - speed_floor) x the S1 speed there.
  - (c) Direction switches happen where the zone plan puts them, within a set distance.
  - (d) Zero spins and zero off-road. The L2 fallback rate is reported, and the LQR rate is
    <= 1 %.
  - (e) The largest sideslip stays under the plan's plus a margin, as in M4b (23-32 against
    27-36 deg).
  - (f) The L1 budget is met: p95 <= 0.9 s with the M seed.
  - (g) Run on mud, packed snow and asphalt, and never on ice.
- **All styles.** Switching style mid-run takes effect at the next re-plan without a stop, and a
  style record that breaks a clamp is refused at load.

### Gaps
- The thresholds in the record (drift_k, slip_floor_k, speed_floor, the guardrail width
  minimum) are proposals and have not been measured. They need a sweep, like M3b's width arm.
- Whether a soft slip floor in L2 fights the RTI linearisation during a sign change has not
  been tested.
