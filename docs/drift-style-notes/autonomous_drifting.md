# Autonomous drifting: sustained, linked and speed-keeping drifts, and drift as a style choice

Researched 2026-10-31, about 15 searches and fetches. This adds to
`docs/mode-choice-notes/drift_planners.md` (Stanford/TRI line, Tavernini/Velenis minimum-time
OCPs, Ajanović TAMP, the Toyota patent, TUM AMG) and `docs/related-work.md`. Facts already
there are not repeated. The focus here is how a drift is *specified*, how left-to-right
transitions and linked drifts are done, how speed is kept while drifting, and whether anyone
parameterises the trade-off between lap time and drift.

Access: the TRI Medium post and the Stanford News tandem story returned 403. No technical paper
on the tandem system was found (as of 2026-10-31). Several theses were read only as abstracts
on their purl pages. Each item says whether it is a real vehicle (REAL), a scale car (RC), or
simulation (SIM).

## Q1. Stanford/TRI: how is a drift specified, and how are direction transitions done?

### Takeaway
Across Stanford's line, the drift reference has grown from an *equilibrium* (Hindiyeh), to
*path + sideslip* with speed left free as a stable zero dynamic (Goh), to *path + sideslip +
speed* tracked together by using the brakes as an extra input (Goel; Peterson on the Supra).
Left/right transitions (figure-8) need an augmented NMPC and a supervisor that keeps steering
authority over yaw. The ~1 km sustained course and the tandem runs show linked drifts are
feasible on full-size cars. Every reference was still made offline by a separate planner (a
nonlinear optimisation for the figure-8) and was never chosen for lap time.

### Cited Findings
- **Goh, PhD thesis, Stanford 2019, "Automated vehicle control beyond the stability limits"
  (REAL, MARTY DeLorean).** The general-path controller does not track speed explicitly.
  Lateral-longitudinal coupling makes velocity a stable zero dynamic, shown on a slowly varying
  trajectory. The figure-8 trajectory is planned as a nonlinear optimisation problem. The main
  difficulty is losing independent control of the velocity vector and the body rotation rate
  at the limits of achievable state derivatives, and modifying the controller for this improved
  results. The proposed envelope criterion is "avoiding the loss of steering authority over the
  rotational dynamics". With a supervisor that modifies the controller, the car completed the
  manoeuvre; without it, the car spun. The final test was a sustained drifting course of nearly
  1 km with several highly dynamic transitions, at up to 40 deg sideslip, 2.5 rad/s yaw rate
  and 50 km/h. — [purl.stanford.edu/th820zb4174](https://purl.stanford.edu/th820zb4174)
- **Goel, PhD thesis, Stanford 2022, "In complete control: simultaneous path, speed and
  sideslip angle control of a drifting automobile" (REAL, MARTY).** Brakes, steering and drive
  torque together make drifting "fully actuated". For a given curvature, the drift equilibria
  grow from a curve to an *area* in the speed-sideslip plane, and front braking adds a further
  dimension. That gives "a way to remove energy while keeping sideslip and position under
  control", i.e. speed and drift angle can be chosen independently. The NMPC's horizon trades
  off the objectives and gives "selective prioritization". An *augmented* NMPC tracks figure-8s
  that switch left and right, and comparisons show the augmentations matter. More braking in
  the figure-8 could help but would need higher model fidelity. The abstract gives no numbers.
  — [purl.stanford.edu/mz751hw3069](https://purl.stanford.edu/mz751hw3069)
- **Peterson, PhD thesis, Stanford 2023, "Exploiting linearity to precisely control a vehicle
  about an unstable equilibrium" (REAL, MARTY and the Supra "Takumi").** An LQR about the
  equilibrium (front brakes hold constant speed) tracked equilibrium and quasi-equilibrium paths
  on MARTY with "centimeter-level accuracy that exceeds prior work". Adding wheel-speed dynamics
  extended it to variable-speed drifting on the combustion-engine Supra, as a fully actuated
  LQR (steering, throttle, brakes). An MPC on the same linear model kept inputs inside their
  limits during transients. The eigenstructure changes consistently from one equilibrium to the
  next, which makes one linear family usable across a region of state space. —
  [purl.stanford.edu/vg668bz0151](https://purl.stanford.edu/vg668bz0151)
- **TRI/Stanford tandem drift (REAL, two GR Supras, Thunderhill, announced July 2024).** The
  lead car "focuses on tracking an ideal path at a high angle". The chase car keeps close to it
  and avoids collision. Both run NMPC re-solved up to 50 times a second, use GPS and talk over
  Wi-Fi. TRI wrote the lead car's algorithms and Stanford the chase car's, which uses learned
  ("AI") vehicle models to adapt to the lead car's motion. No paper found. —
  [Green Car Congress](https://www.greencarcongress.com/2024/07/20240724-tri.html);
  [Toyota UK](https://media.toyota.co.uk/toyota-research-institute-and-stanford-engineering-achieve-worlds-first-autonomous-tandem-drift-sequence/);
  [autonomousvehicleinternational](https://www.autonomousvehicleinternational.com/news/testing/tri-and-stanford-engineering-achieve-worlds-first-autonomous-tandem-drift-sequence.html)
- TRI's 2022 single-Supra NMPC drifts round obstacles within the lane and replans the
  trajectory every 1/20 s. The Toyota Times article gives no angles, speeds or transition
  logic. — [Toyota Times 2022](https://toyotatimes.jp/en/spotlights/1005.html)
- Weber & Gerdes, "Modeling and control for dynamic drifting trajectories", IEEE T-IV 2024
  (Supra). Confirmed to exist. The full text, with its numbers, was not reached. —
  [reviewer.ly listing](https://cs.reviewer.ly/app/publication/874650bb-ae72-4a5a-8db3-b2662af79be4)

### Inferences
- Speed keeping is a solved control question *if* the car has an independent energy-removal
  input (front brakes, or per-wheel torque) on top of rear drive. Under Goh's
  speed-as-zero-dynamics, speed follows from the sideslip and path, which suits "drift where
  cheap" but not D1-style "keep the speed up". A D1-style planner needs a Goel/Peterson-type
  three-reference tracker (path, β, v).
- The figure-8 lesson for linked drifts: the transition is where the rotation-rate limits bind.
  A planner should constrain the β and r transition rate (yaw acceleration) so that steering
  authority is never lost, and should not hand the tracker a step in β.
- Tandem: the lead car is "path + high angle", so it is specified the same way as a single
  drift. The chase car turns the drift into a relative-position problem. For three styles that
  only involve the ego, the lead-car formulation is enough.

### Gaps
- Tandem paper (formulation, horizon, solve time, angles): not found. The Medium and Stanford
  News pages returned 403.
- Goh's figure-8 OCP cost and constraints, and Goel's figure-8 errors, need the full theses
  (PDFs on purl).
- Weber & Gerdes 2024 T-IV numbers were not reached.

## Q2. NMPC and trajectory optimisation that produce drifting: drift as objective or constraint, linked and straight-line drifts, path-following

### Takeaway
Pure minimum-time OCPs make drift appear only where it pays (sharp corners, low μ). In those
OCPs β is at most *bounded* (e.g. |β| ≤ 30 deg) and never rewarded. Drift-tracking NMPCs track
an *equilibrium (β, r)* looked up from path curvature. Transitions between opposite sustained
drifts ("inertia drift") are a separate planning problem, studied so far only in simulation
(Zhou/Betz 2025). I found no OCP that rewards sideslip against lap time, nor any formulation of
a drift on a straight.

### Cited Findings
- **Zhao, Zhao, Wu & Lv, "Track-Guided Hierarchical RL for Autonomous Vehicle Drifting with
  Minimum-Lap-Time Planning", arXiv 2608.00113, 2026 (SIM, CARLA).** OCP cost = pure lap time
  in Frenet arc length, J = ∫(1 − l·κ)/(v·cos(Δψ+β)) ds. β is only constrained, |β| ≤ π/6.
  Bounds: v 0-30 m/s, steering ±30 deg, slip ratio > −1. Boundary conditions β = r = Δψ = 0 at
  both ends. Model: 3-DoF bicycle with combined-slip Magic Formula, μ = 0.6, SNOPT. On Lkarting
  the plan reaches about 29.9 deg β (on the bound) and 25 m/s. Minimum laps (plan / RL tracker /
  Cai baseline / human): Lkarting 33.1 / 37.2 / 47.6 / 51.3 s; Willow Springs 155 / 181.1 /
  193.1 / 202.9 s; Nelson Ledges 142 / 158.8 / 171.1 / 180.2 s. The tracker loses 12-17 % of
  the plan's time, partly from model mismatch (planner bicycle vs CARLA vehicle). No grip-only
  baseline OCP is given. — [arXiv 2608.00113](https://arxiv.org/html/2608.00113)
- **Meijer, Bertipaglia & Shyrokau, arXiv 2405.10859, 2024 (SIM + REAL semi-automated, BMW
  M3).** The NMPC tracks the equilibrium β and r quadratically and penalises steering rate.
  Sideslip is not a hard constraint. ACADO SQP at 50 Hz, N = 25. The equilibrium maps are built
  offline (state derivatives = 0) from desired β and path radii. An outer path-following loop
  adjusts the desired curvature from lateral error, and a lookup table picks the equilibrium.
  SIM: 1 m maximum lateral deviation, and a left-to-right transition is shown. REAL: the human
  steered (the EPS gave only about 30 Nm), the NMPC set drive torque, and about 30 deg β was
  held for several laps on dry and wet surfaces. Speed tracked worse than β and r. —
  [arXiv 2405.10859](https://arxiv.org/html/2405.10859v1)
- **Zhou, Li, Xie, Su & Betz, "A Learning-based Planning and Control Framework for Inertia
  Drift Vehicles", arXiv 2507.05748, 2025 (SIM).** Targets the switch between two opposite
  sustained drifts (an "8" path). Planning logic aims for a smooth transition "with minimal
  velocity loss". Bayesian optimisation learns a performance-driven control policy, and (from
  a search snippet) a learned trigger switches slightly early, after which β reverses quickly.
  The abstract has no numbers. — [arXiv 2507.05748](https://arxiv.org/abs/2507.05748)
- **Zhou, Hu, Zeng, Li, Betz, Xie & Su, ALMPC, arXiv 2502.04696, 2025 (SIM, CarSim).** Drift is
  path tracking while staying near a drift equilibrium point (DEP). An upper Bayesian-
  optimisation layer learns the DEP and the path-tracking law, and a lower MPC drifts. It
  follows a clothoid path with a misidentified μ. A search snippet attributes a taxonomy to
  this line of work: *sustained* (near an unstable equilibrium on a circle), *transient*
  (brief drift to change heading: drift parking, cornering) and *inertial* drift. The abstract
  page does not show this taxonomy. — [arXiv 2502.04696](https://arxiv.org/abs/2502.04696)
- **Wu, Hu, Weng, Li, Fu, Xie & Su, arXiv 2410.05740, 2024-25 (SIM + RC 1:10).** MPC plus
  Gaussian-process correction of the drift equilibrium and the model. GP variance drives
  exploration of cornering drift *velocities*. RC: lateral error −36.7 % with the GP and a
  further −29.0 % with exploration; velocity RMSE −7.2 %. A search snippet says this line finds
  minimum-time planning drifts in corners when β and r have opposite signs; I did not verify
  this on the abstract page. — [arXiv 2410.05740](https://arxiv.org/abs/2410.05740)
- **Steeringless drifting, Zhao, Wu, Zhou, Zhao & Wu, arXiv 2607.24863, 2026 (RC 1:10, 4WID).**
  Drifts in steady circles at about 20 deg β and tracks a figure-8 with wheel torque only. A
  search snippet says each reversal flips the wheel-torque directions between equilibria of
  about 30 deg. — [arXiv 2607.24863](https://arxiv.org/abs/2607.24863)
- **Individual-wheel-drive drift RL, arXiv 2507.23339 (SIM).** Per the search snippet, the
  figure-8 transition runs in phases: right-wheel deceleration, a peak wheel-speed
  differential, then convergence to the opposite equilibrium. β is held near −50 deg. —
  [arXiv 2507.23339](https://arxiv.org/pdf/2507.23339)
- **CBF safe set for figure-8 (arXiv 2503.19994, SIM).** Per the search snippet, a control
  barrier function on a safe set in the β-r phase plane keeps the car recoverable during
  figure-8s. — [arXiv 2503.19994](https://arxiv.org/pdf/2503.19994v1.pdf)
- **GP-enhanced ADMM-iLQR, arXiv 2503.11083, 2025 (SIM).** Hierarchical drifting along general
  paths, with better tracking and computation time than its baselines. —
  [arXiv 2503.11083](https://arxiv.org/abs/2503.11083v1)
- **TU Darmstadt NMPC (Koopman → QP)** holds the car at a desired unstable equilibrium
  (sustained drift). — [TU Darmstadt CCPS news](https://www.ccps.tu-darmstadt.de/ccps/news_details_ccps_237376.en.jsp)
- **Velenis & Tsiotras 2005 (ISIE)** compared *minimum time* with *maximum exit velocity* as
  the cost for cornering. The choice of objective changes the manoeuvre, which is an early
  example of style through the cost. —
  [dcsl.gatech.edu isie05a.pdf](https://dcsl.gatech.edu/papers/isie05a.pdf)
- Tavernini et al. 2013/2014 (paved low drift, loose-surface rally drift, handbrake in tight
  hairpins) are covered in drift_planners.md. No new numbers: full texts are still paywalled. —
  [Southampton 382093](https://eprints.soton.ac.uk/382093)

### Inferences
- Three building blocks map onto the three styles:
  1. *Fastest lap*: minimum-time OCP, β free or bounded (2608.00113's formulation). Drift
     appears only where it pays, which matches our 0.4 % / 1-2 % findings.
  2. *Drift where cheap*: the same OCP plus a sideslip reward, or a constraint
     |β| ≥ β_min on corner segments, with a lap-time ε-constraint T ≤ (1+ε)·T*. No paper does
     this. It is the natural epsilon-constraint construction.
  3. *Drift everywhere (D1)*: the published methods are *tracking* methods. They need a β(s),
     v(s) and path reference built offline (Goh's figure-8 NLP; Meijer's equilibrium
     lookup). The planner can make that reference by stitching equilibria with rate-limited
     transitions (inertia drift), and add a speed floor v ≥ v_min, which needs Goel's brake
     authority.
- A drift on a straight (κ ≈ 0, β ≠ 0) does not exist as a steady equilibrium in the bicycle
  model's usual sense (a drift equilibrium needs yaw rate r = v·κ_path + β̇ terms). On a
  straight it must be a sequence of linked transients ("inertia drift" or a "pendulum"
  swing). That is the Zhou/Betz problem. This is my inference from the equilibrium definitions
  above, not a published result.

### Gaps
- No OCP with a sideslip reward or minimum-sideslip constraint traded against lap time was
  found. The emergentmind "DriftOCP" summary mentions a cost with a sideslip reward and a
  minimal-exit-time terminal term, attributed to Meijer et al. 2024. The paper itself shows
  quadratic tracking to equilibrium, not a reward, so that summary looks wrong or refers to
  another paper. Unverified. — [emergentmind](https://www.emergentmind.com/topics/driftocp)
- No publication on drifting along a straight or on a pendulum/Scandinavian flick for
  autonomous cars was found. Hu "drift parking" was not reached this session.
- Lot/Massaro rally OCP numbers (β, time vs grip): still paywalled.

## Q3. RL drifting and learned + MPC hybrids

### Takeaway
RL drift work tracks references (human or OCP) with a reward that weights β error against
path error. Its lap-time gains come from the reference, not from the RL. The one controlled
number for drift versus grip comes from a human in CARLA: drifting was about 12 % faster than
normal driving on the same car and map, with arcade-like tyre settings. Learned components
used on real cars at scale are tyre and vehicle models inside NMPC (TRI's diffusion model,
neural tyres), not end-to-end policies.

### Cited Findings
- **Cai, Mei, Tai, Sun & Liu, "High-speed autonomous drifting with deep RL", IEEE RA-L
  5(2):1246-1254, 2020 (SIM, CARLA).** The reward is speed times a weighted sum of cross-track,
  heading and β-error terms, weights [40, 40, 20]. β has the lowest weight, and the reward is
  halved below 6 m/s. The references are human drift laps recorded on a Logitech G920 in CARLA,
  "as fast as possible, drifting on sharp bends". SAC on an unseen map: maximum β 26-29 deg
  (43 deg with other friction and mass), maximum speed 103.7 km/h, 79 km/h average through
  corners (human 79.4). Cross-track error 0.90 m over the lap and 1.17 m in corners; heading
  error 5.7 deg. Lap times: SAC 142.4-145.9 s, against 186-191 s without the slip states
  (SAC-WOS) and 212-216 s for DDPG. **Human drifting 141.69 s against human normal driving
  160.53 s on the same setup (about 12 % faster).** SAC was not run without drifting. —
  [ar5iv 2001.01377](https://ar5iv.labs.arxiv.org/html/2001.01377)
- Zhao et al. 2026 (Q2): SAC tracker with 10-point preview of position and β references,
  Gaussian error rewards, and an end reward that decays exponentially with lap time relative
  to the MLT optimum. Curriculum: straight-line drift, then hairpins, then a karting circuit.
  It beats Cai by about 6-7 % in lap time. Ablations show that MLT-guided training beats
  centreline guidance on lap time, β and offset. — [arXiv 2608.00113](https://arxiv.org/html/2608.00113)
- arXiv 2411.11762 (REAL, high-speed cornering EV): per the search snippet, a β reward plus a
  terminal cornering-time reward, with the constants re-tuned across training stages. Not
  verified beyond the snippet. — [arXiv 2411.11762](https://arxiv.org/pdf/2411.11762)
- GT Sophy line (not drifting): Fuchs et al. 2020 call lap time "a very sparse reward". They
  use a progress proxy plus a wall-contact penalty weighted by a trade-off hyperparameter, so
  style comes from reward weights. — [arXiv 2008.07971](https://arxiv.org/pdf/2008.07971)
- Already in our notes: Cutler & How 2016 (RC), Domberg 2022, Djeumou et al. CoRL 2024
  diffusion model (REAL, Supra and LC500), Werner et al. ICRA 2026 (REAL, AMG). — see
  drift_planners.md.

### Inferences
- The 12 % human figure comes from CARLA's tyre model with the authors' friction settings
  (F3.5, about μ 3.5 in CARLA's tyre-friction parameter, not a real μ). It contradicts our
  asphalt ≤ 0.4 % only on the surface. It most likely reflects CARLA's vehicle physics, not
  real tarmac. Treat it as evidence that the *simulator's* tyre model decides whether drift
  pays, which matters if our style planner is tuned in CARLA or Isaac.
- For a three-style planner, RL is better as the tracker of a planned (β, v, path) reference,
  with the reference's style set upstream, than as the style generator. Reward weights are an
  opaque style knob with no Pareto guarantee.

### Gaps
- Spielberg (neural-network vehicle models, Stanford) and Domberg details were not fetched.
- No RL paper sweeps the β weight against lap time.

## Q4. Parameterising style: time vs drift-angle trade-offs, Pareto fronts, ε-constraints

### Takeaway
I found no published Pareto front or ε-constraint study of lap time against drift angle or
drift fraction, for autonomous cars or minimum-time vehicle dynamics. Style is set implicitly
in each work: by the cost choice (Velenis 2005, min time vs max exit speed), by the reference
(human drift laps, equilibrium tables), by reward weights (Cai 40/40/20), or by "driver
preference" in Toyota's patent (drift_planners.md). That gap is open.

### Cited Findings
- Velenis & Tsiotras 2005: minimum-time and maximum-exit-velocity costs give different
  optimal cornering. — [dcsl isie05a](https://dcsl.gatech.edu/papers/isie05a.pdf)
- Goel 2022: the NMPC horizon gives "selective prioritization of objectives" among path, speed
  and β. Within the tracker, the weights act as a priority knob. —
  [purl mz751hw3069](https://purl.stanford.edu/mz751hw3069)
- Zhao 2026: the end reward is relative to the MLT optimum, so the RL agent is scored as a
  fraction of optimal time. — [arXiv 2608.00113](https://arxiv.org/html/2608.00113)
- A search for weight sweeps returned only reward-weight learning in general racing (arXiv
  2601.16906: a learned progress weight cut lap time by 7 s). Nothing drift-specific. —
  [arXiv 2601.16906](https://arxiv.org/pdf/2601.16906)

### Inferences
- The cleanest style parameterisation consistent with the literature: one minimum-time OCP
  with lap time capped, T ≤ (1+ε)·T*_grip-or-best, maximising a drift measure (∫|β| ds, or
  arc length with |β| > β_min), and a speed floor for D1. ε = 0 gives fastest lap; small ε
  gives "drift where cheap", since the optimiser spends its time budget where drift costs least
  (by our study, hairpins and low-μ corners first). Large ε plus v ≥ v_min plus β-sign linking
  constraints gives D1. Sweeping ε traces the Pareto front. Because grip and drift are separate
  local optima (our IPOPT finding), each ε needs mode-seeded multi-start.
- Q2's quantitative cost data, set against our study: corners where drift is optimal lose
  nothing; elsewhere the cost has not been measured in any paper found. Our planner would
  produce the first such curve.

### Gaps
- D1 judging criteria (angle, line, speed, style) as formal objectives: not researched this
  session. No source found that turns judged drift style into a cost.
- No Pareto or ε-constraint vehicle-dynamics study turned up. A more targeted search ("multi-
  objective minimum lap time", "lap time energy Pareto") might find methods to borrow from
  energy-aware racing, but none drift-specific.

## Q5. Quantitative summary

### Takeaway
Real full-size sustained drifts run at 30-45 deg β, 10-14 m/s on figure-8s and up to 50 km/h on
a 1 km course, with about 1-1.4 m lateral error. The LQR on MARTY reached cm-level error on
quasi-equilibrium paths. The only drift-versus-grip time figure is a simulator one (12 %), and
it is not credible for tarmac.

### Cited Findings
- MARTY, sustained course: 40 deg, 2.5 rad/s, 50 km/h, about 1 km. —
  [Goh thesis](https://purl.stanford.edu/th820zb4174)
- MARTY figure-8: 0.7 rad target, 1.4 m lateral error, 10.8-13.8 m/s (drift_planners.md, via
  arXiv 2606.27914). — [arXiv 2606.27914](https://arxiv.org/html/2606.27914)
- MARTY LQR: centimetre-level tracking on (quasi-)equilibrium paths. —
  [Peterson thesis](https://purl.stanford.edu/vg668bz0151)
- BMW M3 (semi-automated): about 30 deg held for several laps, dry and wet. —
  [arXiv 2405.10859](https://arxiv.org/html/2405.10859v1)
- AMG GT 63 S: 0.66 rad peak vs a 0.6 target, 1.1 m lateral error (drift_planners.md). —
  [arXiv 2606.27914](https://arxiv.org/html/2606.27914)
- CARLA RL: 26-29 deg, up to 104 km/h, 0.9 m cross-track; human drift 12 % faster than human
  grip. — [ar5iv 2001.01377](https://ar5iv.labs.arxiv.org/html/2001.01377)
- CARLA MLT plan: 29.9 deg (at the 30 deg bound), 25 m/s at μ = 0.6; RL tracking costs 12-17 %
  of lap time. — [arXiv 2608.00113](https://arxiv.org/html/2608.00113)

### Inferences
- The tracking errors (about 1 m) are larger than a lane's margin. A D1-style planner should
  keep about 1-1.5 m of track-edge margin while drifting, against tighter margins for grip.

### Gaps
- No real-vehicle figure for the lap-time cost of drift against grip.
- The 2024-2025 tandem runs: no published angles or speeds.
