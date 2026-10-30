# Drift-specific planning and control methods, and how they decide when to drift versus grip

Researched 2026-10-30, about 20 searches and fetches. Abstracts and index pages only: several
full texts (Tavernini 2013 and 2014, Goh 2024 VSD, Weber & Gerdes 2024, the Toyota patent, TRI's
Medium post) returned 403 or were paywalled. Numbers below come from abstracts, press releases or
the related-work section of arXiv 2606.27914 unless a primary page is named. Grounding: our study
(`/home/aeon/repos/autoware-drift/docs/drift-optimality.md`) finds drift optimal only in tight
hairpins on asphalt (<= 0.4 %) and on low friction (1-2 %, up to 33 deg sideslip). The drift
buys earlier rotation, not speed. Grip and drift are separate local optima that IPOPT cannot
choose between.

## Q1. Stanford DDL / TRI line: drift equilibria, MARTY, Supra NMPC, tandem. How do they switch between grip and drift?

### Takeaway
The Stanford/TRI line is about tracking. It stabilises and tracks a drift that is given in
advance: a reference path plus a sideslip profile, or an obstacle-avoidance objective. It never
treats a drift as a lap-time choice. Mode transitions are handled one way only: a single NMPC
whose model (coupled slip, wheel-speed states) covers both regimes, so it can run from drift
back to grip continuously. Whether to drift is set by the task, not decided by the planner.

### Cited Findings
- **Equilibrium-based drift stabilisation** (Voser, Hindiyeh & Gerdes 2010; Hindiyeh & Gerdes
  2014; Velenis et al. 2011; Gonzales et al. 2016; Acosta et al. 2018) stabilises fixed drift
  equilibria without following a path. Bárdos et al. 2020 used an LQR-type MIMO controller
  about one equilibrium and it "did not generalize to varying curvature". —
  [Werner et al., arXiv 2606.27914 (ICRA 2026), related work](https://arxiv.org/html/2606.27914)
- Velenis, Frazzoli & Tsiotras (CDC 2009) compute single-track steady-state cornering
  equilibria with the tyres outside the linear range, i.e. drifting. For each one they give the
  steering angle and front/rear slip needed to hold constant speed, yaw rate and sideslip. The
  IJVAS 2010 journal version stabilises them with a sliding-mode controller using only
  drive/brake torques. —
  [Georgia Tech DCSL, cdc09a.pdf](https://dcsl.gatech.edu/papers/cdc09a.pdf);
  [ijvas09R.pdf](https://dcsl.gatech.edu/papers/ijvas09R.pdf)
- **Goh & Gerdes, "Simultaneous stabilization and tracking of basic automobile drifting
  trajectories", IEEE IV 2016, pp. 597-602.** Lookahead error and sideslip are the reference
  states, and the controller is linearised about an equilibrium of a single-track model. On
  MARTY (a 1981 DeLorean, rear-drive, independent rear motors) it reached up to 45 deg
  sideslip. A feasible drift reference is treated as a sequence of unstable drift equilibria. —
  [Stanford DDL](https://ddl.stanford.edu/publications/conference-proceeding/simultaneous-stabilization-and-tracking-basic-automobile);
  [dblp](https://dblp.uni-trier.de/rec/conf/ivs/GohG16.html)
- **Goh, Goel & Gerdes, "Toward automated vehicle control beyond the stability limits:
  drifting along a general path", ASME JDSMC 2020, doi 10.1115/1.4045320.** Derived "without
  reference to a nearby equilibrium point". The rotation rate of the velocity vector tracks
  the path, and yaw acceleration stabilises sideslip. The AVEC 2018 conference version tracked
  "varying curvature, speed, and sideslip" on a full-scale car. —
  [Stanford DDL](https://ddl.stanford.edu/publications/journal/towards-automated-vehicle-control-beyond-stability-limits-drifting-along);
  [AVEC 2018](https://ddl.stanford.edu/publications/conference-proceeding/controller-automated-drifting-along-complex-trajectories)
- MARTY's figure-8 numbers, from Goh's thesis: 18 m radius, 0.7 rad target sideslip, 1.4 m
  lateral error, 0.08 rad sideslip overshoot, 0.11 rad heading error, 2 rad/s peak yaw rate,
  10.8-13.8 m/s. Goel, Goh & Gerdes 2020 add the brakes to deal with under-actuation. MARTY
  completed a roughly 1 km autonomous course at Thunderhill. —
  [arXiv 2606.27914](https://arxiv.org/html/2606.27914);
  [Gigazine 2019](https://wbgsv0a.gigazine.net/gsc_news/en/20191223-autonomous-delorean-marty);
  [Goh thesis, purl](https://purl.stanford.edu/th820zb4174)
- **TRI Supra NMPC (announced 2022-02-02).** It replans "a whole new trajectory every 20th of
  a second" (about 20 Hz), from a 250 Hz RTK-GNSS INS. Actuators: steering, throttle, clutch,
  sequential gearbox and per-wheel brakes, on an x86 computer. It drifts round obstacles on
  Thunderhill West and "can smoothly transition from dynamic, non-equilibrium drifting to grip
  driving" while respecting road bounds. —
  [Toyota pressroom](https://pressroom.toyota.com/toyota-research-institute-pushes-vehicles-capabilities-to-advance-active-safety/)
- **Goh, Thompson, Dallas & Balachandran, "Beyond the stable handling limits: nonlinear model
  predictive control for highly transient autonomous drifting", Vehicle System Dynamics
  62(10):2590-2613, 2024.** The NMPC is built for deep rear-tyre saturation and uses a
  coupled-slip tyre model with explicit wheel-speed dynamics. Racetrack experiments on a GR
  Supra showed a smooth drift-to-grip transition and robustness to longitudinal-force and
  wheel-speed disturbances. A later paper notes "high computational cost and sensitivity to
  model mismatch". —
  [index (bohrium/arXiv)](https://bohrium.dp.tech/paper/arxiv/2a28f15814cf14ed9abe5f5c27722e72d1a78e53588d1663fa20a21e8e819664);
  [arXiv 2606.27914](https://arxiv.org/html/2606.27914)
- **TRI/Stanford tandem drift (announced 2024-07-23)** used two modified GR Supras at
  Thunderhill: TRI built the lead car, Stanford the chase car. Both run NMPC that plans
  steering, throttle, brake and the trajectory, with a neural-network tyre model trained on
  data from earlier sessions. Gerdes: "The physics of drifting is actually similar to what a
  car might experience on snow or ice." I found no technical paper on the tandem system itself. —
  [TRI news](https://www.tri.global/news/toyota-research-institute-and-stanford-engineering-achieve-worlds-first-fully-autonomous);
  [Toyota UK media](https://media.toyota.co.uk/?p=59720)
- **Weber & Gerdes, "Modeling and control for dynamic drifting trajectories", 2024.** The
  Stanford profile lists it as IEEE T-IV 9(2):3731-3741. OpenAlex lists it as IEEE IV 2024,
  doi 10.1109/IV55156.2024.10588633. Both versions may exist. Its keywords cover weight transfer,
  model mismatch and actuator limits, and it is cited for adding weight transfer and wheel speed
  to the drift model. —
  [OpenAlex](https://api.openalex.org/works/doi:10.1109%2FIV55156.2024.10588633);
  [Stanford profile](https://profiles.stanford.edu/j-gerdes)
- **Djeumou, Lew, Ding, Thompson, Suminaka, Greiff & Subosits, "One Model to Drift Them All:
  Physics-informed conditional diffusion model for driving at the limits", CoRL 2024
  (Outstanding Paper).** A diffusion model trained on unlabelled data from several vehicles is
  conditioned on online measurements and used inside real-time MPC. One model drifts both a
  Supra and a Lexus LC 500, matches task-specific models and generalises better. Djeumou et al.
  2023 learned a tyre model good enough to drift from about 3 minutes of data. The diffusion
  model needs a high-performance CPU. —
  [project page](https://thomasjlew.github.io/publication/one_model);
  [arXiv 2606.27914](https://arxiv.org/html/2606.27914)
- **Toyota patent, "Automated control architecture that handles both grip driving and sliding"
  (US 2023/0022906; US 12606172).** A closed-loop controller covers the grip range, the drift
  range and the transitions between them. It plans a trajectory from grip into drift from the
  vehicle state, the environment and a driver preference. The application states that existing
  methods drift open-loop. A search summary also mentions picking the mode that performs best in
  a region of state space, and a clutch kick or handbrake to start a drift. I could not read the
  full text (403, scanned images). —
  [Justia](https://patents.justia.com/patent/20230022906);
  [USPTO PDF](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12606172)
- Werner, Heintzenberg, Lienkamp & Betz (TUM, ICRA 2026) drift a production Mercedes-AMG GT 63 S
  (combustion engine, locked rear differential) on circles and figure-8s. Their additions are a
  170 ms state predictor (RK2, 10 ms step), a reformulation for the locked differential and
  brake-based speed control. The controller runs at 100 Hz. Results: 0.66 rad peak sideslip
  against a 0.6 target, 1.1 m lateral error and 0.06 rad overshoot, with about 140 ms steering
  and 280 ms torque delay. Without the predictor, oscillation became catastrophic above about
  95 ms of delay. —
  [arXiv 2606.27914](https://arxiv.org/html/2606.27914)

### Inferences
- Nothing in this line chooses drift for lap time. The reference sideslip (or "drift around the
  obstacle") is an input. What we can borrow is the NMPC model ingredients: coupled slip,
  wheel-speed states, a learned tyre model, and the delay predictor. A planner that switches
  modes has to come from elsewhere.
- TUM's locked-differential reformulation matters for us. Our car has a locked/spool rear, and
  most drift controllers assume independently driven rear wheels (MARTY).
- Replanning at about 20 Hz for TRI and 100 Hz tracking for TUM suggests a split we could copy:
  the mode is chosen slowly, by a planner, and a fast tracker serves both regimes.

### Gaps
- The tandem-drift papers (2024-2025: lead/chase formulation, horizons, solve times) did not
  turn up. TRI's Medium post returned 403.
- Horizon length and solve time of Goh et al. 2024 are not reported in the sources I reached.
- From memory, not verified this session: Hindiyeh & Gerdes, "A controller framework for
  autonomous drifting: design, stability, and experimental validation", ASME JDSMC 136(5)
  051015, 2014; Voser, Hindiyeh & Gerdes, "Analysis and control of high sideslip manoeuvres",
  Vehicle System Dynamics 48 (suppl.), 2010; Edelmann & Plöchl, "Handling characteristics and
  stability of the steady-state powerslide motion of an automobile", Regular and Chaotic
  Dynamics 14(6), 2009. All three show drift equilibria are saddle-type unstable and need active
  control. Check before citing.

## Q2. Rally / minimum-time OCP work where drift emerges. What makes drift optimal? Multiple local optima?

### Takeaway
Minimum-time OCP work agrees with our result. Large-sideslip "rally" solutions appear on loose,
low-friction surfaces and in tight hairpins. On paved surfaces the solution has low sideslip.
The drivetrain layout changes the answer. The literature names non-convexity and convergence to
different local minima as the main obstacle, but I found no paper that measures the grip-versus-
drift gap as a percentage of corner time. Our 0.4 % (asphalt) and 1-2 % (low friction) may be
the only such figures.

### Cited Findings
- **Tavernini, Massaro, Velenis, Katzourakis & Lot, "Minimum time cornering: the effect of road
  surface and car transmission layout", Vehicle System Dynamics 51(10):1533-1547, 2013, doi
  10.1080/00423114.2013.813557.** Surfaces: dry and wet paved, dirt, gravel. Layouts: RWD, FWD,
  AWD. "Low-drift paved circuit-like manoeuvres and aggressive/high-drift even counter-steering
  rally like manoeuvres are found depending on terrain/layout combinations." The abstract gives
  no numbers. —
  [Surrey Open Research](https://openresearch.surrey.ac.uk/esploro/outputs/journalArticle/Minimum-time-cornering-the-effect-of/99640964802346)
- **Tavernini, Velenis, Lot & Massaro, "The optimality of the handbrake cornering technique",
  ASME JDSMC 136(4), 2014, doi 10.1115/1.4026836.** On a tight hairpin the handbrake manoeuvre
  is both the minimum-time and the minimum-lateral-space strategy. It is validated against a
  professional rally driver on a loose surface. A "proper yet simple cost function" was needed
  to make the optimal driver use the handbrake, so the OCP did not find it unaided. —
  [Southampton eprints 382093](https://eprints.soton.ac.uk/382093/) (403 when fetched; abstract
  from search summary); [Padova IRIS](https://www.research.unipd.it/handle/11577/3155976)
- Velenis, Tsiotras & Lu, "Optimality properties and driver input parameterization for
  trail-braking cornering", European Journal of Control 14(4), 2008: trail braking emerges as a
  minimum-time solution (already cited in our doc). —
  [bohrium index](https://www.bohrium.com/paper-details/optimality-properties-and-driver-input-parameterization-for-trail-braking-cornering/811897894264111104-2695)
- Berntorp et al. (Lund/Linköping, ACC 2013): on a hairpin, different tyre models give
  "quantitatively different behavior" in the optimal manoeuvre. A Lund paper says online
  generation of time-optimal trajectories is "still not there, mainly because of the inherent
  nonconvexity of the optimal control problem". —
  [LiU ACC'13 page](https://www.fs.isy.liu.se/Publications/Articles/ACC_13_KB_BO_KL_BB_LN.html);
  [Lund PDF](https://lup.lub.lu.se/search/files/4190779/4015826.pdf)
- TU Delft MSc thesis, *Optimal Control for Race Car Minimum Time Maneuvering* (hairpins of
  several radii): precision "is mainly affected by convergence of the solver to various local
  minima". Distance-dependent constraints and warm-starting are the remedies. —
  [TU Delft repository](https://repository.tudelft.nl/record/uuid:c3a0dfec-530e-4672-b4cd-55374817e686)
- Olofsson et al. 2013 study optimal manoeuvres on different road conditions. Cited, not read. —
  [arXiv 2606.27914 refs](https://arxiv.org/html/2606.27914)
- Zhao, Zhao, Wu & Lv (arXiv 2608.00113, July 2026): a minimum-lap-time OCP generates drift
  reference trajectories for a hierarchical RL tracker. Drift is useful because it "rapidly
  aligns the vehicle for corner exits". The abstract gives no comparison with a grip baseline. —
  [arXiv 2608.00113](https://arxiv.org/abs/2608.00113)

### Inferences
- "Drift buys rotation that aligns the car for the exit" is the same mechanism in the handbrake
  hairpin paper (minimum lateral space as well as minimum time), in Zhao et al., and in our
  170 deg result. Ours is consistent with all of them.
- The handbrake paper needed a shaped cost to elicit the technique. That is circumstantial
  evidence that the drift optimum has a small basin of attraction, which is what we see with
  IPOPT. The TU Delft and Lund reports of local minima support it.

### Gaps
- I found no quantitative grip-versus-drift time gap on tarmac or gravel in the abstracts I
  could read. The Tavernini figures would need the full text (DOIs above).
- I found no paper that runs a structured multi-start or a mode enumeration over minimum-time
  cornering OCPs.
- Not checked: Lot/Massaro rally-car optimal control beyond the two Tavernini papers, and the
  Li et al. "minimum-time drifting" papers.

## Q3. Hierarchical approaches: equilibrium libraries, primitives, path-following drift, mode-switch logic

### Takeaway
Hierarchical systems decide the mode outside the controller. The choice comes from geometry
(corner stages), from a hand-placed switch point (Kolter), or from a search over primitives per
mode (Ajanović). The controller then tracks either a drift equilibrium or a grip path.

### Cited Findings
- **Kolter, Plagemann, Jackson, Ng & Thrun, "A probabilistic approach to mixed open-loop and
  closed-loop control, with application to extreme autonomous driving", ICRA 2010, pp.
  839-845.** On "Junior", closed-loop control handles the well-modelled approach, then the car
  switches to open-loop for the poorly modelled sideways slide into a parking spot. It finishes
  within about 2 ft of the target. The switch comes from a probabilistic model, not a
  hand-tuned law. —
  [Google Research](https://research.google/pubs/a-probabilistic-approach-to-mixed-open-loop-and-closed-loop-control-with-application-to-extreme-autonomous-driving/);
  [IEEE Spectrum](https://spectrum.ieee.org/autonomous-car-learns-to-powerlslide-into-parking-spot)
- Cutler & How, "Autonomous drifting using simulation-aided reinforcement learning", ICRA 2016,
  pp. 5442-5448, doi 10.1109/ICRA.2016.7487756: circular drift stabilisation on a small-scale
  car, with "limited trajectory flexibility". —
  [researchr](https://researchr.org/publication/CutlerH16/bibtex);
  [arXiv 2507.23339 summary](https://arxiv.org/html/2507.23339v2)
- **"An aggressive cornering framework for autonomous vehicles combining trajectory planning
  and drift control" (IEEE, 2024; authors not captured).** The corner is split into three stages:
  entry, drift, exit. The planner produces a speed profile and a curvature-optimised path, and a
  switching controller (pure pursuit plus MPC) runs each stage. Validated in simulation and on a
  1/10 RC car. —
  [IEEE Xplore 10588721](https://ieeexplore.ieee.org/document/10588721/)
- "Drift control for cornering maneuver of autonomous vehicles" (Mechatronics, 2018): the
  horizon is split into three region types, each planned with its own algorithm and model, and
  the pieces are concatenated. —
  [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0957415818300771)
- Learning-based drift controllers: Cai et al. 2020 (deep RL, high-speed), Domberg et al. 2022
  (arbitrary trajectories), Tóth et al. 2024 (sim-to-real), Zhou et al. 2025 (adaptive LMPC).
  Most results are in simulation only. Dong, Yu & Xi 2022 (real-time MPC for drift plus
  tracking) and Stano et al. 2024 (NMPC with direct yaw moment) are also simulation only. —
  [arXiv 2606.27914](https://arxiv.org/html/2606.27914)
- Two drift controllers that need no precomputed equilibrium: arXiv 2505.17487 (4WD-4WS,
  circular drift) and Joa et al. 2020 (model-free, measured signals only). —
  [arXiv 2505.17487](https://arxiv.org/pdf/2505.17487);
  [arXiv 2606.27914](https://arxiv.org/html/2606.27914)

### Inferences
- None of these gives a cost-based entry or exit criterion. The drift segment is placed by
  geometry ("this is a corner, drift it"). Our study measures where drift actually pays, which
  argues against copying a geometric rule: on asphalt it would drift corners where grip is
  equal or faster.

### Gaps
- No primary source found for Tsinghua "drift parking" (Zhang et al.) or for the transitions
  from a drift circle to a figure-8 beyond the TUM and MARTY figure-8 tracking.

## Q4. Grip versus drift as a planned decision (hybrid automaton, mode scheduling, MIQP), with real-time results

### Takeaway
Only one academic work frames grip versus drift explicitly as a decision: Ajanović et al., as
task and motion planning (a search over mode-specific motion primitives). The other is the
Toyota patent. Neither reports real-time numbers in the sources I reached. NMPC that "drifts
when beneficial" (Stano, Tavolo) lets the mode emerge from one continuous OCP, which is our
situation and brings our local-optimum problem with it.

### Cited Findings
- **Ajanović, Regolin, Shyrokau, Ćatić, Horn & Ferrara, "Search-based task and motion planning
  for hybrid systems: agile autonomous vehicles", arXiv 2301.10384, accepted in Engineering
  Applications of Artificial Intelligence (2023).** The question is when to switch between modes
  such as drifting and gripping, and how to drive in each. It is posed as TAMP: a guided search
  samples combinations of motion primitives, with several locally approximated models
  (learned drifting models included), and the mode choice needs the feasibility of the
  continuous motion in each mode. A drift controller runs steady-state cornering and a
  path-following controller runs near-straight segments. Simulated mixed-curvature track; code
  at git.io/JenvB. The abstract gives no timing or lap-time numbers. —
  [arXiv 2301.10384](https://arxiv.org/abs/2301.10384);
  [PDF](https://arxiv.org/pdf/2301.10384)
- Stano et al., IEEE T-IV 2024: path-tracking NMPC with steering, longitudinal force
  distribution and direct yaw moment, which "can induce drift when that helps tracking". Tavolo
  et al. (arXiv 2406.02198, 2024) add four-wheel steering and drift "when this is beneficial"
  in emergencies. Both are simulation. —
  [arXiv 2406.02198](https://arxiv.org/abs/2406.02198)
- Meijer, Bertipaglia & Shyrokau (arXiv 2405.10859, 2024): NMPC that holds a high sideslip on a
  standard passenger car (BMW M3), as a driver aid. —
  [arXiv 2405.10859](https://arxiv.org/abs/2405.10859)
- Toyota patent (Q1): it plans the grip-to-drift transition with a closed-loop controller over
  both ranges, choosing by state-space region and driver preference. —
  [Justia](https://patents.justia.com/patent/20230022906)

### Inferences
- For our mode choice, the TAMP framing (a discrete mode per segment, then continuous
  optimisation) is the closest published structure. In our setup: enumerate or sample the mode
  per corner (grip / drift at turn-in / trail-brake slide), warm-start IPOPT from each seed,
  and keep the faster. That is "multi-start, seeded" (roadmap 060) with modes as the seeds. A
  GPU sampler such as MPPI (Williams et al. 2016, in our doc) is the other route to covering both
  basins.
- Gains of 0.4 % on asphalt are smaller than the tracking error a drift controller adds (TUM:
  1.1 m lateral, 0.06 rad overshoot even when tuned). On asphalt, a mode choice that defaults to
  grip and allows drift only where the predicted gain beats a risk margin matches the evidence.

### Gaps
- I found no MIQP or hybrid-automaton mode scheduling for grip versus drift with real-time
  results.
- Ajanović's planning times and results need the full PDF.

## Q5. How small are drift gains over grip: tarmac versus gravel/snow?

### Takeaway
No published figure for the size of the gain turned up. The qualitative evidence agrees with
our numbers: low sideslip is optimal on paved roads, high-drift rally technique on loose
surfaces, and the handbrake is optimal only in tight hairpins.

### Cited Findings
- Paved: "low-drift paved circuit-like manoeuvres". Dirt and gravel, depending on the layout:
  "aggressive/high-drift even counter-steering". —
  [Tavernini et al. 2013](https://openresearch.surrey.ac.uk/esploro/outputs/journalArticle/Minimum-time-cornering-the-effect-of/99640964802346)
- Handbrake drift is optimal in minimum time on a tight hairpin, loose surface. —
  [Tavernini et al. 2014](https://eprints.soton.ac.uk/382093/)
- Gerdes: drifting physics is "similar to what a car might experience on snow or ice". TRI
  presents drift as a safety and avoidance skill, not a speed technique. —
  [TRI](https://www.tri.global/news/toyota-research-institute-and-stanford-engineering-achieve-worlds-first-fully-autonomous);
  [Toyota pressroom 2022](https://pressroom.toyota.com/toyota-research-institute-pushes-vehicles-capabilities-to-advance-active-safety/)

### Inferences
- Our ordering (asphalt <= 0.4 %, hairpins only; low friction 1-2 %, most corners) is
  consistent with every qualitative statement found, and no source contradicts it.

### Gaps
- No quantitative tarmac-versus-gravel percentage was found. The Tavernini 2013 and 2014 full
  texts most likely hold corner times; fetch them via the DOIs.
