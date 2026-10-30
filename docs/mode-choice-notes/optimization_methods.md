# Optimization-based methods for choosing between local optima (grip vs drift) in real-time minimum-time OCPs

Grounding: our L1 is a minimum-time spatial NLP over 200-260 m (50-65 nodes at 4 m, ~1000 variables), CasADi to C, IPOPT + MUMPS, warm solve ~0.5 s (95 % of Haruna solves), re-plan every 1 s; L2 is an RTI NMPC (HPIPM, 50 Hz). Grip and drift are separate valleys. On asphalt the gap is <= 0.4 %, on low mu 1-2 %. At mu 0.45, 8 of 20 cases are undecided ([drift-optimality.md](../drift-optimality.md), sections "How the planner finds its optimum" and "Why differentiation cannot choose").

Source-quality note: this session made about 20 tool calls. Items marked **[verified]** were checked against an abstract or full text in this session. Items marked **[from prior knowledge, unverified here]** are standard references I am confident exist, but this session did not check their details. The report writer should treat their numbers as indicative.

## Q1. Seeded / parallel multi-start NLP in real-time MPC: seeds, cost, selection; racing examples

### Takeaway
The racing literature uses "parallel solves, one per discrete class, keep the fastest feasible" almost exclusively for the **overtaking side choice** (homotopy classes around opponents). In that setting it is the accepted real-time pattern, and seeds come from geometric references, one per class. I found no paper that applies it to grip-vs-drift *driving-technique* modes. Our setting is a direct transfer: two or three classes, one core each, and argmin of time among converged, feasible solutions.

### Cited Findings
- He, Zeng, Sreenath, "Autonomous Racing with Multiple Vehicles using a Parallelized Optimization with Safety Guarantee using Control Barrier Functions", ICRA 2022 (arXiv:2112.06435). The planner computes trajectories in parallel. Each is an MPC around a different **homotopic Bezier-curve reference path** lying laterally between surrounding vehicles, and "the time-optimal trajectory among these different homotopic trajectories is selected". That is seed-per-class plus a min-time selection rule. The abstract gives no solver times or number of parallel problems. [verified, abstract] — [arXiv](https://arxiv.org/abs/2112.06435)
- Reiter, Kirchengast, Watzenig, Diehl, "Mixed-Integer optimization-based planning for autonomous racing with obstacles and rewards", 7th IFAC Conference on Nonlinear Model Predictive Control (NMPC 2021). It first selects a homotopy class, with a mixed-integer formulation over obstacles and rewards, and then solves a continuous OCP within that class. Context is the Roborace platform. This session could not extract solve times from the PDF. [verified, title, venue and the two-stage "homotopy class is selected, then solver" structure; details not extracted] — [PDF](https://publications.syscop.de/Reiter2021a.pdf)
- Liniger, Domahidi, Morari, "Optimization-Based Autonomous Racing of 1:43 Scale RC Cars", Optimal Control Applications and Methods 36(5):628-647, 2015 (doi:10.1002/oca.2123):
  - Obstacle avoidance uses a **high-level corridor planner based on dynamic programming**, which picks the corridor (the discrete side) and generates convex constraints. Linearisation then gives one convex QP per sample.
  - The controllers run at 50 Hz on embedded hardware with solve times "in the range of milliseconds".
  - Pattern: a discrete chooser in front of a local convex solver. [verified, abstract] — [arXiv](https://arxiv.org/abs/1711.07300)
- The official MPCC C++ code adds a front slip-angle constraint and a rear friction ellipse. That keeps the planner in the grip valley by construction, the opposite of our goal. [verified, search snippet of repo] — [GitHub alexliniger/MPCC](https://github.com/alexliniger/MPCC)
- A 2026 hierarchical overtaking paper ablates its explicit **homotopy-selection layer**. Without it, success rates drop, because the continuous MPC must resolve side choice inside a non-convex collision region. Direct evidence that leaving the discrete choice to the NLP is worse. [verified, search snippet only; authors not retrieved] — [arXiv:2607.13348](https://arxiv.org/pdf/2607.13348)
- A 2026 F1TENTH paper identifies topological gaps and then runs an accelerated MPC per gap. It reports 51.6 % shorter manoeuvre time and >81 % overtaking success in dense bottlenecks. [verified, snippet] — [arXiv:2603.09188](https://arxiv.org/abs/2603.09188)
- Racing-stack practice [from prior knowledge, unverified here]: TUM's Roborace/Indy local planner (Stahl, Wischnewski, Betz, Lienkamp, "Multilayer graph-based trajectory planning for race vehicles in dynamic scenarios", ITSC 2019) samples a lattice of candidate paths and picks by cost. That is enumeration of discrete alternatives, then local refinement, not gradient search across them — [arXiv:2005.08664](https://arxiv.org/abs/2005.08664)
- AMZ: Vázquez, Brühlmeier, Liniger, Rupenyan, Lygeros, "Optimization-based hierarchical motion planning for autonomous racing", IROS 2020. It reports a ~20 % lap-time improvement over the previous AMZ approach. [snippet-verified figure] — [arXiv:2003.04882](https://arxiv.org/pdf/2003.04882)

### Inferences
- How seeds are chosen in the literature: (a) a geometric reference per homotopy class (Bezier offsets, corridors from DP), (b) the previous solution shifted (warm start, always one of the seeds), (c) lattice/graph primitives. For grip vs drift the analogous seeds are: the previous plan; a "rear-out at turn-in / trail-brake into slide" primitive, e.g. the stored solution of the same corner type from an offline study or the low-mu continuation; and possibly a pendulum flick. These match the three seeds already proposed in drift-optimality.md.
- Selection rule used everywhere: minimum cost among converged and feasible solutions. Our cost is time, so the rule is direct. Add **hysteresis**, e.g. switch modes only if the gain exceeds a noise threshold of ~0.2-0.5 % or 2 replans agree. On asphalt the valleys differ by <= 0.4 %, comparable to solver tolerance and model noise, so without hysteresis the plan would chatter between modes. The undecided 8/20 cases at mu 0.45 are exactly that regime.
- Compute: k seeds cost k x 0.5 s of CPU in parallel, with no wall-clock increase if each has its own core. A cold drift seed will need more IPOPT iterations than the warm one. Budget it with a max-iteration/time cap and drop it if it is not converged at 0.9 s. A cheaper variant runs the alternate-mode seed only every 2-3 replans, or only when the window contains a corner below a radius/mu threshold.

### Gaps
- I found no paper that does multi-start specifically to choose a *driving technique* (grip vs drift/trail-brake) online.
- I could not extract the Reiter 2021 solve times or the number of classes. The PDF text was not decodable in this session.
- I found no published per-solve compute numbers for parallel homotopy MPC in the He et al. abstract.

## Q2. Continuation / homotopy in minimum-time vehicle OCPs; online use?

### Takeaway
Continuation is standard **offline** practice for converging hard minimum-time vehicle OCPs. Typical forms are parameter sweeps in friction, tyre stiffness or track and parameter variation, and indirect-method path following. I found no paper that runs a full friction continuation inside a real-time loop. The online analogue is cheaper: a warm start *is* one continuation step in time. A cached "drift-valley" solution, continued offline per corner type and mu, can serve as a seed.

### Cited Findings
- Cots, "Geometric and numerical methods for a state constrained minimum time control problem of an electric vehicle", ESAIM: COCV 23(4):1715-1749, 2017. It uses multiple shooting combined with **differential path following (homotopy)** to solve a family of minimum-time vehicle OCPs. This is the canonical numerical-continuation pattern for convergence. [verified, abstract/snippet] — [numdam](https://www.numdam.org/item/COCV_2017__23_4_1715_0); [arXiv:1903.04048](https://arxiv.org/pdf/1903.04048)
- Dal Bianco, Bertolazzi, Biral, Massaro, "Comparison of direct and indirect methods for minimum lap time optimal control problems", Vehicle System Dynamics 57(5), 2019 (online 2018). The paper compares direct collocation with indirect (PMP) approaches for lap-time OCPs. Indirect methods in this group (the PINS solver) rely on continuation for convergence. [title/venue verified; the continuation detail is from prior knowledge] — [PDF](https://research.unipd.it/bitstream/11577/3271001/2/2018%20-%20Comparison%20of%20direct%20and%20indirect%20methods%20for%20minimum%20lap%20time%20optimal%20control%20problems%20PP%20-%20MM.pdf)
- Velenis and Tsiotras (JOTA 2008 / ACC 2005) compute minimum-time velocity profiles along a fixed path with tyre-friction saturation, using a semi-analytic method rather than homotopy. Velenis's 2006 Georgia Tech thesis covers minimum-time manoeuvring numerically. [verified, snippets] — [JOTA PDF](https://dcsl.gatech.edu/papers/jota06.pdf); [thesis](https://repository.gatech.edu/entities/publication/339b9fba-be9f-46d2-b33d-0abffba173ed)
- Tavernini, Massaro, Velenis, Katzourakis, Lot, "Minimum time cornering: the effect of road surface and car transmission layout", VSD 51(10), 2013. On loose surfaces the optimum is a large-sideslip technique; on tarmac it is far less so. [from project doc's literature list; not re-verified here]
- Christ, Wischnewski, Heilmeier, Lohmann, "Time-optimal trajectory planning for a race car considering variable tyre-road friction coefficients", VSD 59(4), 2021 (TUM). It solves the minimum-time OCP with CasADi/IPOPT over a spatially varying friction map, offline. [from prior knowledge, unverified here]
- Limebeer and Massaro, *Dynamics and Optimal Control of Road Vehicles*, Oxford University Press, 2018. The textbook treatment of curvilinear minimum-time OCPs and their numerics. [from prior knowledge]

### Inferences
- **Online use for our L1.** A full mu sweep (say 5-10 continuation steps at ~0.5 s each) does not fit 1 s. Three cheap forms do:
  1. **Offline continuation, online seed.** Precompute drift-valley solutions for corner archetypes (radius x mu x entry speed) by continuation. Store them and map them into the window's curvilinear coordinates as a seed. The online solve is then one ordinary warm-ish solve.
  2. **Tracking the valley over time.** Once a drift seed has converged, subsequent replans warm-start from the shifted drift solution. Receding-horizon warm starting is a continuation in time and keeps the solver in that valley (the same mechanism that currently traps it in grip).
  3. **One- or two-step "fall-off" continuation inside the seed solve.** First solve with the post-peak tyre fall-off removed, where the drift valley is easier to reach, then a second solve with the true fall-off, warm-started from the first. That is 2 x 0.5 s on one core; it fits if run on a separate core every other cycle.
- Continuation does not choose between valleys. It only *reaches* one. It belongs in seed generation; the selection rule still decides.

### Gaps
- I found no paper with online, in-the-loop friction or tyre-curve continuation for a race car.
- This session did not confirm whether Tavernini et al. 2013 or Velenis's rally work used continuation explicitly for convergence.

## Q3. Mixed-integer (MIQP/MINLP) and complementarity (MPCC) formulations for driving modes / tyre regimes

### Takeaway
Mixed-integer MPC for tyre saturation exists, but only in **piecewise-linearised** form for evasive manoeuvres (MILP/MIQCP), not for drifting. Complementarity / nonsmooth OCP tooling (NOSNOC, FESD) is mature for switched systems and friction, with real-time results only on small processes. For our smooth Pacejka-type model there is no true discrete mode, so an explicit integer adds cost without exactness benefit, except for genuine switches (wheel lock, ABS). Not a fit for a 1 s L1 replan; possibly useful offline as a reference "global" check.

### Cited Findings
- Gharavi, De Schutter, Baldi, IEEE Transactions on Intelligent Vehicles, 2024 (title not retrieved). Formulates MPC as an MILP or MIQCP with piecewise approximations of the nonlinear stability and **tyre-saturation constraints**. Concludes hybridisation is an efficient route to real-time MPC in **emergency evasive manoeuvres**. [verified, search summary only; exact title and timings not retrieved]
- Reiter et al. 2021 (Q1) apply mixed-integer optimisation to obstacle *side* and reward choice in racing, not to tyre regime. [verified, title] — [PDF](https://publications.syscop.de/Reiter2021a.pdf)
- Nurkanović and Diehl, "NOSNOC: A software package for numerical optimal control of nonsmooth systems", IEEE Control Systems Letters, 2022. CasADi + IPOPT, piecewise-smooth systems via complementarity, with time-optimal reformulations. Finite Elements with Switch Detection (FESD) is in Numerische Mathematik 2024, with up to five orders of magnitude better accuracy than standard time-stepping for the same compute. [verified, snippets] — [GitHub](https://www.github.com/nurkanovic/nosnoc); [arXiv:2111.06759](https://arxiv.org/abs/2111.06759v5)
- "On Optimal Control of Hybrid Dynamical Systems using Complementarity Constraints" (2025) is a survey/tutorial of the MPCC approach. [verified, link only] — [arXiv:2503.03879](https://arxiv.org/pdf/2503.03879)
- "Real-Time Non-Smooth MPC for Switching Systems" (Dec 2025, submitted to ECC 2026) applies NOSNOC-based MPC to a three-tank process. Real-time MPCC exists, but on small, slow plants. [verified, snippet] — [arXiv:2512.09611](https://arxiv.org/abs/2512.09611v1)
- Drifting NMPC in the literature stays smooth (no integers):
  - Goh et al., VSD 2024, NMPC for highly transient drifting with deep rear saturation, Toyota GR Supra. [snippet] — [arXiv:2310.00716](https://arxiv.org/abs/2310.00716)
  - An NMPC for automated drifting with a standard passenger car (2024). [snippet] — [arXiv:2405.10859](https://arxiv.org/html/2405.10859v1)
  - A dual-envelope constrained NMPC (2026) that assumes front unsaturated / rear saturated in drift. [snippet] — [arXiv:2604.07342](https://arxiv.org/pdf/2604.07342)

### Inferences
- An MPCC solved by IPOPT with relaxation/homotopy on the complementarity is still a local method. It does **not** choose valleys globally. Exact discrete choice needs branch-and-bound (MINLP: Bonmin/Couenne/SCIP), which on a 1000-variable nonconvex OCP is seconds to hours, far from 1 s. [inference, no timing source]
- The useful bit for us: if L1 adds wheel lock or open-differential switching (drift-optimality.md notes the open axle is missing), NOSNOC/FESD-style complementarity is the principled way to model the switch. It remains local, so it still needs seeds.

### Gaps
- I found no MIQP/MINLP formulation of grip-vs-drift for minimum-time driving, and no solve times for MINLP lap-time problems.

## Q4. Smoothing / randomized smoothing / stochastic-gradient methods that bridge modes

### Takeaway
Randomized smoothing (Suh, Pang, Tedrake) replaces the true gradient with the gradient of the cost convolved with noise. This flattens small ridges and, with a large enough noise scale, lets a local method "see" across a ridge. It is equivalent to analytic smoothing (log-barrier relaxation) and is the bridge between gradient methods and path-integral/MPPI sampling. Hybrids of sampling and DDP exist explicitly for local-minima escape. For our ridge (a half-slide 1-3 deg past peak) smoothing would need a noise scale comparable to the valley separation, which costs plan precision. Best used as a seed generator, not as L1 itself.

### Cited Findings
- Suh, Pang, Tedrake, "Bundled Gradients through Contact via Randomized Smoothing", IEEE RA-L 2022 (arXiv:2109.05143). Uses a gradient bundle, the average of gradients at noise-perturbed points, to plan through contact discontinuities. [verified, snippet] — [arXiv](https://arxiv.org/abs/2109.05143)
- Pang, Suh, Yang, Tedrake, "Global Planning for Contact-Rich Manipulation via Local Smoothing of Quasi-dynamic Contact Models", IEEE T-RO 2023 (arXiv:2206.10787). Shows the equivalence of randomized smoothing and analytic smoothing (log-barrier relaxation) and embeds local smoothed optimisation in a global (RRT-like) planner. Results are comparable to RL with dramatically less compute. [verified, snippet] — [arXiv](https://arxiv.org/pdf/2206.10787.pdf)
- Aoyama, Jung, Ratheesh, Theodorou, "Beyond Pure Sampling" style hybrid: a DDP phase follows the gradient, then sampling perturbs the solution to escape local minima in non-convex MPC. [snippet; exact arXiv ID uncertain, possibly 2112.09988] — [arXiv:2112.09988](https://www.arxiv.org/pdf/2112.09988)
- MPPI-IPDDP (Kim and Kim): MPPI for coarse exploration, then interior-point DDP for smooth, constraint-satisfying refinement. This is the "sampler picks the valley, derivatives refine" architecture. [verified, abstract] — [arXiv:2208.02439](https://arxiv.org/html/2208.02439v2)
- Variance-reduced MPPI (2026) uses a local quadratic model (gradient/Hessian) to shape the sampling distribution. [snippet] — [arXiv:2602.03639](https://arxiv.org/html/2602.03639v1)

### Inferences
- The connection to path-integral control: MPPI's update is a softmin-weighted average of perturbed rollouts, i.e. a zeroth-order estimate of the gradient of a smoothed (temperature-dependent) cost. Its temperature and noise scale set how wide a ridge it can jump. Another researcher covers MPPI/GPU in depth.
- For L1, an intermediate option: run IPOPT on a **tyre-smoothed** problem first, with the post-peak fall-off softened (the analytic-smoothing analogue). Then refine on the true model. This is the same as the Q2 "fall-off continuation" and costs one extra solve.

### Gaps
- I found no application of randomized smoothing to minimum-time vehicle OCPs or to tyre force-curve nonconvexity.

## Q5. Real-time NMPC practice and how racing teams deal with local optima

### Takeaway
Real-time racing stacks avoid the local-optimum problem structurally rather than solving it:
- an **offline global race line**, from a minimum-curvature QP or minimum-time OCP solved once with care;
- an online **tracking or short-horizon NMPC** (acados SQP-RTI / FORCES / HPIPM) warm-started by shifting;
- **discrete choices** (overtake side, which candidate path) made by enumeration or a lattice/graph upstream.

None of the stacks reviewed chooses a driving technique (drift vs grip) online. Our L1 is unusual in solving a full minimum-time NLP online every second.

### Cited Findings
- HPIPM (Frison and Diehl, "HPIPM: a high-performance quadratic programming framework for model predictive control", IFAC World Congress 2020). A Riccati-based interior-point QP solver exploiting OCP stage structure. On a chain-of-masses closed-loop test, partially condensed solves ran below 1 ms, faster than OSQP and qpOASES. [verified, snippet] — [arXiv:2003.02547](https://arxiv.org/pdf/2003.02547)
- acados (Verschueren et al., "acados: a modular open-source framework for fast embedded optimal control", Mathematical Programming Computation 14, 2022). Provides SQP and SQP-RTI with multiple shooting, HPIPM as QP solver, CasADi-generated model functions, and globalisation via merit-function line search. Its RTI does one QP per sample, warm-started from the shifted previous solution. [from prior knowledge, unverified here] — [arXiv:1910.13753](https://arxiv.org/abs/1910.13753)
- Liniger et al. 2015 (Q1): 50 Hz, ms solve times, DP corridor chooser plus local convex QP. [verified, abstract] — [arXiv](https://arxiv.org/abs/1711.07300)
- AMZ Driverless (Kabzan et al., "AMZ Driverless: The full autonomous racing system", J. Field Robotics 2020): an MPCC-based controller on the Formula Student car. Separately, learning-based MPC on AMZ reported ~10 % lap-time reduction once the GP dictionary converged. [snippet] — [arXiv:1905.05150](https://arxiv.org/abs/1905.05150)
- TUM (Roborace, Indy Autonomous Challenge): Heilmeier et al., "Minimum curvature trajectory planning and control for an autonomous race car", VSD 58(10), 2020. Uses an offline QP for the global race line, plus an online graph-based local planner (Stahl et al. 2019) and a tube-MPC tracking controller. Local optima of the racing line are handled offline. [from prior knowledge, unverified here] — [arXiv:2005.08664](https://arxiv.org/abs/2005.08664) (Stahl et al.)
- Survey: Betz et al., "Autonomous Vehicles on the Edge: A Survey on Autonomous Vehicle Racing", IEEE Open Journal of ITS 2022. Global racing-line methods are OCPs (SQP/NLP from an initial path, gradient-based, initial-guess dependent) or evolutionary algorithms. [verified, snippet] — [arXiv:2202.07008](https://arxiv.org/pdf/2202.07008)

### Inferences
- Applied to us:
  - Keep the L1/L2 split; it is the standard pattern.
  - Move the discrete choice upstream of IPOPT, as racing teams do for overtaking side, with seeds or a sampler.
  - Use acados/HPIPM-style Riccati structure or MA27/MA57 to cut the per-seed solve time. That frees the cores multi-start needs; on banded problems MA27/MA57 are commonly much faster than MUMPS (unquantified here).
- RTI at L2 has no globalisation and stays in L1's valley by design. That is desired: the mode decision belongs to L1.

### Gaps
- I found no published account of TUM, AMZ or IAC teams detecting or handling *multiple local optima* of the minimum-time OCP online.
- I did not obtain acados RTI timing numbers for a comparable 50-65-stage problem in this session.

## Q6. Published comparisons of local vs global methods for minimum-lap-time problems

### Takeaway
Sparse. The literature compares direct vs indirect *local* methods, or replaces the problem with a convex one that has a global guarantee. I found no study that maps the multiple local optima of a lap-time OCP or quantifies multi-start spread. Our 8/20 undecided cases (valleys within noise at mu 0.45; <= 0.4 % on asphalt) are themselves a data point of that kind.

### Cited Findings
- Dal Bianco et al., VSD 2018/19 (direct vs indirect, both local, for minimum lap time). [title verified] — [PDF](https://research.unipd.it/bitstream/11577/3271001/2/2018%20-%20Comparison%20of%20direct%20and%20indirect%20methods%20for%20minimum%20lap%20time%20optimal%20control%20problems%20PP%20-%20MM.pdf)
- Convex reformulations: for all-wheel-drive electric race cars, a fully convex lap-time formulation "can be efficiently solved with global optimality guarantees". A 2026 paper claims the same globally optimal strategies as direct OCP methods at much lower cost. Both rely on simplified models: point mass / no sideslip, so they cannot represent drift. [snippet] — [arXiv:2603.02339](https://arxiv.org/abs/2603.02339v1); [arXiv:2111.04650](https://arxiv.org/pdf/2111.04650v1)
- Anderson and Ayalew, "Modeling professional drivers with MPC", VSD 2017. An outer optimisation over local-MPC cost weights minimises global manoeuvre time. A global/local decomposition, not a multi-start study. [snippet] — [PDF](https://cecas.clemson.edu/ayalew/Papers/Vehicle%20Systems%20Dynamics%20and%20Control/Papers/Modeling%20Professional%20Drivers_VSD2017/modeling%20professional%20drivers%20with%20MPC.pdf)
- The Betz et al. survey lists evolutionary algorithms as the "global" alternative for racing-line optimisation, at much higher cost. [snippet] — [arXiv:2202.07008](https://arxiv.org/pdf/2202.07008)

### Inferences
- Convex lap-time formulations give global optima only because they drop the sideslip/tyre nonconvexity that *creates* the drift valley. They are no help for grip-vs-drift.

### Gaps
- I found no paper that quantifies the multiple local optima of minimum-lap-time OCPs, or that compares multi-start or global NLP solvers (BARON, etc.) against IPOPT on them.

## Q7 (added). GPU for the optimisation side: GPU IPMs, CasADi on GPU, single small NLP vs batched multi-start

### Takeaway
For **one** small banded NLP like ours (~1000 variables, 50-65 stages), the evidence says the GPU is **slower** than the CPU. The MadNLP + cuDSS optimal-control benchmarks cross over only at roughly 500-5000 grid points (several thousand to tens of thousands of variables); at N=100 the CPU is 5-14x faster. For **batched multi-start** (many seeds at once) GPUs do pay off, but only at batch sizes in the hundreds to thousands, and mostly with QP/ADMM or closed-form solvers rather than full IPMs. At our 2-5 seeds, CPU cores win.

### Cited Findings
- Montoison and Caillau, "Modeling and Optimization of Control Problems on GPUs", arXiv:2510.03932, 2025. OptimalControl.jl + ExaModels + **MadNLP + cuDSS**, compared with MadNLP + MUMPS on CPU. Times in seconds, CPU / GPU:

  | Problem | GPU | N | CPU (s) | GPU (s) | Result |
  |---|---|---|---|---|---|
  | Goddard | A100 | 100 | 0.0146 | 0.158 | CPU ~11x faster |
  | Goddard | A100 | 1000 | 0.134 | 0.211 | CPU ~1.6x faster |
  | Goddard | A100 | 2000 | | | crossover; text: "GPU is faster than CPU after N=2000" |
  | Goddard | A100 | 100,000 | 21.5 | 8.75 | GPU 2.5x faster |
  | Goddard | H100 | 100 | 0.0077 | 0.108 | CPU ~14x faster |
  | Goddard | H100 | | | | crossover at N=2000-5000 |
  | Quadrotor | A100 | 100 | 0.0214 | 0.099 | CPU ~4.6x faster |
  | Quadrotor | A100 | | | | crossover at N~500 |
  | Quadrotor | A100 | 100,000 | | | GPU ~5.9x faster |

  The text also claims speedups of "about 22/55", which are inconsistent with the tables (max ~2.5x / ~6x). Flagged as a likely typo or a different metric. [verified, full HTML tables] — [arXiv:2510.03932](https://arxiv.org/html/2510.03932v2)
- MadNLP + ExaModels + cuDSS on the largest AC-OPF (case70k) is ~18x faster than Ipopt + ExaModels + MA27 on CPU at 1e-8 tolerance. The earlier ExaModels/MadNLP paper reports >20x for AC-OPF, but these are huge problems (tens of thousands of buses). [verified, snippets] — [Julia Discourse](https://discourse.julialang.org/t/examodels-jl-and-madnlp-jl-on-gpus/111377); [arXiv:2307.16830](https://arxiv.org/pdf/2307.16830v1)
- ExaModelsPower.jl reports OPF speedups vs Ipopt + MA27 using a shifted geometric mean to damp very small solve times. Small instances are not the GPU's regime. [snippet] — [arXiv:2510.12897](https://arxiv.org/pdf/2510.12897)
- CusADi (Jeon et al., IEEE RA-L 2025, arXiv:2408.09662):
  - Compiles CasADi expression graphs to CUDA kernels to evaluate **thousands of instances** of one function in parallel, callable from PyTorch.
  - Batched MPC needed a closed-form approximation of the OCP (MIT Humanoid, thousands of IsaacGym envs). It reports ~10x over a comparable CPU MPC, and 10-100x over parallel CPU evaluation depending on transfer overhead.
  - Parallelism *within* one expression is not exploited (future work), so it does nothing for a single solve.
  [verified, snippet] — [arXiv](https://arxiv.org/abs/2408.09662); [GitHub](https://github.com/se-hwan/cusadi)
- WarpMPC (2026): batched SQP-MPC on GPU via ADMM with unrolled LDLᵀ, peak ~279,000 solves/s. Its advantage over baselines (batched BoxOSQP, MPAX, single-core OSQP) appears only for batches above ~2000. Dense BoxOSQP showed **no throughput gain over one CPU core** at small batch. It does not compare with HPIPM. [snippet] — [arXiv:2607.11603](https://arxiv.org/abs/2607.11603)
- CuPIQP (EPFL PREDICT): GPU interior-point QP for batches of small-to-medium OCP-structured QPs. No HPIPM comparison found. [snippet] — [ecosyste.ms entry](https://awesome.ecosyste.ms/projects/github.com%2Fpredict-epfl%2Fcupiqp)
- ExaTron.jl (JuliaCon 2021): a batch solver for many small NLPs on GPUs, for ADMM-decomposed OPF. [snippet] — [talk](https://pretalx.com/juliacon2021/talk/LMLJS8/)
- cuOSQP: I did not retrieve a source in this session. GPU OSQP is known (Schubiger, Banjac, Lygeros, "GPU acceleration of ADMM for large-scale quadratic programming", JPDC 2020) [from prior knowledge, unverified here]. Its gains are for large QPs.

### Inferences
- **Single L1 solve on GPU: no.** At ~1000 variables we sit near the Goddard/Quadrotor N=100-200 range, where the CPU is 5-14x faster. A sequential banded factorisation has too little parallel work per Newton step to amortise kernel launches. This confirms drift-optimality.md. Per-solve speed should come from structure: a Riccati-based solver (HPIPM inside acados SQP, or IPOPT with MA27/MA57) and fewer iterations.
- **Batched multi-start on GPU: only at large batch.** 2-5 seeds per window fits on CPU cores (one IPOPT per core, ~0.5 s each). The GPU pays off only for hundreds or thousands of seeds, e.g. a dense seed sweep over slide onset point x direction x entry speed. Even then the tools that scale (CusADi, WarpMPC, CuPIQP) are QP/ADMM or closed-form, not a full nonconvex IPM. The practical form would be a GPU batch of cheap RTI/QP-level (or sampling) evaluations to rank seeds, then 2-3 IPOPT refinements on CPU. That overlaps with the MPPI researcher's scope.
- CusADi could cheaply evaluate our existing CasADi dynamics/cost for thousands of candidate seeds at once, as a ranking step. It would reuse the same symbolic model without a separate GPU model.

### Gaps
- I found no benchmark of a single ~1000-variable, 50-65-stage NLP on GPU vs HPIPM/acados on CPU; the crossover numbers above are from different OCPs (Goddard, Quadrotor).
- I found no published head-to-head of batched GPU QP/NLP solvers against HPIPM.
- I did not retrieve cuOSQP or cuDSS small-matrix numbers specifically.
