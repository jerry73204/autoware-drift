# Sampling, hybrid sampling+gradient, and learning-based methods for grip/drift mode choice (with a GPU section)

Grounding (our system, from /home/aeon/repos/autoware-drift/docs/drift-optimality.md): L1 is a minimum-time direct-transcription NLP, 200-260 m window, 50-65 nodes 4 m apart (~1000 variables, banded), CasADi + IPOPT (MUMPS), warm solve <0.5 s for 95 % of Haruna solves, 1 s re-plan; L2 is RTI NMPC with HPIPM at 50 Hz. Grip and drift are separate local optima; drift gains <=0.4 % on asphalt hairpins and 1-2 % on low friction, and the drift valley starts trail braking ~25 m before the corner, so the choice must be made early in a long window. The doc already ranks "seeded multi-start" first and "a sampler picks the mode, derivatives refine" third.

Method of this note: ~22 search/fetch calls. The Williams et al. 2017 IT-MPC paper (arXiv 1707.02342) was read in full text; MPPI-Generic's benchmark section was read in full; most other items come from abstracts/search snippets and are marked as such. Items I cite from memory without having verified them in this session are flagged "(unverified)".

## 1. MPPI / information-theoretic MPC (Williams, Theodorou et al., AutoRally) and its variants

### Takeaway
MPPI is proven for aggressive, sliding driving on a real 1:5 car on dirt, but with a 2-3 s horizon, 40-60 Hz, a few thousand samples, track limits only as (time-decaying) impulse penalties, and a hand-shaped cost including an explicit sideslip cap; it reached ~9 m/s, was conservative (top speed ~1 m/s below target) and failed laps at the highest target. Its natural role for us is a short-horizon mode probe, not a replacement for a 200-260 m minimum-time planner.

### Cited Findings
- Original MPPI on hardware: Williams, Drews, Goldfain, Rehg, Theodorou, "Aggressive Driving with Model Predictive Path Integral Control", ICRA 2016 — GPU implementation controlling the fifth-scale AutoRally car on a dirt track, compared against a CEM-based MPC; described by the MPPI-Generic authors as the first demonstration of GPU sampling MPC on real hardware — [IEEE Xplore 7487277](https://ieeexplore.ieee.org/document/7487277); [MPPI-Generic arXiv 2409.07563](https://arxiv.org/html/2409.07563v2)
- Learned-dynamics variant: Williams et al., "Information Theoretic MPC for Model-Based Reinforcement Learning", ICRA 2017 — neural-network dynamics replaced the physics model with comparable results on AutoRally; poster reports >8 m/s with speed adjusted into corners — [UW PDF](https://homes.cs.washington.edu/~bboots/files/InformationTheoreticMPC.pdf); [poster](https://nolanwagener.github.io/media/mppi/poster.pdf)
- Journal-length treatment: Williams, Drews, Goldfain, Rehg, Theodorou, "Information Theoretic Model Predictive Control: Theory and Applications to Autonomous Driving", arXiv 1707.02342 (2017; I believe later in IEEE T-RO 2018, venue not verified in this session) — [arXiv 1707.02342](https://arxiv.org/pdf/1707.02342.pdf). Numbers read from the full text:
  - Real-time: "control loops from 40-60 Hz using a few thousand samples of 2-3 second long trajectories"; "sampling 1200, 2.5 second long trajectories at 40 Hz corresponds to approximately 4.8 million queries to the full non-linear dynamics of the vehicle every second"; each sample uses 4-16 CUDA threads. Onboard GPU: Nvidia GTX 750 Ti, quad-core i7. Experiment parameters: 40 Hz, 2 s horizon, lambda 12.5.
  - Constraints by cost: track cost = costmap + 0.9^t * 10000 * I(off track) — a *time-decaying* impulse; without the decay "a strong disturbance can push the importance sampling trajectory far off the track, which results in most samples receiving the impulse cost and being rejected, this destabilizes the optimization" (works in sim, fails on the real car).
  - Drift handling: a "stabilizing cost" penalises sideslip quadratically up to 0.75 rad (~42 deg) and rejects samples beyond it. With the NN model the controller "decreases speed by performing a small slide into turns", sometimes lifting the front-left wheel, then slides slightly on exit — i.e., a mild slide emerged from sampling, not from a mode variable.
  - Results over >1700 laps, targets 6/8.5/11 m/s: at 11 m/s only IT-MPC counter-clockwise completed all 100 laps; top speed 9.06 m/s; IT-MPC runs ~1 m/s below target (conservative). CEM-MPC was faster but only 66.32 % lap success at 11 m/s and, with the basis-function model at 8.5 m/s, hit barriers on >50 % of trials: "unable to finely discriminate between trajectories which do and do not contact the barrier".
  - Model error: clockwise the NN predicted oversteer when the car understeered (asymmetric training data); the car still lapped.
  - Authors' claims of fit: vehicle dynamics are "highly non-linear, but not unstable" — good for sampling; "highly unstable systems are difficult to sample useful trajectories from". Chattering: "the stochastic nature of the sampling procedure can lead to significant chattering", removed by Savitzky-Golay smoothing of the output sequence. A claim that the update is the global minimum of the KL objective "as long as the sampling procedure sufficiently explores the state space".
- Warm-start fragility: Stable MPPI (Georgia Tech) notes MPPI reuses the previous optimised sequence as warm start, and that a large disturbance can make that sequence an unsafe initialisation; fix uses a path tracker to supply the sampling reference — [GT repository](https://repository.gatech.edu/entities/publication/590c0a2d-76f4-468e-8b50-7a1c91432aef)
- Smooth MPPI: Kim et al., "Smooth Model Predictive Path Integral Control without Smoothing", RA-L 2022 / IROS 2022 — samples derivative actions (control rates) as inputs to remove chattering without a post-filter; outperforms MPPI + smoothing baselines — [arXiv 2112.09988](https://arxiv.org/html/2112.09988v7)
- Shield-MPPI: Yin & Tsiotras, arXiv 2302.11719 — combines control barrier functions with MPPI, tested on an aggressive racing platform in simulation and hardware — [arXiv 2302.11719](https://arxiv.org/pdf/2302.11719)
- Chance-constrained IT-SMPC with safety shielding (2024) exists as a further constrained variant — [arXiv 2408.00494](https://arxiv.org/pdf/2408.00494)
- Biased-MPPI (TU Delft, RA-L 2024): mixes ancillary (classical or learned) controllers into the sampling distribution; demonstrated on a small rally car — [TU Delft repository](https://repository.tudelft.nl/file/File_5b491050-cd30-44b7-9cb8-1ca8fe8a2846)
- Variant catalogue in MPPI-Generic's survey: implemented MPPI, Tube-MPPI, Robust MPPI (RMPPI); literature extensions listed include colored-noise, residual, biased, spline-interpolated, Stein-variational-guided, log-MPPI, CoVO-MPC, U-MPPI, smooth MPPI, Tsallis, constrained covariance steering, multi-modal and risk-aware MPPI — [arXiv 2409.07563](https://arxiv.org/html/2409.07563v4)
- (unverified, from memory) Tube-MPPI: Williams et al., "Robust Sampling Based Model Predictive Control with Sparse Objective Information", RSS 2018; RMPPI: Gandhi, Vlahov, Gibson, Williams, Theodorou, "Robust Model Predictive Path Integral Control: Analysis and Performance Guarantees", RA-L 2021 (arXiv 2102.09027); log-MPPI: Mohamed et al., RA-L/IROS 2022 (log-normal mixture sampling for better exploration); Stein Variational MPC: Lambert et al., CoRL 2020 (arXiv 2011.07641) — keeps a *multimodal* particle distribution over control sequences, which is directly the "several valleys" structure. These four were searched but the search returned only Smooth MPPI.

### Inferences
- MPPI's evidence on sliding is real but modest: 1:5 car, dirt, "small slide" entries, sideslip capped at 42 deg by cost. No reviewed source shows MPPI choosing between a deliberate drift line and a grip line under a *minimum-time* objective; its cost was speed-tracking + track penalty.
- Horizon mismatch: published MPPI horizons are 2-3 s (~100 steps). Our window is 200-260 m (roughly 8-20 s at hairpin-to-straight speeds). Sample efficiency degrades with horizon length and control dimension (motivating CEM-GD and log-MPPI); a raw full-window MPPI would need far more samples or a coarse control parametrisation (splines/knots).
- The drift decision begins ~25 m (≈1-2 s) before the corner: that falls *inside* an MPPI-style 2-3 s horizon if the probe is centred on the upcoming corner, so a short, corner-local sampler is the geometrically right tool.
- Constraint handling by penalties is the main reason not to let MPPI replace IPOPT: the road edge and motor limits become soft; CEM's failure (cannot discriminate barrier-contacting samples) is a warning about elite-selection on mixed penalty/time costs.
- Stein variational MPC / multi-modal MPPI are the natural sampling variants for "keep both grip and drift hypotheses alive", since vanilla MPPI's weighted average across two valleys can land on the ridge (half-slide) — exactly the bad in-between our doc describes. Weighted averaging is a real hazard for bimodal problems; a mode chooser should take argmin/elite clusters per mode, not the mean.

### Gaps
- Exact T-RO 2018 venue and final numbers for 1707.02342 not verified (preprint used).
- No source found on MPPI with a full two-track/combined-slip tyre model for minimum-time racing; AutoRally used basis-function or NN bicycle-like models.
- No quantitative comparison of MPPI vs NLP lap time on the same car found.

## 2. CEM / CMA-ES style planners for vehicles

### Takeaway
The best vehicle evidence for CEM is a negative one: in the AutoRally comparison CEM-MPC was faster but much less safe than MPPI (66 % lap success at 11 m/s; >50 % barrier collisions in one setting). CEM is better used over a low-dimensional mode/seed parametrisation than over raw control sequences.

### Cited Findings
- CEM-MPC vs IT-MPC on AutoRally: CEM faster but 66.32 % lap success at the 11 m/s target, collisions on >50 % of trials at 8.5 m/s with the basis-function model; "unable to finely discriminate between trajectories which do and do not contact the barrier" — [Williams et al., arXiv 1707.02342](https://arxiv.org/pdf/1707.02342.pdf)
- Same cost and parameters used for both; elite threshold >0.8 percentile; authors note a gradient method would need a smooth cost where samplers do not — [arXiv 1707.02342](https://arxiv.org/pdf/1707.02342.pdf)

### Inferences
- For mode choice the decision space is small: {grip, drift-same-direction, pendulum/flick} x {slide onset s, e.g. brake-in vs power-on} x maybe {peak sideslip}. CEM/CMA-ES over these 2-4 parameters, each candidate scored by a cheap rollout or even by the IPOPT solve itself, is cheap and avoids CEM's weakness on high-dimensional control sequences.

### Gaps
- No CMA-ES vehicle racing/drifting planner with real-time numbers found in this session.

## 3. Hybrid sampling + gradient (sampling picks the basin, local solver refines)

### Takeaway
The pattern is well established in robotics/model-based RL (CEM-GD: 100x fewer samples than CEM; cuRobo: many parallel seeds + L-BFGS, ~50 ms, ~60x faster than prior optimisers; RRT-seeded iLQG to escape local minima; MPPI-IPDDP), but I found no published vehicle racing/drift example of "sampler chooses grip vs drift, NLP/IPOPT refines" with reported gains. It is a sound, unoccupied design point; our doc's seeded multi-start is its simplest instance.

### Cited Findings
- CEM-GD: Huang, Lale, Rosolia, Shi, Anandkumar, "CEM-GD: Cross-Entropy Method with Gradient Descent Planner for Model-Based RL", arXiv 2112.07746 (2021). First step: many CEM samples explore, top trajectories initialise gradient descent; later steps use far fewer samples. MuJoCo: beats CEM with 100x fewer samples per step, ~25 % less time, ~10 % less memory. Motivation stated: zeroth-order scales poorly with horizon/action dimension; first-order gets stuck in local optima on non-convex landscapes — [arXiv 2112.07746](https://arxiv.org/abs/2112.07746)
- Earlier interleaved CEM + gradient planning (2020) — [arXiv 2004.08763 summary](https://api.emergentmind.com/papers/2004.08763)
- cuRobo (Sundaralingam et al., NVIDIA, arXiv 2310.17274): global trajectory optimisation by many parallel seeds + L-BFGS + parallel noisy line search + particle optimiser; ~50 ms average, ~60x faster than prior trajectory optimisers (manipulators, not vehicles) — [arXiv 2310.17274](https://arxiv.org/abs/2310.17274v2)
- SBP-guided MPC (Hannigan et al., 2019): RRT trajectory warm-starts iLQG to overcome local minima — [summary](https://bengravell.github.io/knowledge-base/papers/2019_hannigan_sbp_guided_mpc_to/)
- MPPI-IPDDP: MPPI explores (escapes local optima but produces coarse "zigzag" trajectories), IPDDP refines into smooth constrained trajectories — [arXiv 2208.02439](https://arxiv.org/html/2208.02439v2)
- Learned multimodal proposals refined by a sampling method for driving MPC in traffic — [arXiv 2310.02918](https://arxiv.org/html/2310.02918v1)

### Inferences
- For us the "sampler" need not sample controls at all: the discrete choice is few-dimensional, so a handful of mode seeds (grip, drift, flick) solved in parallel IPOPT instances = exact refinement of each basin; a sampler adds value only if seeds must be *found* (onset point, depth of slide) rather than hand-written.
- A cheap two-stage scheme fits the 1 s budget: (a) GPU rollouts of a few hundred/thousand parameterised mode candidates through a corner-local horizon (~3-5 s) with a two-track model, scored by time-to-gate + penalties (<1-5 ms on a modern GPU, see section 6); (b) the best 2-3 distinct basins (cluster by sign of yaw-rate excess / sideslip onset) seed parallel IPOPT solves (~0.5 s each on separate cores); (c) keep the fastest feasible. Gains are capped by the doc's own numbers (<=0.4 % asphalt, 1-2 % low mu).

### Gaps
- No vehicle racing or drifting paper found reporting lap-time gains from a sampling warm-start of an NLP/NMPC. Searches for "MPPI-initialized NMPC racing" returned only learned MPPI initialisers and non-racing hybrids.

## 4. Learned warm-starts, mode classifiers, value functions, generative proposals

### Takeaway
Learned initialisers for minimum-time racing OCPs now exist (F1-data raceline seeds, ITSC 2026; BC+RL initial-guess policies incl. an F1 track), and TRI's CoRL 2024 diffusion work shows learning inside a real-car drifting MPC — but the diffusion model there generates *dynamics-model parameters*, not mode proposals. No paper found that learns a grip-vs-drift classifier or drift-mode warm start for an NLP.

### Cited Findings
- Shehadeh et al., "Efficient Trajectory Optimization for Autonomous Racing via Formula-1 Data-Driven Initialization", ITSC 2026, arXiv 2603.07126 — NN predicts a raceline offset from local track geometry (no dynamics), trained on F1 telemetry from 17 tracks; seeds a minimum-time OCP; faster convergence and significantly lower runtime vs geometric baselines while preserving final lap time (abstract-level) — [arXiv 2603.07126](https://arxiv.org/pdf/2603.07126); [Lamarr](https://lamarr-institute.org/publication/efficient-trajectory-optimization-for-autonomous-racing-via-formula-1-data-driven-initialization/)
- "Faster Model Predictive Control via Self-Supervised Initialization Learning", arXiv 2408.03394 — warm-start policy from behaviour cloning of MPC rollouts, then RL fine-tuning for solve time; helps deterministic and sampling MPC; F1-track path-tracking demo; the guess only moves the starting point, constraints still enforced by the MPC — [arXiv 2408.03394](https://arxiv.org/abs/2408.03394v3)
- TransformerMPPI (arXiv 2412.17118, Dec 2024): transformer learns the initial control sequence for MPPI; better average cost, sample efficiency and speed than MPPI in obstacle avoidance and autonomous racing (simulation) — [arXiv 2412.17118](https://web3.arxiv.org/pdf/2412.17118)
- Djeumou, Lew, Ding, Thompson, Suminaka, Greiff, Subosits, "One Model to Drift Them All: Physics-Informed Conditional Diffusion Model for Driving at the Limits", CoRL 2024 (PMLR v270; Outstanding Paper Award per search summary) — conditional diffusion model over the parameters of a physics-informed dynamics model, conditioned on online measurements, inside real-time MPC; one model drifted a Toyota Supra and a Lexus LC 500 on different tyres/roads, matching task-specific models and generalising better — [PMLR](https://proceedings.mlr.press/v270/djeumou25a.html); [project page](https://thomasjlew.github.io/publication/one_model)
- TRI NMPC drifting line: NMPC for highly transient drifting with deep rear saturation and drift-to-grip transitions; obstacle-drifting demo with NMPC on a GR Supra — [summary](https://www.wevolver.com/article/researchers-program-an-autonomous-vehicle-to-drift-around-obstacles-using-nonlinear-model-predictive-control-nmpc); [Bohrium summary](https://bohrium.dp.tech/paper/arxiv/2a28f15814cf14ed9abe5f5c27722e72d1a78e53588d1663fa20a21e8e819664)
- Diffusion as an MPC warm start (non-vehicle): D-SafeMPC — diffusion proposes, MPC projects onto feasible set, fed back into denoising; poor initial trajectories can stop MPC converging — [arXiv 2607.10842](https://www.alphaxiv.org/abs/2607.10842.md); object-centric diffusion warm-starts for collision-free MPC — [arXiv 2601.02873](https://arxiv.org/html/2601.02873v2); diffusion-based approximate MPC handles multimodal action distributions better than L2 regression — [arXiv 2504.04603](https://arxiv.org/html/2504.04603v3)

### Inferences
- Our setting is ideal for offline imitation of a mode label: we already solve both seeds offline in the study (20 corners x mu levels). A classifier on (corner geometry curvature profile, entry speed, mu estimate, gradient) -> {grip, drift, undecided} is cheap to train, trivially real-time, and safe because it only picks a seed; IPOPT still owns feasibility. Its failure mode is distribution shift (new surfaces); mitigate by always also running the previous-plan seed.
- Regression warm-starts (L2) average modes and land on the ridge; for a multimodal problem use a classifier + per-mode regression, or a generative (diffusion/mixture) model — consistent with the diffusion-MPC findings above.
- Given gains of <=0.4 % on asphalt, the learned chooser is only worth building for low-mu surfaces.

### Gaps
- No learned drift-vs-grip classifier or learned value function for corner mode choice found.
- Whether the F1-initialisation paper handles drift: not covered (asphalt racelines).

## 5. RL drifting and racing as evidence of mode choice / proposal generator

### Takeaway
Model-free RL (SAC-family) has produced high-speed drifting through sharp corners in simulation (Cai et al., RA-L 2020) and superhuman racing in Gran Turismo (Sophy, Nature 2022), so a policy can implicitly choose when to slide; but it gives no constraint guarantees, needs a high-fidelity simulator and long training, and is best used for us as an offline proposal/seed generator or label source, not as the planner.

### Cited Findings
- Cai, Mei, Tai, Sun, Liu, "High-speed Autonomous Drifting with Deep Reinforcement Learning", IEEE RA-L 5(2):1247-1254, 2020, doi 10.1109/LRA.2020.2967299, arXiv 2001.01377 — SAC, no explicit motion equations, drifting framed as trajectory following with error-based states/rewards; high-speed drifting through sharp corners on unseen maps; generalises to different mass and tyre friction. Simulator: a secondary source says CARLA; I did not confirm (I recall it as CARLA-based, unverified) — [arXiv/ar5iv 2001.01377](https://ar5iv.labs.arxiv.org/html/2001.01377); [PolyU record](https://research.polyu.edu.hk/en/publications/high-speed-autonomous-drifting-with-deep-reinforcement-learning/); [secondary CARLA mention](https://pp.bme.hu/tr/article/view/18581)
- Yu et al., "Curriculum reinforcement learning-based drifting along a general path", 2024 (Cambridge), validated in CARLA — [Cambridge](https://www.cambridge.org/core/product/6787BE43C312ADB27B317DABDDD2E6EE)
- Wurman et al. (Sony AI), "Outracing champion Gran Turismo drivers with deep reinforcement learning", Nature 2022 — QR-SAC, trained in real time in GT Sport (game speed cannot be changed), beat top e-sports drivers head-to-head — [Sony AI](https://ai.sony/publications/outracing-champion-gran-turismo-drivers-with-deep-reinforcement-learning); [UT PDF](https://www.cs.utexas.edu/~pstone/Papers/bib2html-links/nature22.pdf); [Sony blog](https://ai.sony/blog/blog-019/)
- (unverified, from memory; search did not return it) Cutler & How, "Autonomous drifting using simulation-aided reinforcement learning", ICRA 2016 — PILCO-style model-based RL with simulator priors learning a sustained drift on a small RC car.

### Inferences
- RL results show drifting *emerges* when rewarded, but none of these report a minimum-time comparison of drift vs grip per corner, so they are not evidence that drift is faster — consistent with our finding that it rarely is.
- Use case for us: an RL policy trained in swb/Isaac on low-mu surfaces could generate drift-mode rollouts as IPOPT seeds; cost-benefit is poor vs hand-written seeds given 1-2 % gains.

### Gaps
- No RL paper quantifying lap-time gain from drifting vs grip found.

## 6. GPU: where it helps L1/L2 and where it doesn't (throughput, libraries, batch vs single)

### Takeaway
GPUs excel at (a) many cheap rollouts (MPPI/CEM: thousands of 100-step rollouts in 0.1-0.4 ms on an RTX 4090) and (b) large batches of small optimisation problems (batched QP/SQP: 10^3-10^4 instances), and at single very large NLPs (MadNLP+cuDSS ~18x on 70k-bus OPF). A single ~1000-variable banded NLP like our L1 window (or a single L2 QP) sits in the regime where GPU launch/transfer overheads dominate and CPU Riccati/sparse solvers win. So: GPU for the mode chooser (rollout sampler, or a learned proposal net), optionally for batched multi-start if we ever want tens of seeds; not under IPOPT or HPIPM.

### Cited Findings
- MPPI-Generic (Vlahov, Gibson, Gandhi, Theodorou, arXiv 2409.07563, 2024; C++/CUDA; MPPI, Tube-MPPI, RMPPI; github ACDSLab/MPPI-Generic). Benchmark: differential-drive model, 100 steps, dt 0.02 s, costmap cost; i9-13900K + RTX 4090, mean ms per iteration — 128 samples: MPPI-Generic 0.123, AutoRally 0.198, TorchRL 24.47, Nav2 (CPU) 0.390; 2048: 0.146 / 0.214 / 28.21 / 6.72; 16384: 0.389 / 0.338 / 28.60 / 62.26. Jetson Orin Nano 16384: 12.07 ms (AutoRally 9.94; Nav2 334.9). Jetson Nano 2048: 7.30 ms. GPUs tested: 1050 Ti, 1060, 1080 Ti, 2080, 3080, 4090. Paper says the GPU "is not fully utilized at small sample sizes"; TorchRL "heavily CPU-bound"; text and table disagree on which of MPPI-Generic/AutoRally wins at 16384 on the 4090 — [arXiv 2409.07563](https://arxiv.org/html/2409.07563v4); [benchmarks page](https://acdslab.github.io/mppi-generic-website/benchmarks)
- AutoRally 2017 rate as a fidelity anchor: 1200 x 2.5 s rollouts at 40 Hz = ~4.8 M full nonlinear vehicle-dynamics evaluations/s on a GTX 750 Ti, 4-16 CUDA threads per sample — [arXiv 1707.02342](https://arxiv.org/pdf/1707.02342.pdf)
- CusADi (Jeon, Hong, Lee, Khazoom, Kim, RA-L Feb 2025): compiles CasADi functions to CUDA kernels for batched evaluation of thousands of instances (callable from PyTorch); ~10x vs a similar CPU MPC, 10-100x vs parallel CPU depending on transfer overhead — [arXiv 2408.09662](https://arxiv.org/abs/2408.09662); github se-hwan/cusadi. Relevance: our CasADi dynamics/tyre functions could be batched on GPU unchanged for rollouts.
- MadNLP + ExaModels (Pacaud & Shin, CDC 2024): GPU interior-point NMPC with cuDSS sparse Cholesky / hybrid KKT; ~18x vs Ipopt+MA27 on the largest AC-OPF case (case70k); NVIDIA reports >10x vs an AMD EPYC 7443 CPU; condensed-space KKT is more ill-conditioned — speedups are for *large single* problems — [arXiv 2403.15913](https://arxiv.org/html/2403.15913v2); [Julia discourse](https://discourse.julialang.org/t/examodels-jl-and-madnlp-jl-on-gpus/111377); [NVIDIA blog](https://developer.nvidia.com/blog/nvidia-cudss-library-removes-barriers-to-optimizing-the-us-power-grid)
- ReLU-QP (Bishop, Zhang, Gurumurthy, Tracy, Manchester, ICRA 2024): ADMM as a weight-tied ReLU network on GPU; competitive with CPU solvers (OSQP, ProxQP) on small-medium problems, order-of-magnitude faster on large (high-dimensional) MPC; weight matrices precomputed offline (limits fast-changing linearisations, per third-party summary) — [arXiv 2311.18056](https://arxiv.org/pdf/2311.18056)
- cuOSQP: GPU OSQP using indirect PCG instead of factorisation (mentioned in ReLU-QP search context) — [ar5iv 2311.18056](https://ar5iv.labs.arxiv.org/html/2311.18056)
- GATO (arXiv 2510.07625, 2025): batched trajectory optimisation for batches of tens to low hundreds; 18-21x over CPU and 1.4-16x over GPU baselines as batch grows — [arXiv 2510.07625](https://arxiv.org/html/2510.07625v1)
- WarpMPC (arXiv 2607.11603, 2026 preprint): ADMM + unrolled LDL^T in JAX/Warp; 8,000-250,000 SQP iterations/s across batches up to 10,000; 3-25x over baselines — [arXiv 2607.11603](https://arxiv.org/abs/2607.11603)
- jaxipm (arXiv 2606.26341, 2026 preprint): described as the first GPU-batched NLP solver, IPOPT-based, in JAX; quadrotor NMPC benchmarks — [arXiv 2606.26341](https://arxiv.org/abs/2606.26341)
- TurboMPC (arXiv 2606.24039, 2026 preprint): GPU differentiable MPC with inequality constraints, deployed on a full-scale car; batched Bayesian tuning gave faster laps than hand tuning — [emergentmind](https://www.emergentmind.com/papers/2606.24039)
- cuRobo: parallel seeds + L-BFGS on GPU ~50 ms per hard problem, ~60x faster than prior optimisers — [arXiv 2310.17274](https://arxiv.org/abs/2310.17274v2)

### Inferences
- Per-method verdict for our stack:
  - **MPPI/CEM mode probe (yes, GPU):** At 100 steps x 2048 samples MPPI-Generic takes ~0.15 ms on a 4090 with a simple model. A two-track model with combined-slip tyres is perhaps 10-50x the arithmetic of a diff-drive/bicycle step (my estimate, not measured); even at 100x it is ~15 ms per iteration — far inside a 1 s re-plan, and still fine at 50 Hz if desired. The AutoRally anchor (4.8 M full-dynamics evals/s on a 2014 GTX 750 Ti) supports this. Recommended libraries: MPPI-Generic (C++/CUDA, fastest), or CusADi to reuse our CasADi model on GPU; JAX/Isaac batched rollouts also viable (no benchmark found in this session).
  - **Batched multi-start NLP (GPU optional, CPU first):** With 2-3 seeds, three CPU cores running IPOPT already fit the budget (doc's own number). GPU batched NLP/SQP (jaxipm, GATO, WarpMPC, CusADi-based) pays off only for tens-to-thousands of seeds; those tools are 2025-26 preprints, not IPOPT drop-ins.
  - **Single L1 NLP on GPU (no):** ~1000 variables, banded; the only GPU NLP evidence (MadNLP+cuDSS) shows wins on 10^5-10^6-variable problems; small problems leave the GPU under-utilised (MPPI-Generic's own observation at small batch) and pay kernel launch/sync costs per IPOPT iteration (tens of iterations, each a sequential sparse factorisation). Better levers: MA27/MA57 or a structure-exploiting (Riccati) linear solver, fewer iterations, dedicated core.
  - **L2 QP on GPU (no):** HPIPM's Riccati QP at 50 Hz is the regime where ReLU-QP reports only parity with CPU solvers; GPU adds latency/jitter.
  - **Learned proposal nets (GPU optional):** a classifier/proposal MLP is microseconds on CPU; GPU only matters for diffusion proposals (tens of denoising steps), still ms-scale.
- Conclusion: GPU helps L1 only *in front of* IPOPT (choosing the basin by massive rollouts, or batching many seeds); it does not help the IPOPT solve itself nor L2.

### Gaps
- No benchmark found of MPPI with a full two-track combined-slip tyre model on GPU (rollouts/s); the 10-100x cost factor above is an estimate.
- No JAX/Isaac-Lab MPPI throughput numbers retrieved (MPPI-Generic compared only to TorchRL, Nav2, AutoRally).
- MPCGPU (Adabag et al., GPU PCG-based real-time MPC) was searched but not found in results; numbers not verified.
- jaxipm/WarpMPC/TurboMPC per-solve latencies for ~1000-variable problems not extracted (abstract-level only).
