# Onboard sensing of road surface and tyre-road friction ahead of the vehicle

Context: roadmap 060 M6 (`docs/roadmap/060-live-race-planner.md`). A 20 m patch at mu x0.6 at H3's entry
put the car 5.4 m off the plan at 27 deg, touching the guardrail. In the model loop x0.9 and x0.8 pass,
x0.7 survives 5.3 m off, and x0.6 leaves. The car's own slip-based estimate (sim_vehicle_ecu, at the limit)
saved 0.11 s and 1 m at x0.8, but it turned x0.7 into a crash: its scale went back to the map's while the
car was still sliding. Conclusion recorded in M6: "An unseen low-grip patch met at the limit is not caught
by an estimate made after the slide began." The planner needs each segment's surface class (peak mu,
fall-off past the peak) for 200-260 m ahead. These notes cover what sensing can supply that.

Method note: about 20 searches and fetches. Several primary PDFs could not be read in full (a PMC
browser check, compressed PDFs), so some numbers come from abstracts or search-engine summaries. Those
are marked **[abstract/snippet only]**. Numbers I read in the primary text myself are marked
**[verified in text]**.

## 1. Effect-based estimators (wheel slip, IMU, steering/aligning torque, intelligent tyres): excitation, convergence, accuracy, and why they are reactive

### Takeaway
Effect-based estimators measure the tyre's response. They can see the peak only once the tyre is loaded
near it: slip-based methods need roughly 50 % or more of the grip used, the aligning-torque and
pneumatic-trail methods are the earliest at about 20-50 %, and the in-tyre accelerometer methods need
about 60 %. Once excited they converge fast, in tens of ms to about 0.1 s. They are blind to anything
ahead of the contact patch, so their look-ahead is 0 m by construction. That is the M6 finding, and the
literature says the same.

### Cited Findings
- A review of road-friction virtual sensing (emphasis on low excitation) finds that slip-based methods
  generally need substantial excitation and are reliable above roughly 50 % grip utilisation. Methods
  using slip angle and pneumatic trail need the least excitation **[abstract/snippet only]** —
  [Road Friction Virtual Sensing: A Review of Estimation Techniques with Emphasis on Low Excitation Approaches](https://www.researchgate.net/publication/321344788_Road_Friction_Virtual_Sensing_A_Review_of_Estimation_Techniques_with_Emphasis_on_Low_Excitation_Approaches)
- A summary of newer methods reports reliable estimates at 20-50 % utilisation. It notes that direct
  methods show low accuracy and slow convergence when excitation is low **[snippet only]** —
  [Road Friction Estimation (academia.edu)](https://www.academia.edu/337338/Road_Friction_Estimation)
- Chalmers, "Quantification of Excitation Required for Accurate Friction Estimation": the excitation
  needed varies widely across tyre-road pairs when brush-type models are used. In a simulated sudden
  friction drop during braking, a recursive estimator converged to the new level in about 0.06 s
  **[snippet only]** — [Chalmers research](https://research.chalmers.se/en/publication/241863)
- With designed tyre-force excitation, about 65 % friction utilisation gave an estimate error below
  0.1 in normalised force for all tyre-road combinations tested **[abstract only]** —
  [Design of tyre force excitation for tyre-road friction estimation](https://www.researchgate.net/publication/309617109_Design_of_tyre_force_excitation_for_tyre-road_friction_estimation)
- A real-time slip-based maximum-friction estimator (MHE-style) reports about 0.125 s average
  computation time **[snippet only]** —
  [Real-Time Slip-Based Estimation of Maximum Tire-Road Friction Coefficient](https://www.researchgate.net/publication/3414990_Real-Time_Slip-Based_Estimation_of_Maximum_Tire-Road_Friction_Coefficient).
  The classic adaptive wheel-speed-only estimator, designed to tolerate poor excitation and abrupt
  changes, is in Automatica — [Slip-based tire-road friction estimation (Gustafsson, Automatica 1997)](https://www.sciencedirect.com/science/article/abs/pii/S0005109897000034).
  It predates the 2010 window but is the baseline.
- Steering torque and pneumatic trail: Hsu, Laws and Gerdes (IEEE TCST 2010) estimate slip angle and the
  peak lateral force from the pneumatic trail in the steering torque. The trail falls before lateral
  force saturates, which gives early warning. Hsu's thesis refers to about 80 % of peak lateral force as
  the operating point **[snippet, cut off]** —
  [Hsu, Laws, Gerdes, Estimation of Tire Slip Angle and Friction Limits Using Steering Torque](https://www.hiof.no/iio/itk/personer/und-forsk-ansatte/mabenr/print9.pdf);
  [Hsu 2009 Stanford thesis](https://ddl.stanford.edu/sites/g/files/sbiybj25996/files/media/file/2009_thesis_hsu_estimation_control_of_lateral_tire_forces_0.pdf);
  [The Predictive Nature of Pneumatic Trail](https://www.researchgate.net/publication/251847371_The_Predictive_Nature_of_Pneumatic_Trail_Tire_Slip_Angle_and_Peak_Force_Estimation_using_Steering_Torque)
- Han, Lee, Choi (ACC 2016, KAIST), "Early Detection of Tire-Road Friction Coefficient based on Pneumatic
  Trail Stiffness". The pneumatic trail's high sensitivity to slip angle lets the observer "detect the
  peak tire-road friction coefficient in low slip regions". Estimates are obtained recursively "under the
  persistent excitation condition". It was validated in CarSim and on a proving ground: "early detection
  ... is possible with less excitation signals than the conventional methods" **[verified in text,
  abstract]** — [ACC 2016 PDF](https://acl.kaist.ac.kr/wp-content/uploads/2021/10/2016ACC_HKS.pdf)
- One EPS-based algorithm outputs mu_max only above 0.5 g lateral acceleration, to avoid low-excitation
  problems. This is not Hsu/Gerdes's threshold **[snippet only; source context not confirmed]** —
  [UC Berkeley thesis (eScholarship)](https://escholarship.org/content/qt95g4f274/qt95g4f274.pdf)
- A steering-torque patent for stationary or slow-rolling vehicles claims no long convergence or
  sustained excitation is needed, but it gives surrogate rather than precise friction —
  [US 11897479](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11897479)
- Intelligent tyres: Pirelli's Cyber Tyre is an inner-liner tri-axial accelerometer. Pirelli claims it
  estimates contact forces and the friction coefficient, and reads conditions "from dry to wet to icy",
  including the speed margin before aquaplaning —
  [Pirelli press: Cyber Tyre reads the road](https://press.pirelli.com/cyber-tyretm-reads-the-road-and-warns-of-trouble-ahead/).
  A PoliMi thesis with Pirelli used an EKF on the accelerometer and gives friction "in any working
  condition if tyre use is about 60 % of maximum available" **[snippet only]** —
  [PoliMi thesis](https://www.politesi.polimi.it/handle/10589/109723); [PoliMi 2010 paper](https://re.public.polimi.it/handle/11311/589881)
- A tyre-sensor and lateral-deflection friction estimator (KAIST) is on
  [ResearchGate](https://www.researchgate.net/profile/Sanghyun-Hong-2/publication/263530651_Tyre-Road_Friction_Coefficient_Estimation_Based_on_Tyre_Sensors_and_Lateral_Tyre_Deflection_Modelling_Simulations_and_Experiments/links/558e1b0008ae47a3490bdfb4/Tyre-Road-Friction-Coefficient-Estimation-Based-on-Tyre-Sensors-and-Lateral-Tyre-Deflection-Modelling-Simulations-and-Experiments.pdf)
  (numbers not extracted).
- Racing-specific effect-based estimator: Oeltjen et al. (arXiv 2509.15423, 2025/2026) use IMU, lidar
  and control commands with no tyre model. They detect slip by comparing commanded and measured motion
  and estimate mu from accelerations under no-slip conditions. The car is 1:10 scale. The abstract says
  results "closely match" ground truth, but no error or latency numbers are on the abstract page —
  [Online Slip Detection and Friction Coefficient Estimation for Autonomous Racing](https://arxiv.org/abs/2509.15423)
- Svensson and Törngren characterise slip-based local estimates as "highly accurate but only available
  under slip" —
  [Fusion of Heterogeneous Friction Estimates for Traction Adaptive Motion Planning and Control (arXiv 2105.06692)](https://arxiv.org/abs/2105.06692)

### Inferences
- The failure in M6 is structural, not a tuning problem. An effect-based estimator observes the peak only
  near the peak. At the limit, the first useful sample comes as the car enters the patch, and the 20 m
  patch takes about 0.5-0.7 s at 100-130 km/h. Even a 0.06 s estimator cannot re-plan a corner entry
  whose braking point has already passed.
- The x0.7 crash (the scale went back to the map's while the car still slid) fits a known failure mode:
  the estimator loses excitation, or sees changing excitation, and falls back to its prior. Any
  effect-based estimator used must hold the reduced estimate (an asymmetric, slow-release forgetting
  factor) until the patch is confirmed exited.
- Pneumatic trail / aligning torque is the only effect-based signal that gives warning *before* the
  peak, at about 20-50 % utilisation. A racing car spends most of a hairpin entry above that, so at a
  hairpin it is not truly earlier, but it is useful on the straights and gentle corners that lead into
  one. It needs a steering-torque measurement, or the sim's tyre self-aligning moment.
- Effect-based estimates have one use for look-ahead: to *label the map*. Each pass records mu at
  position s, as in the TUM and FS friction maps (section 4). That helps lap 2 onward, not an unseen
  first patch.

### Gaps
- No full-text numbers were obtained for Hsu/Gerdes accuracy (error in mu), for the Pirelli or
  Continental and Bridgestone systems in production, or for Oeltjen et al.'s error. Continental
  (ContiSense / eTIS) and Bridgestone (CAIS) searches returned nothing usable. Their friction claims are
  unverified here.
- No source gave the latency from patch entry to a converged estimate *at racing speeds* on real cars.

## 2. Cause-based / predictive look-ahead sensing: camera, lidar, radar, NIR optical sensors, thermal

### Takeaway
Cameras are the only onboard sensors with real look-ahead in the published systems, and the published
datasets stop at about 30 m (RoadSaW: 7.5-30 m patches). Class accuracy on benchmarks is about 90-97 %,
but friction is a *class* (high, medium, low) or an inferred value. Where it was checked against
measurements, errors are about ±0.1 mu or worse. Spectroscopic NIR sensors (Vaisala MD30, Lufft MARWIS,
Teconer RCM411) give the best-validated mu (SD 0.05-0.10 against a braking meter), but they look at the
road a metre or so from the vehicle and give essentially zero look-ahead. Lidar intensity is a plausible
wet/snow/ice cue with little published accuracy data. No source supports a 200-260 m friction preview
from onboard sensors.

### Cited Findings
**Camera classification datasets and accuracy**
- RSCD (Road Surface Classification Dataset, China, vehicle-mounted camera): about 370k annotated images
  in the original article, and about 1M 240x360 patches on IEEE DataPort. It has 27 combined classes
  (friction level x material x unevenness). The friction levels are dry, wet, water, ice, fresh snow and
  melted snow. One study reports 94.84 % top-1 and 99.22 % top-2 on five friction classes and derives a
  peak friction from class probabilities with an empirical model **[snippet only]** —
  [A road surface image dataset with detailed annotations for driving assistance applications (Data in Brief 2022)](https://www.sciencedirect.com/science/article/pii/S2352340922006771)
- RoadFormer (arXiv 2506.02358): 92.52 % top-1 on the full RSCD test set and 96.50 % on the 5-class
  simple-RSCD **[snippet only]** — [arXiv 2506.02358](https://arxiv.org/abs/2506.02358).
  Accuracies across papers are not comparable because label sets differ (same source).
- RoadSaW (Cordes et al., CVPRW 2022, VISCODA): bird's-eye-view patches at **7.5, 15, 22.5 and 30 m**
  ahead. It has 12 balanced classes (asphalt, cobblestone, concrete x 4 wetness levels), with wetness
  labelled from a Lufft MARWIS's measured water-film height. The baseline is MobileNetV2 (real time) plus
  an RBF network for confidence. The paper reports limits in generalisation. A companion RoadSC dataset
  covers snow types at 15 m —
  [RoadSaW paper (CVF open access)](https://openaccess.thecvf.com/content/CVPR2022W/WAD/papers/Cordes_RoadSaW_A_Large-Scale_Dataset_for_Camera-Based_Road_Surface_and_Wetness_CVPRW_2022_paper.pdf);
  [VISCODA dataset page](https://www.viscoda.com/downloads/roadsaw-dataset);
  [poster](https://downloads.viscoda.com/research/2022-06-CVPR/2022-06-VISCODA_RoadSaW-poster.pdf)
- Roychowdhury et al. (Volvo Cars / Chalmers, IJCNN 2018, DOI 10.1109/IJCNN.2018.8489188), front camera
  in two stages. Stage 1 classifies the road surface condition at 97 % average accuracy. Stage 2 is a
  rule-based model on a 5x3 grid of patches over the ego lane, grading friction as high, medium or low
  at 89 % accuracy. A related Volvo work adds a BEV transform before the patch classification
  **[repository abstract]** —
  [Machine Learning Models for Road Surface and Friction Estimation using Front-Camera Images](https://research.chalmers.se/en/publication/510219);
  [Friction Estimation using Front-Camera Image Signal Processing](https://research.chalmers.se/en/publication/517943)
- Šabanovič et al. (Sensors 2020, 20(3):612): DNN classification of road type and condition (3 surfaces
  x dry/wet) from video, mapped to a friction coefficient for ABS. SqueezeNet was best of those compared.
  Exact accuracy was not extracted —
  [Identification of Road-Surface Type Using Deep Neural Networks for Friction Coefficient Estimation](https://www.mdpi.com/1424-8220/20/3/612/htm)
- Nolte et al. compared ResNet50 and InceptionV3 on 6 road-surface classes —
  [arXiv 1804.08872](https://arxiv.org/pdf/1804.08872).
  FrictionSegNet (Chalmers) does segmentation with friction estimation —
  [Chalmers record](https://research.chalmers.se/en/publication/543319) (numbers not extracted).
- A non-contact detection review (Sensors 2022) reports 0.845 agreement between predicted and actual
  friction in one study. Vision trained on vehicle response is reported at 0.98 correlation
  **[search summary only; PMC page blocked by a browser check, so the underlying studies are not
  identified]** — [Current Non-Contact Road Surface Condition Detection Schemes and Technical Challenges](https://pmc.ncbi.nlm.nih.gov/articles/PMC9782052/)

**Spectroscopic (NIR) road-weather sensors**
- Teconer RCM411: an optical sensor, near-infrared spectroscopic absorption. Ice is detected from the
  water absorption peak shifting to longer wavelengths at freezing, and mu is modelled from the
  water/ice quantities. Haavasoja (Teconer) with Kiuru, Valtonen and Pellinen (Aalto) report about
  2000 km of winter driving in 2011-2013 and **248 reference points** from a braking friction meter.
  The overall **SD of the difference was 0.10 mu**, 0.05 in snow-event slippery conditions, and 0.14 on
  one test drive. Mu ranged 0.15-0.80 and no clear systematic error was found. The authors say it
  "detects different surface conditions on a short distance". Vendor co-authored **[verified in text]** —
  [PIARC Andorra 2014, paper 13](https://proceedings-andorra2014.piarc.org/content/actes/13_eng.pdf)
- Vaisala MD30 (mobile, for snow ploughs): reports grip 0.09-0.82, states (dry, moist, wet, snowy, icy,
  slushy), water layer 0-5 mm, mounting height 20-110 cm. The datasheet gives no grip *accuracy* —
  [MD30 datasheet](https://c-2156-20160831-www-vaisala-com.i.icims.com/sites/default/files/documents/MD30-Datasheet-B211719EN.pdf);
  [MD30 product spotlight](https://vaisala.com/sites/default/files/documents/WEA-GT-Product-Spotlight-MD30-B211809EN.pdf).
  An earlier study found that a stationary Vaisala DSC111's grip correlated poorly with a friction
  meter, especially at low friction **[snippet only]** —
  [Iowa roadway friction modelling report](https://publications.iowa.gov/43824/2/roadway_friction_modeling_t2.pdf)
- Clear Roads CR 16-03 / MnDOT (2019) compared Lufft MARWIS, Teconer RCM411, High Sierra IceSight and
  Vaisala DSP310 at MnROAD and in live traffic. Sensors were ranked, but "the absolute differences ...
  are often very small" and variability is high, so universal standards are hard to set **[summary
  only; tables not read]** — [ROSA P dot/58609](https://rosap.ntl.bts.gov/view/dot/58609)
- Iowa lab evaluation of MARWIS vs Teconer (friction, film or ice depth, temperature) —
  [Quantifying Salt Concentration on Pavement Phase II](https://rosap.ntl.bts.gov/view/dot/66603).
  A Purdue student evaluation found MARWIS friction agreeing with skid tests and its surface
  temperature reading about 4 °F low —
  [Purdue Road School 2016 poster](https://docs.lib.purdue.edu/roadschool/2016/posters/5)

**Lidar intensity, radar, thermal**
- Lidar: Google/Waymo-family patents flag a wet road when, for example, ≥85 % of road points have
  intensity <10 on a 0-250 scale. They describe snow as low intensity, ice as medium and water as higher,
  and propose camera fusion to resolve the ambiguity. These are patent examples, not validated
  accuracies —
  [US 9207323](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9207323);
  [US 9632210](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9632210).
  A two-waveband lidar application detects water, snow and ice from the ratio of the two returns —
  [Lidar System For Detecting Road Conditions](https://trea.com/information/lidar-system-for-detecting-road-conditions/patentapplication/a6c80405-7a95-46a2-a3e7-a099260498b2).
  A 2025 Frontiers in AI study on Arctic roads claims "a strong correlation between LiDAR intensity and
  the coefficient of friction" with no number in the snippet —
  [Frontiers in AI 2025, 1600174](https://www.frontiersin.org/articles/10.3389/frai.2025.1600174/full)
- Radar and thermal cameras: no primary source with quantitative road-condition or friction accuracy was
  found in this pass (see Gaps).

### Inferences
- Look-ahead budget. The published camera work tops out at 30 m (RoadSaW), and Volvo's grid covers the
  ego lane just ahead. At 130 km/h (36 m/s), 30 m is 0.8 s. That is less than the distance to brake from
  130 to about 60 km/h at 1 g (about 52 m). The 200-260 m window the planner needs is roughly 7-10x
  beyond any published onboard surface classifier. Resolution falls with range (a patch at 200 m
  covers a few pixels), so beyond about 50-80 m a camera can at most see *large*, contrasting regions
  (standing water sheen, snow cover, a dirt/gravel change). It cannot see a slick 20 m asphalt patch.
  This is an inference: no source tested classification at 100-260 m.
- Cameras give a surface *class*, not mu. The mapping from class to mu is a lookup with a wide spread
  (for example, wet asphalt can be 0.4-0.8 depending on texture and film depth). The validated
  instruments' ±0.1 mu (1 SD) is the best case for anything non-contact. For the planner this means:
  plan each class as a *distribution* (mean, ±0.1-0.15), not a point value.
- Peak mu is reachable from a class only through a prior. The fall-off past the peak (the planner's
  "1.0/0.8/0.6" shapes) is not measured by any non-contact sensor found. It would have to come from the
  class's tyre model (snow vs ice vs wet asphalt).

### Gaps
- Radar-based road condition (for example 77 GHz backscatter, polarimetric) and thermal cameras: no
  accuracy or range numbers were found in this pass.
- The underlying studies behind the 0.845 and 0.98 numbers in the Sensors 2022 review could not be
  identified (page blocked).
- MnDOT/Clear Roads accuracy tables (per sensor, per condition) were not extracted.
- No source measures camera surface-class accuracy as a function of distance beyond 30 m.

## 3. Fusion of predictive (camera) and reactive (slip) estimators: published systems and uncertainty handling

### Takeaway
The standard architecture uses the camera class as a *prior* (or as a constraint on the feasible mu
range) and the dynamics-based estimate as the *measurement update*. The fusion is done with a Kalman
filter with forgetting toward the class prior, a constrained cubature KF, a heteroscedastic GP over
space, a Bayesian network, or sequential Monte Carlo. Svensson and Törngren's GP formulation is the
closest match to a planner's needs: a spatial mu field with per-point variance, where camera points are
cheap and noisy and slip points are accurate and sparse.

### Cited Findings
- Svensson and Törngren (KTH, arXiv 2105.06692, 2021): heteroscedastic Gaussian-process regression fuses
  forward-looking camera classifications ("foresight, high availability, but rough") with local
  slip-based estimates ("highly accurate but only available under slip"). The output feeds traction-
  adaptive motion planning and control. The abstract reports only "initial simulation based results"
  with no numbers — [arXiv 2105.06692](https://arxiv.org/abs/2105.06692)
- A Chalmers MSc thesis on "Sensor data fusion for road friction estimation" takes a Bayesian approach.
  Its Kalman filter with a forgetting factor pulls the estimate toward an a-priori distribution tied to
  the currently classified road surface **[snippet only]** —
  [Chalmers thesis PDF](https://publications.lib.chalmers.se/records/fulltext/127242.pdf)
- JME 2024 (Chinese Journal of Mechanical Engineering): DeepLabv3+ road-type segmentation is mapped to a
  friction *range*, which constrains a state-constrained square-root cubature KF on a brush tyre model.
  The authors report fast convergence on an abrupt adhesion change **[abstract only]** —
  [Fusion Estimation Method of Tire-road Friction Coefficient Based on Vehicle Vision and Dynamical Model](https://qikan.cmes.org/jxgcxb/EN/10.3901/JME.2024.24.254)
- An IEEE paper fuses machine vision and vehicle dynamics for road friction **[title only]** —
  [IEEE 9304538](https://ieeexplore.ieee.org/abstract/document/9304538)
- A Bayesian network jointly estimates road condition and friction potential from a camera estimate and
  a dynamics observer ("cause- and effect-based") —
  [Friction and Road Condition Estimation by Combining Cause- and Effect-Based Methods using Bayesian Networks](https://www.researchgate.net/publication/382302407_Friction_and_Road_Condition_Estimation_by_Combining_Cause-_and_Effect-Based_Methods_using_Bayesian_Networks)
- A Vehicle System Dynamics 2026 paper: a hierarchical Bayesian friction model with sequential Monte
  Carlo, where a prior conditioned on road-weather and pavement type is updated by Bayes' rule —
  [VSD 10.1080/00423114.2026.2645983](https://www.tandfonline.com/doi/pdf/10.1080/00423114.2026.2645983)
- A pavement friction network fusing vehicle-dynamics and vision features —
  [ResearchGate 370112824](https://www.researchgate.net/publication/370112824_Pavement_Friction_Evaluation_Based_on_Vehicle_Dynamics_and_Vision_Data_Using_a_Multi-Feature_Fusion_Network)
- A GM-style patent covers road surface detection with recursive adaptive learning and validation (a
  camera classifier retrained from dynamics outcomes) —
  [US 9139204](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9139204)
- Bosch, Volvo and NIRA production fleet friction services: no primary source was retrieved in this pass
  (see Gaps).

### Inferences
- For our planner, the natural form is Svensson-Törngren's: a spatial field mu(s) along the course with
  a variance per segment. The prior is the map or camera class (wide variance). Slip and aligning-torque
  measurements shrink the variance where the car has loaded the tyre. The planner uses a lower quantile
  (for example mean - k sigma) instead of the mean. This puts "uncertainty planned in" (M6's next step)
  into the cost directly.
- A fusion fixes M6's x0.7 failure mode only if the dynamics update cannot *raise* the estimate during a
  slide. The class prior should be the floor while the camera or map says "patch", and the
  slip-estimator's upward updates should be gated on stable, non-saturated driving.

### Gaps
- No quantitative before/after for a fused system (error reduction, look-ahead gained) was retrieved.
  Svensson-Törngren is simulation-only and its numbers were not extracted.
- Production systems (Bosch's road-condition service, Volvo/NIRA Road Surface Information, Continental)
  were not covered with primary sources. Claims about them would be unverified.

## 4. Autonomous racing practice (IAC, Roborace, A2RL, Formula Student driverless)

### Takeaway
Racing teams do not sense friction ahead with perception. They use (a) a global, often lap-wise scaling
of a gg-diagram ramped up conservatively, (b) a spatial friction or "grip" map built offline or from
previous laps, and (c) online stability-based adaptation per track position, refined lap by lap. The
unseen-patch-on-lap-1 problem is handled by margins, not sensing.

### Cited Findings
- TUM (Roborace) project: friction potential "cannot be measured directly ... has to be estimated". The
  goal is real-time estimation *and prediction* over several planning horizons, with high local
  resolution over the whole track for local effects (a drying track, a varying surface) —
  [TUM FTM project page](https://mos.ed.tum.de/en/ftm/main-research/intelligent-vehicle-systems/estimation-and-prediction-of-the-tire-road-friction-potential-of-an-autonomous-racecar);
  Hermansdorfer, Betz, Lienkamp, "A Concept for Estimation and Prediction of the Tire-Road Friction
  Potential for an Autonomous Racecar", IEEE ITSC 2019, pp. 1490-1495, DOI 10.1109/ITSC.2019.8917024 —
  [ResearchGate](https://www.researchgate.net/publication/337625553_A_Concept_for_Estimation_and_Prediction_of_the_Tire-Road_Friction_Potential_for_an_Autonomous_Racecar).
  The concept is that a car on an inhomogeneous surface, over several laps, estimates mu and stores it
  in a map.
- TUM IAC stack (Betz et al., J. Field Robotics 2023): velocity profiles come from gg-diagrams with a
  scale factor adjustable lap by lap, plus an absolute margin. One documented run used **0.8** (lap 1,
  warming brakes and tyres), **0.9** (lap 2), **0.925** (timed lap) and back to 0.8 (cool-down). Average
  lap 66.2 s at Indianapolis 2021 run 2 **[search summary; schedule attributed to a TUM paper; exact
  source page not confirmed]** —
  [TUM Autonomous Motorsport, JFR 2023](https://onlinelibrary.wiley.com/doi/full/10.1002/rob.22153);
  [mediaTUM thesis](https://mediatum.ub.tum.de/doc/1773257/tzjof4mg5oad6nrwk6v1slks4.main.pdf)
- GripMap (Werner, Schwehn, Lienkamp, Betz; arXiv 2504.12115, 2025): spatially resolved dynamic
  constraints in the Frenet frame, stored with perfect hashing for fast lookup, for offline and online
  planning. A search snippet says it cites a 2023 IAC Las Vegas incident in which a car entered an area
  of reduced grip, and that grip is highest on the racing line and lower away from it **[incident not on
  the abstract page; snippet only]** — [arXiv 2504.12115](https://arxiv.org/abs/2504.12115)
- Bergerhoff, Baader, Meißner, Deinzer (TUAS Würzburg-Schweinfurt / Mainfranken Racing; IEEE IV 2026,
  arXiv 2606.10733) **[verified in text]**:
  - They estimate no physical mu. A scaling factor theta is inferred from stability metrics (slip
    angles, path deviation, state errors) by a PID controller at each track position.
  - The theta map is a sparse GP, rebuilt asynchronously after each lap. The previous lap's map is used
    until then.
  - On the real MF17 car, lap 1 took 37.15 s and lap 10 24.32 s (-35 %). The 10-lap mean was 26.90 s
    against 29.30 s for the non-adaptive baseline (-8 %). At FSATA 2025 the 10-lap total was 251.18 s,
    against 408.91 and 425.78 s for two competitors.
  - Its survey of other approaches: fixed gg plus a forward-backward solver (Heilmeier et al. / Roborace;
    StarkStrom Augsburg; Chalmers); a whole-lap factor (Wischnewski et al.: one unstable spot limits the
    whole lap); per-segment factors (KIT, Fischer 2022: abrupt changes can destabilise); offline TRFC maps
    (Herrmann et al., Christ et al., GripMap); prior track knowledge (Kapania et al.); AMZ (Zurich) and
    Lisbon integrate the velocity plan in an MPC.
  — [arXiv 2606.10733](https://arxiv.org/html/2606.10733)
- Christ/Herrmann-style planning with wheel-specific mu from a track friction map, tested on a real
  autonomous race car **[snippet only]** — cited in the search results above. See the GripMap and
  2606.10733 related-work lists.
- Oeltjen et al. (2025/26) is the only racing-specific online slip and mu estimator found (IMU and lidar,
  1:10 scale) — [arXiv 2509.15423](https://arxiv.org/abs/2509.15423)
- A2RL: no primary source on friction handling was found.

### Inferences
- Every real racing practice found is effect-based plus memory: it learns mu(s) by driving and replays
  it on the next lap. None claims camera friction preview. For a *first* pass over an unseen patch,
  teams rely on global margins (0.8-0.925 of the gg) and warm-up laps.
- Our M5 g1 = 0.92 sits right at TUM's timed-lap factor (0.925). TUM's warm-up 0.8 is the margin that
  would have absorbed the x0.8 patch, but not x0.7 or x0.6. This matches the model-loop result (x0.8
  passes, x0.7 does not).
- The FS lesson (KIT): per-segment scaling with abrupt changes destabilises the car. The planner needs
  mu transitions smoothed over a distance comparable to the braking and turn-in length, or planned
  explicitly as a step with uncertainty.

### Gaps
- A2RL (Abu Dhabi) friction handling: no source found.
- IAC Las Vegas 2023 low-grip incident: the details (who, what speed, what grip) are unverified.
- AMZ's and TUM-FS's friction specifics were not retrieved beyond the survey in 2606.10733.

## 5. What is realistic for our sim car (Isaac Sim, RTX camera and lidar; we control the world's surface classes)

### Takeaway
Realistically: (1) a surface-class map of the course (we own the ground truth, so it is a "pre-mapped
friction map" with chosen uncertainty), (2) optionally a simulated camera or lidar classifier that
*confirms or refutes* the map within about 30-60 m, with a calibrated confusion matrix, and (3) the slip
and aligning-torque estimator used only to *update the map after the fact* and to hold a reduced
estimate, never to raise it mid-slide. A perception-only 200-260 m preview has no support in the
literature.

### Cited Findings
- Camera preview distance in published datasets is 7.5-30 m (RoadSaW), and class accuracy is about
  90-97 % on benchmarks (RSCD, Volvo) — see section 2 sources:
  [RoadSaW](https://openaccess.thecvf.com/content/CVPR2022W/WAD/papers/Cordes_RoadSaW_A_Large-Scale_Dataset_for_Camera-Based_Road_Surface_and_Wetness_CVPRW_2022_paper.pdf),
  [Roychowdhury 2018](https://research.chalmers.se/en/publication/510219)
- The validated non-contact mu error is SD 0.05-0.10 (Teconer vs a braking meter) —
  [PIARC 2014](https://proceedings-andorra2014.piarc.org/content/actes/13_eng.pdf)
- Racing practice is map plus lap-wise adaptation plus conservative factors —
  [arXiv 2606.10733](https://arxiv.org/html/2606.10733); [TUM JFR 2023](https://onlinelibrary.wiley.com/doi/full/10.1002/rob.22153)
- Fusion pattern, a spatial GP with heteroscedastic noise —
  [arXiv 2105.06692](https://arxiv.org/abs/2105.06692)

### Inferences (design suggestions, not findings)
- **Model the sensing, not the pixels.** Because swb sets the surface classes, a faithful and cheap
  option is a *virtual perception sensor*. It reads the true class at distance d, flips it according to
  a confusion matrix whose accuracy decays with d (for example about 95 % at ≤30 m, falling towards
  chance by about 100 m on small patches), and attaches class-to-mu priors with ±0.1 SD. This tests the
  planner's uncertainty handling directly, and the published numbers ground it. Training a real
  classifier on RTX renders is possible, but its accuracy would describe Isaac's materials, not the real
  world.
- **Window layering.** Use 0-30 m for perception confirmation, 30-260 m for map prior only, and add
  online effect-based updates written back to the map for later laps. Let L1 use a lower quantile
  mu_q(s) = mu_mean(s) - k*sigma(s), so an unconfirmed patch costs time instead of a crash.
- **Lidar intensity in Isaac** depends on the material's reflectance set-up. If the surface classes get
  distinct NIR reflectances, an intensity-based wet/snow/ice cue is a cheap second channel, but the
  literature gives no accuracy to calibrate it against (patents only).
- **Fall-off past the peak** (the drift decision's key input, per the post-phase result) cannot be
  sensed by any method found. It must come with the class (a per-class tyre curve), and be refined only
  by the effect-based estimator after the car has crossed the peak.
- **Fix the estimator's exit first** (M6 noted this). Use a slow-release, hold-low rule while |slip| or
  sideslip is past the peak, or until the map or perception says the patch has ended.

### Gaps
- No public study evaluates camera road-surface classification in a game-engine or RTX simulator against
  real-world accuracy, so the sim-to-real accuracy of a rendered classifier is unknown.
- Isaac Sim lidar intensity fidelity for wet or icy materials is not covered by any source found.
