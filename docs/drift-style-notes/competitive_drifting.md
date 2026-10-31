# Competitive drifting: how D1 GP, Formula Drift, Drift Masters and the FIA define, judge and execute a drift

Scope: what makes a drift "good" in measurable terms (judging criteria, weights, clips and zones, angle, speed, transitions, deductions), and how drivers execute it (entry techniques, linking, speed and angle control, braking while sideways), with whatever numbers could be sourced. Researched 2026-10-31. Rule years are given for each claim. Primary rulebooks are preferred: the D1 2024 English Appendix-B and the 2016 Japanese Appendix-1, the Drift Masters 2024 Judging Rules and 2026 Rd4 briefing, and the FIA Drifting Guidelines (WMSC 2018) were all read as text extracted from the PDFs. **Formula Drift's current rulebook was not retrieved.** FD material comes from news reports and an older formulad.com article.

## 1. Judging: what each series measures, and the weights

### Takeaway
Every series scores the same three ideas: **line** (pass through marked inner clips and fill the outer zones), **angle** (large, *stable* body slip that does not "return"), and **style** (early, fast initiation, a fast and smooth lock-to-lock transition, and committed throttle and pace). They differ in weights and in how much of the score is measured by machine:
- D1 GP: score = speed × angle + swing, measured by GPS/gyro (DOSS) since 2013.
- Formula Drift: since 2026, about 80% is automated line + angle (UDSM telemetry) and 20% is style from the judges.
- Drift Masters: line 60 / angle 20 / style 20, all by human judges.
- FIA: recommends line 40 / angle 30 / style 20 / speed 10.

Speed is scored explicitly only in D1 (and optionally in FIA events). Elsewhere it enters through "commitment/pace" and through keeping to the accel/decel map.

### Cited Findings

**D1 GP (Japan): 2024 rulebook, Appendix-B "D1 Scoring Criteria" (revised 2024-01-01)**
- Single run, as defined: "a competition of driving skills to drive faster with a larger and more stable drift angle and to switch angles more quickly within a defined curved course." — [D1 2024 Rule Book (English) Appendix-B, pp.28-31](https://entry.d1gp.co.jp/download_files/Data_4.24-01縲占ｦ丞援譖ｸ2024%20闍ｱ隱樒沿縲・English_2024_P28-31_D1謗｡轤ｹ蝓ｺ貅.pdf)
- Three scoring items (same source):
  - **Fast driving speed.** "The driving speed must be as fast as possible in any part of the course. Drifting angle and speed are inversely related, but the speed must be as fast as possible while maintaining a large angle." A "significant decrease in speed during the competition run will result in a significant point deduction." Designated zones and running lines can factor into speed evaluation.
  - **Sharp swing.** "Quickness, precision, and appropriate timing are required when entering the drift (swing out) and when switching sides of the drift angle (swing back)." Points are deducted for slow swing-out or swing-back, for late timing of the swing's start or return, and when "the angle is not determined after the start and return of the swing" (the angle does not settle).
  - **Stable large angle.** Drift angle is defined as "the directional angle of the vehicle relative to the line of travel" (body sideslip). "The larger the angle, the better. However, if the angle returns, points will be deducted as a mistake." "Temporary large angles will not be evaluated."
- **Scoring formula (2024):** speed full score 10.00; stable large angle full score 10.00; sharp swing on a 7-step scale from 0 to +3. "The calculation method is: 1) score x 2) score + 3) score. Example: 9.8 x 9.7 + 0.5 = 95.56." Infraction deductions are subtracted from that total. — [D1 2024 Appendix-B](https://entry.d1gp.co.jp/download_files/Data_4.24-01縲占ｦ丞援譖ｸ2024%20闍ｱ隱樒沿縲・English_2024_P28-31_D1謗｡轤ｹ蝓ｺ貅.pdf)
- 2024 deduction checklist (the "evaluation point" table):
  - Speed: initial drift speed; speed maintained everywhere; no significant slowdown; proximity to a specified passing point; unstable driving line.
  - Angle: big angle returns; overusing the side brake; disturbance from a shift error; significant steering corrections; "drift under" (understeer).
  - Swing: delayed drift, no settled angle, "twice drift" (a double swing), and a "challenging" run, which earns credit.
  — [same source](https://entry.d1gp.co.jp/download_files/Data_4.24-01縲占ｦ丞援譖ｸ2024%20闍ｱ隱樒沿縲・English_2024_P28-31_D1謗｡轤ｹ蝓ｺ貅.pdf)
- When the measuring system is approved, "the measured values will be considered final", and "scoring elements that can be scored by measurement shall be scored only by measurement." — [same source](https://entry.d1gp.co.jp/download_files/Data_4.24-01縲占ｦ丞援譖ｸ2024%20闍ｱ隱樒沿縲・English_2024_P28-31_D1謗｡轤ｹ蝓ｺ貅.pdf)
- **DOSS operation (2016 rules, Japanese):**
  - The scoring section is split into **3 to 5 sectors**, mixing "swing-out/swing-back sectors" with "drift sectors". Each sector is scored, and the sum is the DOSS score.
  - The judge adds a completeness score for throttle/steering/brake operation, understeer and double swing on a 5-step scale (−6, −3, −2, −1, 0), and a surprise/creativity score on a 6-step scale (0, +0.5, +1.0, +1.5, +2.0, +3.0).
  - Off course costs **−3** ("one wheel's tread outside the designated track"); **four wheels off costs −10**.
  - A score of 100 is the reference for a high score, but scores above 100 occur.
  - In tandem, a lead car's DOSS score maps to a rank: ≥100 → 10, ≥99 → 9, ≥97 → 8, ≥95 → 7, ≥90 → 6, ≥80 → 5, ≥70 → 4, ≥50 → 3, ≥30 → 2, ≥10 → 1.
  - The DOSS unit is mounted on the roof at mid-wheelbase on the car's centreline; no antennas or cameras are allowed within 50 cm of its GPS antenna.
  — [D1GP 2016 Regulations Appendix-1 (採点方法)](https://d1gp.co.jp/10_regu/img/2016gprg-1.pdf)
- What DOSS measures: max speed, average speed, average angle, angle change, and sharpness of the car's swing (GPS + angular-rate sensor). Introduced for solo runs in 2013 and for battles from 2019. — [autosport web, 2013-04-18](https://www.as-web.jp/past/d1グランプリ開幕戦舞洲は古口が総合優勝を飾る); [Weblio D1 GRAND PRIX](https://www.weblio.jp/content/D1_GRAND_PRIX); [MONOist 2021](https://monoist.itmedia.co.jp/mn/articles/2110/18/news033_3.html)
- At Maishima 2013, DOSS took the maximum speed in the **swing-back section**, so "keeping speed into the swing-out and swinging back sharply" was the key to a high score. — [autosport web 2013](https://www.as-web.jp/past/d1グランプリ開幕戦舞洲は古口が総合優勝を飾る)
- A larger angle, higher speed, an engine note that never cuts out, and smooth motion all score higher. Quick swing-out and swing-back earn points; a spin or an angle "return" costs heavily. — [autosport web 2018 (Autopolis)](https://www.as-web.jp/domestic/359912/2)

**Formula Drift (USA)**
- 2015-era scheme (formulad.com):
  - Line judge 1-25, Angle judge 1-25, Style judge 1-30. The line and angle judges each add a style score of 1-10.
  - Line = "the ideal path a vehicle must take on course, marked by inner clipping points and outer clipping zones", designed to use the course wall to wall.
  - Angle = overall slip angle over the pass, with "minimal steering adjustments mid-corner" and transitions "as smooth as possible."
  - Style = smoothness and excitement: aggressive flicks, closeness to walls, difficulty, fluidity.
  — [Formula Drift 101, news.formulad.com, 2015-10-06](https://news.formulad.com/?p=21117)
- Older FD rule (forum copy of formulad.com): hitting an *inner* clipping point counts as off course and deducts points; contact outside the marked outer zones may deduct points. — [GTPlanet copy of FD judging page](https://www.gtplanet.net/forum/threads/formula-d-d1-rules-explained-for-gt6-drift-comps-drift-olr.306688/)
- 2019 practice: the style judge looks at initiation, fluidity and commitment. Angle points depend on "the driver's ability to achieve and maintain a high level of angle." — [FordMuscle 2019](https://www.fordmuscle.com/?p=899127)
- 2023 season:
  - Qualifying weights were Line 40%, Angle 30%, Style 20%, X-Factor 10%. Each judge scores out of 100 and the scores are averaged.
  - X-Factor examples: "heavy, controlled throttle use", "big angle with consistent pace", and stability and balance at a very high level.
  - A decel light (green accel / yellow decel / red decel past threshold) serves as a judging aid.
  — [drift.news 2023-03-14](https://drift.news/formulad-judging-regulations/)
- **2026 season:** FD adopted Race Data Labs' **UDSM** (Universal Drift Scoring Method).
  - About **80% of a PRO qualifying score is automated**: drift line and car angle are "quantified and scored entirely" by telemetry. The remaining **20% is Driving Style**, scored by the three judges, who account for difficulty and errors.
  - UDSM also measures proximity, comparative angle and comparative speed in tandem, deceleration (unaffected by smoke), and runs with 1-4 tyres off course.
  - PRO returned to two-run qualifying. UDSM debuted at Long Beach, 2026-04-10/11.
  — [The Shop Magazine 2026-02-24](https://theshopmag.com/news/formula-drift-race-data-labs-2026-udsm-qualifying-format-changes/); [Performance Racing Industry 2026-02-23](https://www.performanceracing.com/magazine/industry-news/02-23-2026/formula-drift-changes-qualifying-format-2026-season)
- UDSM proximity: a driver got "100% proximity" over four runs, which required staying "within 2ft of the opponent's car for the duration of the run." — [search-result summary of 2026 FD coverage; see racer.com 2026-08-24](https://racer.com/2026/08/24/shanahan-wins-formula-drift-pro-round-6-in-seattle) (not fetched directly; HTTP 429)

**Drift Masters European Championship**
- Drift is defined as: "the driver breaks the rear wheels out of a gripping position and counter-steers the vehicle around a course. Generally, the line is designated by markers on the course that are comprised of inner clipping points and outer zones." — [DM 2024 Judging Rules](https://dm.gp/media/pkmbve3r/dm2024-judging-rules.pdf)
- **Qualifying weights (2024 rules, unchanged in 2026 briefings):** Line 60, Angle 20, Style 20. Style breaks down as Initiation 5, Fluidity 10, Commitment 5.
  - 2026 split by judge: Judge 1 scores Line out of 60; Judge 2 scores Line out of 60 plus Commitment 5; Judge 3 scores Angle 20 + Initiation 5 + Fluidity 10. The two line scores are averaged. Half points are allowed.
  — [DM 2024 Judging Rules](https://dm.gp/media/pkmbve3r/dm2024-judging-rules.pdf); [DM 2026 Rd4 Finland briefing](https://dm.gp/media/p43d0rgm/dm26-rd4-finland-qualifying-battle-briefing_www.pdf)
- Target behaviour: "start with a committed, high angle initiation, drift through all of the outside zones and reach all of the inner clips with the maximum degree of angle and pace, while making quick, aggressive transitions with no corrections or mistakes." — [DM 2024](https://dm.gp/media/pkmbve3r/dm2024-judging-rules.pdf)
- Definitions (2024):
  - **Line:** points go to outside zones and inner clips, and possibly to "touch & go" areas, scored by sector.
  - **Angle:** "achieve and maintain a high level of angle with capability of gain or maintain speed outside of deceleration zones." Judges may mark areas where high angle is not required.
  - **Initiation:** early (judged against the initiation cones), rate to angle ("quickly getting to the desired angle"), smooth (no angle reductions or corrections).
  - **Fluidity:** smooth rotation in transition, "lock to lock angle – high degree of angle to high degree of angle", car settled.
  - **Commitment:** consistent throttle, "maintaining pace throughout – using momentum to fill zones & width of course", "make it look dangerous – approach barriers and track edge with confidence."
  — [DM 2024](https://dm.gp/media/pkmbve3r/dm2024-judging-rules.pdf)
- The 2026 Rd4 briefing adds to initiation: "Speed". To fluidity: "Throttle application", "Accuracy". To commitment: "Aggressive on throttle to initiation", "Maintaining proper pace throughout the course as per the Decel map". Initiation is open (handbrake, clutch kick, weight transfer), but "**Weight transfer / Clutch kick scores higher**." — [DM 2026 Rd4](https://dm.gp/media/p43d0rgm/dm26-rd4-finland-qualifying-battle-briefing_www.pdf)
- **Deductions** (2024 and 2026): double initiation, tyre off course, missing zones and clips, "Short Straightening (correction)", off line, lack of angle. **Zero score / incomplete:** spinning out; "opposite drift" (angle of the wrong sign for that point on course); hood or doors opening; stopping drifting; two wheels off (2024), changed to "three wheels off" (2026); an unchaseable run (2026). — [DM 2024](https://dm.gp/media/pkmbve3r/dm2024-judging-rules.pdf); [DM 2026 Rd4](https://dm.gp/media/p43d0rgm/dm26-rd4-finland-qualifying-battle-briefing_www.pdf)
- Qualifying should model a good *lead* run: judges "do not want to see the lead driver driving fast and shallow in order to run away from the chase driver." In other words, a shallow, fast drift is explicitly penalised relative to angle. — [DM 2024](https://dm.gp/media/pkmbve3r/dm2024-judging-rules.pdf)

**FIA (Intercontinental Drifting Cup framework): FIA Drifting Guidelines (WMSC 2018-12-05)**
- Solo run criteria are ANGLE, LINE, STYLE and SPEED. "Speed only to be used when an accurate way of measuring it can be employed." The maximum is 100.
- Recommended weights: **40 Line / 30 Angle / 20 Style / 10 Speed**. Judges may change these, but each criterion must stay between 10 and 50 points and the total at 100.
- Definitions:
  - Line: "Maximum approach to the designated points (clipping zone) in the curves while maintaining smooth drifting posture."
  - Angle: "The size of the angle and stability when the car approaches the curves, and the timing and stability of the consecutive drifting motion."
  - Style: "Commitment (the forward movement of the car) and fluidity (the motion of the vehicle on its own axis)."
  - Speed: "a consistent average speed achieved throughout the run, as opposed to the maximum speed."
- Clipping zone: "reference marks on the edge of the track ... which the drivers get as close as possible to without running them over", placed inside or outside the trajectory.
- Telemetry, if used, is "a tool to inform the judges" and not an independent scoring system.
- Zero points: spin; "blatant understeering"; any wheel outside the track; dangerous technique; crashing into the opponent; being ">10% slower in a battle than in qualification (if telemetry is used)." Reduced marks: driving error; "deliberately skidding 'slowly' in a battle"; missing or running over a clipping zone.
— [FIA Drifting Guidelines](https://www.fia.com/sites/default/files/guideline_for_drifting_event_-_wmsc051218_-_clean_0.pdf)

### Inferences
- Measurable definition common to all four rule sets, for a planner: (a) body sideslip β large and **monotone within each corner**, with no "return" (dip) that judges read as a correction; (b) **speed as high as possible at that β**, with no unscheduled deceleration; (c) **fast β sign reversal** at transitions, with β settling quickly afterwards (D1's "angle determined"); (d) the path's inner edge reaches each inner clip and its outer edge reaches the outer zone (the car's *body* extremities, typically the front bumper at inner clips and the rear bumper/rear wheel at outer zones, judging by the DM 2026 line instructions below); (e) zero off-track wheels.
- D1's product form (speed × angle) makes the angle-speed trade explicit and multiplicative. A 10% loss in either costs the same. A planner objective could mirror it as J = v̂·β̂ + k·swing, with v̂ and β̂ normalised per course.
- "Temporary large angles will not be evaluated" (D1) and "lack of angle" plus "short straightening" (DM) mean the cost should penalise β *variance* and dβ/dt sign changes inside a drift sector, not only reward mean β.
- The DM 2026 Rd4 instruction "Deceleration by increased angle after transition" shows that in red zones, drivers are expected to shed speed by *raising β* (scrubbing sideways), not by straightening.

### Gaps
- Exact current FD rulebook text (2025/2026 point maps, the UDSM angle and line formulas, thresholds) was not retrieved. The UDSM formula is not public in the sources found.
- The DOSS internal formula (how GPS heading, yaw rate and speed become the 10-point speed and angle scores, and how "swing sharpness" is computed from angular velocity) is not published in the rulebooks read. Only the composition (speed × angle + swing) is.
- FIA IDC event-specific supplementary regulations (2024-2026) were not found. Only the generic 2018 guideline was read.
- Smoke is not a scored criterion in any of the rulebooks read. It appears only indirectly (UDSM decel measurement "unaffected by smoke"). No source found that scores smoke.

## 2. Clipping points, zones, course layout and what a solo "line" consists of

### Takeaway
A course is: start line → acceleration straight (sometimes with a chicane in DM tandem) → an **initiation cone sequence** ("3, 2, 1"; the car must be sideways by the single cone) → a judged section of 3-7 corners (FIA), with **inner clipping points/zones (IZ)**, **outer zones (OZ)**, optional "touch & go" areas and an **accel/decel map** → finish line (the car must still be drifting under control) → deceleration area. The "line" is the ordered sequence of IZ and OZ contacts, with specified body parts, plus where to transition and where to slow.

### Cited Findings
- FIA course sections: start line, acceleration section, competition (judgment) section "including minimum 3 curves and maximum 7 curves", finish line, deceleration section, and the judges' position. — [FIA Drifting Guidelines 2018](https://www.fia.com/sites/default/files/guideline_for_drifting_event_-_wmsc051218_-_clean_0.pdf)
- DM initiation: "the vehicle must be sideways as it passes the single standing cone in the '3, 2, 1' cone sequence placed trackside." An optional start chicane makes the lead lift briefly so the chase can keep proximity. — [DM 2024](https://dm.gp/media/pkmbve3r/dm2024-judging-rules.pdf)
- A worked DM line (2026 Rd4, Ahvenisto), quoted in full: "Initiate on the Left side of the track; Get left rear wheel to the track Edge before OZ1; Front right wheel into IZ2 and continue to OZ3; Front right wheel into IZ4 and continue to OZ5; Decelerate after transition and fill OZ6; Front left wheel into IZ7; Decelerate after transition and fill OZ8; Keeping smooth arc till the finish line." The angle instructions for the same course are: "Consistent angle from OZ1, through IZ2, OZ3, IZ4 to OZ5; Smooth transition from OZ5 to OZ6; Deceleration by increased angle after transition is finished before OZ6 ... In both deceleration zones speed adjustment possible only after transition is finished!" — [DM 2026 Rd4](https://dm.gp/media/p43d0rgm/dm26-rd4-finland-qualifying-battle-briefing_www.pdf)
- DM **accel/decel map**:
  - Green: "must be accelerating or at very least maintaining speed".
  - Red: "may decelerate ... using handbrake, footbrake or excessive off throttle".
  - Orange: "partial throttle or minor speed adjustment ... may not be heavily using the handbrake or footbrake".
  Lead runs that do not follow it receive deductions. — [DM 2024](https://dm.gp/media/pkmbve3r/dm2024-judging-rules.pdf)
- FD: line marked by "inner clipping points and outer clipping zones", designed to use the course "wall to wall." — [Formula Drift 101 (2015)](https://news.formulad.com/?p=21117)
- D1: designated zones and running lines factor into speed and angle evaluation, "since angle determination by appearance may require running on the same line." The judges explain the scoring points on the course map at the drivers' briefing before qualifying. — [D1 2024 Appendix-B](https://entry.d1gp.co.jp/download_files/Data_4.24-01縲占ｦ丞援譖ｸ2024%20闍ｱ隱樒沿縲・English_2024_P28-31_D1謗｡轤ｹ蝓ｺ貅.pdf)
- D1 off course = one wheel's tread outside the designated track. — [D1 2016 Appendix-1](https://d1gp.co.jp/10_regu/img/2016gprg-1.pdf)
- FD's 2026 UDSM detects runs with 1-4 tyres off the prescribed course. — [The Shop Magazine 2026](https://theshopmag.com/news/formula-drift-race-data-labs-2026-udsm-qualifying-format-changes/)

### Inferences
- For a planner, a "course" can be encoded as an ordered list of constraints:
  - initiation gate: β ≥ β_min by gate s₀;
  - IZ: the distance from a body point (front inner corner) to a marker is ≤ ε_in, never negative, since touching or hitting an inner clip is "off course" in old FD rules;
  - OZ: the distance from a rear outer body point to the outer edge is ≤ ε_out over a zone length;
  - transition intervals: β changes sign within [s_a, s_b];
  - speed-profile constraints: green = dv/ds ≥ 0; red = deceleration allowed, preferably via increased β;
  - finish: still in a controlled drift.
- Outer zones are filled with the *rear* of the car (the tail swings to the wall). Inner clips are hit with the *front*. With β of 30-50°, the outer rear corner sweeps a much wider envelope than the CG path, so the CG line sits well inside the geometric outer edge.

### Gaps
- No source gave typical clip and zone dimensions (zone lengths, distance tolerances in metres). Judges set these per event on a map.

## 3. Angles, speeds, yaw rates: quantitative data

### Takeaway
Hard numbers are scarce in public rulebooks. Sourced values:
- Pro competition entry speeds: roughly 110-130 km/h in D1 (Maishima 2013), about 80-85 mph (130-137 km/h) at FD Irwindale, and over 100 mph (>160 km/h) at the old FD Long Beach layout.
- Instrumented expert drifts: β about 30-45° in steady state, ±40° figure-eight transitions in about 1 s at 50 km/h with yaw rate up to 120°/s.
- Drift-car steering lock: 55-70°, against 30-40° stock.
- Competition sideslip in degrees: no published figure found.

### Cited Findings
- D1 Maishima 2013 (single-run final): sector-2 peak speeds reached the "high 120s km/h" for several drivers. Winning DOSS score 100.86; top qualifier 99.02. — [autosport web 2013-04-18](https://www.as-web.jp/past/d1グランプリ開幕戦舞洲は古口が総合優勝を飾る)
- A D1 single-run report quoted entries of 114 km/h ("fastest entry of the day") and 111-112 km/h for a 98.4-point winner. — *Seen only as a search-result summary attributed to [as-web.jp](https://www.as-web.jp/past/d1グランプリ開幕戦舞洲は古口が総合優勝を飾る). The fetched article did not show these figures, so treat as unverified.*
- FD Irwindale: drivers reached "around 80 to 85 mph before flicking the car sideways" into the turn 3-4 complex; in 2018 the lead entered turn 3 "at speeds nearing 80 mph." — [search summary of DrivingLine Irwindale coverage](https://www.drivingline.com/articles/vaughn-gittin-jr-knocks-out-the-competition-at-formula-drift-irwindale-2018/) (not fetched)
- FD Long Beach (old Dockyards course): high-speed entry with few visual references; initiation between cones 2 and 1; fastest recorded entry above 100 mph. — [Speedhunters track tour](https://www.speedhunters.com/?p=1043) (search snippet; page fetch failed)
- Expert rally driver, instrumented RWD Escort (110 bhp, dirt, VBox + IMU):
  - On a 13 m radius: steady state at **V ≈ 8.1 m/s, yaw rate ≈ 36°/s, β ≈ 32°**, throttle near maximum, counter-steer. Peaks in β came under high throttle and counter-steer. Inner rear wheel spinning faster (LSD-limited).
  - On a 2 m radius: entered with the **handbrake** (rear lock) at t≈3 s, then back on full throttle. Steady state at **V ≈ 3 m/s, yaw rate 1.5 rad/s, β ≈ 45°**, with steering *into* the corner (no counter-steer). The radius achieved was below the car's 4.5 m kinematic minimum.
  - "Expert drivers use the throttle to induce high vehicle yaw rate and sideslip," i.e. longitudinal inputs control lateral dynamics.
  — [Velenis et al., Control Eng. Practice 19 (2011) 1363-1376](https://dcsl.gatech.edu/papers/cep10.pdf)
- Stanford MARTY (autonomous, RWD DeLorean): path and sideslip tracking "at values of sideslip as high as 45 degrees". A figure-eight at **50 km/h transitions through ±40° of sideslip in about a second**, steering lock-to-lock in about 1 s, with yaw rate "as high as 120 deg/s." — [Stanford DDL](https://ddl.stanford.edu/publications/journal/towards-automated-vehicle-control-beyond-stability-limits-drifting-along); [Stanford News 2019-12](https://news.stanford.edu/stories/2019/12/autonomous-delorean-drives-sideways-move-forward) (via search summary)
- Analysis of pro driver Ken Gushi's drifts in MARTY (AVEC 2024):
  - Yaw-rate sign stays constant while steering angle changes sign (counter-steer).
  - The driver holds lateral velocity and yaw rate nearly constant through the drift.
  - Yaw rate during the controlled drift is comparable to the yaw rate at the start of the transient.
  — [Righetti et al., AVEC 2024, Springer 10.1007/978-3-031-70392-8_4](https://link.springer.com/chapter/10.1007/978-3-031-70392-8_4) (search summary; PDF fetch 403)
- Steering lock: stock systems are typically limited to 30-40°; drift angle kits extend this to 60-70°+. Examples: Wisefab E36 narrow kit up to 55°, Wisefab S550 kit 65°, IRP E36 up to 65-70°. These are vendor claims. — [SLR Speed angle-kit guide](https://slrspeed.com/blogs/news/drift-angle-kit-complete-guide); [Wisefab S550](https://usa.wisefab.com/wf550-ford-mustang-s550-front-drift-angle-lock-kit); [Driftworks Wisefab E36](https://www.driftworks.com/wisefab-bmw-e36-front-narrow-drift-angle-lock-kit.html)
- FIA guideline vehicle table lists a 60 km/h speed maximum for entry-level/grassroots categories; pro categories are subject to track homologation. — [FIA Drifting Guidelines](https://www.fia.com/sites/default/files/guideline_for_drifting_event_-_wmsc051218_-_clean_0.pdf)

### Inferences
- Plausible planner targets for a "competition-looking" drift, derived from the above rather than stated by any series:
  - β plateau of 30-45° (higher at low speed and small radius);
  - transitions as a β sign change of about ±35-40° in about 1 s, implying peak yaw rates around 100-120°/s at about 50 km/h;
  - steering usage close to full lock (55-70° road-wheel) at the transition peaks.
- Entry speeds scale with the venue. On a 1:1 full-size car, pro entries are around 110-160 km/h. A scaled-down course (e.g. 50 km/h) should be judged by the same *shape*: high β, no dips, fast swing.

### Gaps
- No published sideslip-angle numbers (degrees) from D1 DOSS, FD UDSM or DM judging. No published D1/FD/DM run times or comparisons against grip laps.
- No measured yaw-rate or angular-velocity thresholds for D1's "sharp swing" score.

## 4. Technique: entries, linking, speed and angle control, braking while sideways

### Takeaway
- **Initiation:** feint / weight transfer ("flick"), clutch kick, handbrake, braking (load the front, then turn), and power-over. Pros pick by speed and corner. Fast entries use feint or weight transfer, with a clutch kick for extra yaw. Tight or slow corners use the handbrake. Power-over suits long sweepers in high-power cars. DM 2026 explicitly scores weight-transfer and clutch-kick initiations above handbrake ones.
- **Holding a drift:** throttle sets β (more throttle → more angle; lift → the car straightens). Steering is counter-steer used mainly to keep the front pointed along the intended path. Revs stay high.
- **Linking:** over-rotate slightly, then lift/unload so the rear regrips and the car swings to the other side. Let the wheel spin through to the opposite lock, then get back on throttle firmly.
- **Speed scrubbing in red zones:** add angle (scrub sideways) and/or use handbrake or footbrake, ideally only after the transition is complete.

### Cited Findings
- DM 2026: "INITIATION IS OPEN: Handbrake, Clutch kick, Weight transfer (Weight transfer / Clutch kick scores higher)". Commitment includes "Aggressive on throttle to initiation". In tandem: "Cars should accelerate all the way to the initiation point; Penalty's will be applied to any lead driver that slows or lifts before initiation", including "handbraking in the incorrect direction to the corner". The lead "may use a 'flick'" and may use the full track width, "but can't lose momentum thanks to that (recommended to use only 2/3 of the track)." — [DM 2026 Rd4](https://dm.gp/media/p43d0rgm/dm26-rd4-finland-qualifying-battle-briefing_www.pdf)
- D1 2024 deducts "Using side brake too much" under angle. — [D1 2024 Appendix-B](https://entry.d1gp.co.jp/download_files/Data_4.24-01縲占ｦ丞援譖ｸ2024%20闍ｱ隱樒沿縲・English_2024_P28-31_D1謗｡轤ｹ蝓ｺ貅.pdf)
- Technique overview (practitioner and vendor sources, not rulebooks):
  - Clutch kick: rev and dump the clutch to spike rear torque; best in lower gears at moderate speed.
  - Handbrake: locks the rears; used for lower speeds and tight corners, but overuse costs momentum.
  - Power-over: floor it mid-corner; needs torque; suited to long sweepers.
  - Feint / Scandinavian flick: brief steer away to load the outside, then into the corner; used to keep speed in fast corners.
  - Techniques are often combined, e.g. a handbrake or clutch kick plus power-over.
  — [SLR Speed: techniques from clutch kick to flick](https://slrspeed.com/blogs/news/drifting-techniques-explained-clutch-kick-to-flick); [TorqueCars drifting techniques](https://www.torquecars.co.uk/articles/drifting-techniques)
- Driver61:
  - Initiation: brake before the corner to move weight onto the front and unload the rear, flick the steering, apply opposite lock. Too soft understeers; too rough spins.
  - Maintenance: lift → less angle, more throttle → more angle; keep revs high. Too much throttle uses up steering lock and spins.
  - Transition: "slightly over-rotate just before changing direction, then lift off," let the wheel spin through the hands, then get back on throttle "quickly and firmly."
  - Exit: a high-power car eases throttle slightly to bring the rear in line; a low-power car (GT86) lets the drift run out as the steering opens.
  — [Driver61 "How to Drift"](https://driver61.com/uni/how-to-drift/)
- Matt Farah, coached by FD driver Corey Hosford (2016):
  - Entry is the most violent phase; sideways momentum scrubs speed.
  - Balance: keep the front wheels pointed where the car should go, medium throttle, counter-steer. More throttle gives more angle, less lets it straighten.
  - Exit: don't lift. Lifting moves weight forward, the front grips and the rear snaps the other way.
  - Keep revs near but below the top of the powerband to leave room to add power.
  — [The Drive 2016](https://www.thedrive.com/accelerator/4331/an-obsessives-guide-to-drifting-a-car-like-a-pro.md)
- Keiichi Tsuchiya (video summary): brake first, then turn in toward the clip with heel-and-toe. Too much throttle before the clipping point causes understeer, so throttle comes at or after the clip. — [search summary of a Tsuchiya instructional video](https://www.dailymotion.com/video/x9tlfi2) (not verified)
- Instrumented expert: throttle near maximum during steady drift. Handbrake used to *initiate* a tight-radius drift, then released for full throttle. Counter-steer on the 13 m radius, but steering *into* the corner on the 2 m radius. — [Velenis et al. 2011](https://dcsl.gatech.edu/papers/cep10.pdf)
- Mid-drift handbrake use is debated among practitioners: some call it a correction that breaks fluidity, others a useful tool. — [GTPlanet thread](https://www.gtplanet.net/forum/threads/two-drift-questions.296577) (forum, low authority)
- FIA battle rule: if the follower "reduces his steering angle, in order to build up speed" and stays close or overtakes, the follower loses points. Speed bought by reducing angle is penalised. — [FIA Drifting Guidelines](https://www.fia.com/sites/default/files/guideline_for_drifting_event_-_wmsc051218_-_clean_0.pdf)

### Inferences
- For a planner/controller split:
  - Throttle (rear slip ratio) is the primary β actuator.
  - Steering is the path/heading actuator (front axle points along the desired velocity direction).
  - The handbrake is an impulsive yaw initiator, best used only at initiation or in red zones.
  - A transition is a scheduled yaw-moment reversal: brief over-rotation → throttle lift (rear regains lateral grip) → steering swings through to the opposite lock → throttle reapplied hard. This matches DM's "lock to lock" and D1's "sharp swing back".
- Rules penalise anything that *looks* like a correction: β dips, double initiations, steering corrections, handbrake overuse. A planner's reference should therefore be smooth in β(s) within a sector, with the β sign change concentrated in short intervals.

### Gaps
- No authoritative (rulebook or pro-interview) source was found on "manji" (straight-line alternating drift) or on footbrake use while sideways (left-foot braking to extend or tighten the line). These are common in practice, but no citable description was found.
- No sourced data on gear choice, tyre choice or pressure, or rear-grip setup from pro teams. The setup items found are vendor steering-lock claims only.

## 5. Tandem, only as far as it changes the solo line

### Takeaway
The lead is expected to run the qualifying line at qualifying speed and follow the accel/decel map, in a "chaseable" way (no running away fast and shallow, no "parking" the chaser at inner clips). The chase must initiate no later than the lead, match or exceed the lead's angle, and stay close. For a solo planner, tandem rules mostly confirm the solo target: same line, same angle, predictable speed profile.

### Cited Findings
- DM 2024:
  - Tandem points split 60% lead and 40% chase. In 2026 the split is 10 points per judge for the lead and 9 for the chase.
  - A chaseable lead "filled the zones, hit the clips and maintained a consistent speed and/or slowed/braked in the proper areas."
  - Chase goals: initiate no later than the lead; keep proximity; "match or better the lead drivers angle"; mimic transitions and line.
  - Proximity box: the chaser must be at least level with the lead's front bumper inside the box, or the run is incomplete.
  — [DM 2024](https://dm.gp/media/pkmbve3r/dm2024-judging-rules.pdf); [DM 2026 Rd4](https://dm.gp/media/p43d0rgm/dm26-rd4-finland-qualifying-battle-briefing_www.pdf)
- D1 2024:
  - The lead is judged as in a solo run, with "priority given to line and speed."
  - The chase's speed is judged relative to the lead: if the gap shrinks "while the angle is the same or greater, the speed is judged to be higher."
  - Opening the gap at equal or smaller angle costs points. Closing at equal or greater angle earns them.
  — [D1 2024 Appendix-B](https://entry.d1gp.co.jp/download_files/Data_4.24-01縲占ｦ丞援譖ｸ2024%20闍ｱ隱樒沿縲・English_2024_P28-31_D1謗｡轤ｹ蝓ｺ貅.pdf)
- FIA: the leader's "speed must be identical to the qualification speed"; tactics to slow the follower are prohibited; being >10% slower than in qualifying scores zero (with telemetry). — [FIA Drifting Guidelines](https://www.fia.com/sites/default/files/guideline_for_drifting_event_-_wmsc051218_-_clean_0.pdf)

### Inferences
- Tandem rules imply that a good solo line is also *repeatable and predictable*: speed held in green zones, decelerations only in marked red zones, and transitions at fixed course positions. These are useful constraints for a planner, even solo.

### Gaps
- Precise proximity metrics beyond "within 2 ft" (UDSM 100% proximity) and DM's front-bumper proximity box were not found.
