# Motorsport practice: choosing strategy and technique from preconditions

Grounding (our context, not web sources): roadmap 061 M3b
(`docs/roadmap/061-grip-or-drift-mode-choice.md`) wants a rule table that picks the planner's
initial guesses (G grip only; + D drift; + F flick) from the window ahead: corner turn and
least radius, surface (mu, fall-off), grade, width. `autoware-drift/docs/drift-optimality.md`
finds that drift pays only in tight hairpins on asphalt (at most 0.4 % at R 9 m, 0.2 % at
15 m, 0.1 % at 25 m) and on low friction (mu 0.3-0.45) in nearly every corner (1-2 %). The
same doc notes that on asphalt the grip and drift optima lie within 0.4 % of each other, so
picking the wrong one costs little there, while on low friction they differ by 1-2 %.

Research budget note: several primary sources (MDPI Heilmeier paper, Southampton/Padova
eprints of Tavernini et al., PMW magazine) returned HTTP 403. For those, findings come from
search-result abstracts only and are flagged.

## F1 / endurance: tyre compound choice from weather and track state

### Takeaway
F1 switches between slick, intermediate and wet tyres using a **lap-time crossover rule**:
change when the lap time on the current tyre passes a threshold relative to the other
tyre's (Pirelli's rule of thumb is 110-112 % of the dry slick lap). The lap time is used
as an observed proxy for track state. Weather radar decides the timing when the state is
changing. The big losses come from **asymmetric risk under a changing precondition**: Sochi
2021 was called "about 50/50", the leader stayed on slicks and fell from 1st to 7th.
Strategy simulators (Monte Carlo) evaluate the user-prescribed options as distributions of
outcomes rather than single numbers.

### Cited Findings
- **Crossover rule (slick to inter):** Take the normal slick lap as 100 %. Switch to
  intermediates when lap times reach "110 % or 112 %" of it. Example: a 1:30 dry lap means
  inters at about 1:40. "A similar percentage increase" over the normal inter lap marks the
  move from inters to full wets (no number given). — [Pirelli: When it's time to change from slick to wet tyres](https://www.pirelli.com/global/en-ww/race/when-it-s-time-to-change-from-slick-to-wet-tyres-in-formula-1)
- **Cost of the wrong tyre:** Running inters when slicks are right costs "five or six
  seconds per lap" (about 5-7 % of a 90 s lap). — [Pirelli](https://www.pirelli.com/global/en-ww/race/when-it-s-time-to-change-from-slick-to-wet-tyres-in-formula-1)
- **The precondition in physical units:** One inter clears about 35-40 L/s of water at
  300 km/h. A full-speed car on inters clears about 150 L/s, and full wets "can be doubled".
  Inters suit a "damp" track and wets a "properly waterlogged" one. Pirelli gives no water
  depth threshold, so the operational trigger is lap time, not depth. — [Pirelli](https://www.pirelli.com/global/en-ww/race/when-it-s-time-to-change-from-slick-to-wet-tyres-in-formula-1)
- **Who decides:** The pit wall supplies "theoretical information on how much water is on
  the track and where". The choice is "down to the driver". — [Pirelli](https://www.pirelli.com/global/en-ww/race/when-it-s-time-to-change-from-slick-to-wet-tyres-in-formula-1)
- **Why a crossover exists:** Inters start overheating and wearing as the track dries,
  while slicks struggle to reach temperature. Both tyres are degraded near the crossover. — [Ferrari Tech Talk, Turkish GP 2020](https://www.ferrari.com/en-CZ/formula1/articles/tech-talk-turkish-grand-prix-2020)
- **The crossover is circuit- and event-specific:** Pirelli estimated the inter-to-slick
  crossover at 1:40 for Suzuka 2022, and noted that new wet and inter tyres that year
  shifted the crossover points. — [Pirelli press, 2022 Japanese GP Friday](https://press.pirelli.com/2022-japanese-grand-prix---friday/). At Silverstone 2016 most teams placed the
  crossover between lap 15 and lap 18 of 52. — [Tyrepress 2016](https://tyrepress.com/2016/07/british-grand-prix-strategies-out-of-the-window-following-rain-pirelli) (via search summary)
- **Live measurement over forecast:** At Suzuka 2022, sector times from a driver's first
  flying lap on the other tyre showed the inter about 4 s faster than the wet. That lap was
  judged the optimum moment to stop. — [F1.com Monday Morning Debrief, Japan 2022](https://www.formula1.com/en/latest/article/monday-morning-debrief-why-nailing-the-strategy-was-so-tough-at-the-rain.6mVFnAE6LiMq5jK9IjN0gy)
- **Pit loss shifts the threshold:** A stop under a Safety Car or VSC costs about 11 s of
  race time, against a full stop under green. Teams wait for such periods, and wet races
  produce more of them. — [F1.com, Japan 2022](https://www.formula1.com/en/latest/article/monday-morning-debrief-why-nailing-the-strategy-was-so-tough-at-the-rain.6mVFnAE6LiMq5jK9IjN0gy) (via search summary)
- **Sochi 2021, the canonical failure (Mercedes side):** Mercedes' inputs were the weather
  radar ("The bit that really made the difference for us was the weather radar", Mike
  Elliott), sector times and driver feedback. McLaren expected the rain to hold steady
  over the last four laps and Mercedes expected it to increase. Hamilton pitted on lap 49.
  Elliott: "we knew if we got that wrong we could fall right back down in the field." —
  [RacingNews365](https://racingnews365.com/mercedes-explain-their-race-winning-russian-gp-strategy-call)
- **Sochi 2021, McLaren side:** Norris stayed on slicks "even as lap times slowed by over
  10s", then the rain increased beyond what slicks could handle. He slid off and finished
  7th after leading. Seidl said "in hindsight it was the wrong decision that we made as a
  team" and that the call was about 50/50. McLaren's review asked why the pit wall did not
  overrule the driver. — [Formula1.com (Seidl)](https://www.formula1.com/en/latest/article/mclaren-didnt-get-it-together-as-a-team-says-seidl-after-norris-misses-out.21pEf2fFvoBjmfNjyMfaew); [The Race](https://www.the-race.com/formula-1/mclaren-needs-to-work-out-why-it-didnt-overrule-norris); [Autosport: mistake not only the forecast](https://www.autosport.com/f1/news/mclarens-mistake-in-russian-gp-not-only-its-weather-forecast/6676911/). Norris: "For whatever reason, we didn't see more rain coming." — [GPFans](https://www.gpfans.com/en/f1-news/70214/mclaren-not-seeing-hard-rain-cost-maiden-f1-win-norris/). McLaren afterwards vowed to improve its "risk-reward judgement" (headline only; article not fetched). — [Motorsport.com](https://www.motorsport.com/f1/news/mclaren-improve-risk-reward-judgement-russia-loss/6683530/)
- **Formal tools: Monte Carlo race simulation (TUM, Heilmeier et al.):** The 2020 *Applied
  Sciences* paper (10(21):7805) "Application of Monte Carlo Methods to Consider
  Probabilistic Effects in a Race Simulation for Circuit Motorsport" evaluates a strategy by
  running thousands of simulations, which gives a distribution of finishing positions. The
  user prescribes the pit stops and compounds, so the simulator evaluates options rather
  than choosing them. — [MDPI](https://mdpi.com/2076-3417/10/21/7805) (403 on fetch; from abstract/search summary); [TUM mediaTUM record](https://mediatum.ub.tum.de/node?id=1551001)
- **Automating the choice ("Virtual Strategy Engineer"):** TUM's VSE uses one neural network
  for whether to pit and a second for which compound. — [TUM portal](https://portal.fis.tum.de/en/publications/virtual-strategy-engineer-using-artificial-neural-networks-for-ma/). Heilmeier's 2022 dissertation, "Simulation of Circuit Races for the Objective Evaluation of Race Strategy Decisions". — [mediaTUM PDF](https://mediatum.ub.tum.de/doc/1647512/1647512.pdf). Open code is at [github.com/TUMFTM/race-simulation](https://github.com/TUMFTM/race-simulation). Flag: that repository was not verified in this session; the search returned a fork, [coderask/f1racesim](https://github.com/coderask/f1racesim).
- **Sequential decision formulations:** An online-planning (MCTS) paper frames pit and
  compound choice as sequential decisions. — [PRL 2021 workshop paper](https://prl-theworkshop.github.io/prl2021/papers/PRL2021_paper_1.pdf). An RL model (RSRL, 2025) is positioned as a faster alternative to the "industry standard of hard-coded and Monte Carlo-based" simulation. — [arXiv 2501.04068](https://ar5iv.labs.arxiv.org/html/2501.04068)

### Inferences
- F1's operational precondition is **measured performance**, not the raw weather variable.
  Water depth is not used as a trigger; lap or sector time against a reference is. For our
  planner, the analogue is to key the rule table on the *predicted gain* (from the
  drift-optimality map) rather than on mu alone. M3b already does this by deriving
  thresholds from the map.
- The crossover is a point where two options are equal, so a rule is ambiguous near it
  (the "50/50" calls). Practice handles this by (a) taking a **live probe**: one car on the
  other tyre gives sector times, and (b) **forecasting the precondition's trend** (radar).
  For M3b, (a) maps to running both seeds when the map's predicted gain is inside the noise
  band. That is what multi-start does: it is cheap insurance in exactly the ambiguous
  region. (b) maps to looking far enough ahead.
- **Asymmetric cost** is the dominant lesson. Being on the wrong tyre as conditions worsen
  is catastrophic (Norris, P1 to P7, spins). Pitting early costs a bounded pit loss. In
  our case the asymmetry runs the same way. On asphalt a missing drift seed costs at most
  0.4 %. On low friction a missing D seed can cost 1-2 % and loses the fallback when G
  fails (18 % of re-plans at mu 0.3, per roadmap 061). So the conservative default should
  be "add D whenever low friction is plausible", which is the opposite of "grip only
  unless sure".
- "Who decides" has an analogue: in F1 the driver (local perception) can override the pit
  wall (global information). In our case the map's surface class is the pit wall and
  on-board friction estimation (`sim_vehicle_ecu`) is the driver. The Sochi failure was a
  missing override channel, which suggests letting a live low-mu estimate force D seeds
  even when the map says asphalt.

### Gaps
- No published team threshold for rain probability or radar lead time (e.g. "pit if
  P(rain > x mm/h within N laps) > p") was found. Team tools are proprietary.
- The quantitative Monte Carlo results in Heilmeier 2020 could not be read (403). No
  numbers on how often the best expected-value strategy differs from the best median one.
- No verified endurance-racing (WEC/Le Mans, Michelin) source on wet-tyre decisions was
  reached within the budget.
- "Rubbering in" (track evolution) was not quantified by any source found in this session.

## Rally: recce, pace notes, conditions crews, tyre choice, when to slide

### Takeaway
Rally formalises the road ahead as a **discrete, ordinal corner code**: direction, a 1-6
(or 1-5) severity, modifiers (tightens, opens, long, don't cut), distance to the next call,
and caution and surface remarks. Crews write it on recce and update it with conditions
reported by "ice/snow note" crews who drive the stage shortly before. Tyre choice is made
**hours before** a loop from forecast and crew reports, and wrong calls are expensive
(about 19 s/km between slicks and studs on ice, per a magazine feature). Technique follows
surface and corner: optimal-control studies reproduce "circuit-style, low-drift" cornering
on pavement and "rally-style, high-drift" cornering on loose surfaces. A handbrake turn
comes out minimum-time for a tight hairpin on loose gravel.

### Cited Findings
- **Severity scales:** The "British Club" system runs 1 (gentlest) to 5 (almost 90 deg).
  The "McRae in Gear" system runs the other way: 6 is almost straight and 1 is a hairpin.
  "Square" is a true 90 deg, "square plus" is sharper than 90 deg, and "hairpin" is about
  180 deg. "D/C!" means don't cut (a hazard on the inside), "+" means it tightens, and "!"
  is a caution. Notes should include "surface type and conditions, potholes", and a pair of
  narrowing lines means the road narrows. Some events ban recce and allow only the
  organiser's notes. — [Wikipedia: Pacenotes](https://en.wikipedia.org/wiki/Pacenotes)
- The numbers originally reflected gear and now mean severity. Some crews reverse the
  scale, and driver and co-driver must agree beforehand. "Right 4 tightens" means it may
  close to a 3 or a 2. — [Race and Rally: pacenotes explained](https://raceandrally.com/blogs/all-blog-posts/rally-pacenotes-explained); [Coach Dave Academy](https://coachdaveacademy.com/tutorials/pace-notes-in-asseto-corsa-rally/). Flag: these are
  secondary or game-oriented sources, so treat the conventions as approximate.
- **Ice/snow note crews (Monte Carlo):** They drive the route before the stages and pass
  information to the co-drivers, who adapt their pace notes. The difficulty is seeing the
  road as the competing driver would and keeping the crew's own language. — Citroën and VW
  Monte Carlo pieces as summarised by search, e.g. [Racecar.com: Citroën C3 WRC returns to Monte Carlo](https://www.racecar.com/news/87810/motorsport/citroen-c3-wrc-returns-to-the-monte-carlo-rally). Flag: paraphrase from search, not fetched.
- **Cost of a wrong tyre on Monte Carlo:** About 19 s/km between slicks and studs on ice
  sections. Toyota's Tom Fowler called one Monte tyre choice among the most difficult he
  could remember. — [PMW magazine: Tire choice conundrums](https://pmw-magazine.com/features/tire-choice-conundrums.html) (403 on fetch; figure from search summary, **unverified**). Drivers in 2025 said they wished they had taken
  one more studded tyre, or lost a lot of time on studs. — [rally-base Monte Carlo 2025](https://rally-base.com/2025/rallye-monte-carlo-2025/information-overview)
- **Gravel compound rules (Pirelli Scorpion KX):** The hard is for abrasive surfaces and
  long stages, the soft for cool or damp conditions. In Portugal the soft is first choice
  in cool and damp weather and the hard in warm and dry. In Estonia the soft was the main
  choice for grip on sandy surfaces. In Croatia teams took soft for second passes even on
  dry roads, because the surface was slippery. In Mexico teams alternated soft and hard
  between morning and afternoon loops, from an allocation of 24 hard and 12 soft. —
  [Pirelli: Portuguese premiere for Scorpion KX](https://press.pirelli.com/portuguese-premiere-for-pirellis-scorpion-kx-wrc-tyres/); [Pirelli: Rally Mexico](https://press.pirelli.com/rally-mexico-hard-and-soft-tyres-share-the-scene-ensuring-performance-and-reliability-on-all-special-stages/); [Off-Road Racer: Croatia](https://offroadracer.com/croatia-rally-dirt-on-the-roads-complicates-drivers-weekend/); [Pirelli: Gravel tyre](https://www.pirelli.com/global/en-ww/race/racingspot/rally/gravel-tyre-playing-fast-and-loose-53220/) (via search summaries)
- **Track state depends on running order:** On dry gravel, cars high in the order "sweep
  a clear line" for those behind. Rain compacts the gravel and helps the front runners,
  while later cars meet mud and standing water. — [FIA: WRC Portugal preview](https://www.fia.com/news/wrc-portugal-primed-seven-event-gravel-stretch-begins) (via search summary). The same corner on the same day therefore has a different
  surface class depending on position and pass.
- **Scandinavian flick:** It uses weight transfer to induce oversteer and carry the car
  through a turn while scrubbing speed. It is useful on loose surfaces (gravel, snow, mud)
  where traction is limited. — [Wikipedia: Scandinavian flick](https://www.Wikipedia.org/wiki/Scandinavian_flick); [Fanatec guide](https://www.fanatec.com/uk/en/explorer/games/gaming-tips/the-scandinavian-flick-a-technique-in-rally-racing/). Modern drivers rely more on left-foot braking and trail braking.
  A rally-school instructor said the flick "really isn't used much anymore" and taught
  trail-braking turn-in instead. — [Jalopnik: Learning the Scandinavian Flick at Rally School](https://jalopnik.com/learning-the-scandinavian-flick-at-rally-school-5758087). The tightest corners and hairpins usually call for the handbrake. — [Fanatec](https://www.fanatec.com/uk/en/explorer/games/gaming-tips/the-scandinavian-flick-a-technique-in-rally-racing/) (consumer guide)
- **Optimal-control evidence, surface dependence:** In Tavernini, Massaro, Velenis,
  Katzourakis and Lot, "Minimum time cornering: the effect of road surface and car
  transmission layout" (*Vehicle System Dynamics*, 2013), the optimal driver shows
  "circuit-style, low-drift" cornering on dry and wet pavement and "aggressive, high-drift,
  counter-steering rally-style" cornering on loose surfaces (dirt, gravel), across RWD,
  FWD and AWD. — [Padova IRIS record](https://research.unipd.it/handle/11577/2835923); [Southampton eprints 384111](https://eprints.soton.ac.uk/384111) (abstract via search; full text 403, **no numbers verified**)
- **Handbrake hairpin:** In Tavernini, Velenis et al., handbrake cornering is the
  minimum-time and minimum-lateral-space strategy on a tight hairpin. It was found optimal
  for an FWD car and validated against an expert driver on loose gravel (CDC 2013; JDSMC
  2014). — [Southampton eprints 382093](https://eprints.soton.ac.uk/382093); [382683](https://eprints.soton.ac.uk/382683) (abstract via search)
- **Trail braking as minimum time:** In Velenis and Tsiotras, trail-braking and the
  pendulum turn are special cases of minimum-time cornering. Trail-braking emerges when
  the car must return to straight-line driving right after the corner. Simple piecewise
  linear driver inputs reproduce near-optimal trail-braking over general corner geometry. —
  [SAE 2008-01-2986 (PDF)](https://dcsl.gatech.edu/papers/sae08.pdf); [JOTA 2008 (IDEAS)](https://ideas.repec.org/a/spr/joptap/v138y2008i2d10.1007_s10957-008-9381-7.html). The ACC 2018 follow-up (You and Tsiotras) splits trail braking into guiding, sliding and
  exiting stages for real-time generation. — [ACC 2018 PDF](https://dcsl.gatech.edu/papers/acc18a.pdf)

### Inferences
- The pace-note code is a ready-made **feature vector for the window ahead**: direction,
  ordinal severity (a radius band plus turn angle), modifiers (tightens, which means least
  radius matters more than average; don't cut, which means a lateral-space limit), surface
  remarks, and narrowing. M3b's inputs (turn, least radius, distance, surface, grade,
  width) match it closely. "Narrows" plus "hairpin" is exactly where the handbrake
  literature finds a slide optimal on lateral-space grounds as well as time. This is a
  further reason to run the width arm before writing any width rule.
- Rally separates **slow-changing preconditions** (decided hours ahead from recce, forecast
  and ice crews: tyres, setup) from **fast ones** (the co-driver's call a few hundred metres
  ahead: technique). M3b decides at the fast scale (each re-plan). Seed choice is cheap and
  reversible, unlike a tyre choice, so it can afford to be generous with seeds.
- The surface on the map can be stale (running order, second pass, rain). Rally answers
  with conditions crews and note updates. For us, the map's class should carry a
  confidence, and low confidence should add seeds rather than remove them.
- The literature agrees with drift-optimality.md qualitatively: pavement favours low drift
  and loose surfaces favour high drift, with tight hairpins as the exception where rotation
  is the binding constraint. No numbers were verified to compare magnitudes.

### Gaps
- No primary FIA/WRC regulation text on recce (number of passes, speed limits) was fetched.
  Wikipedia does not give them.
- "Gravel crews" (a team's pre-stage condition reporters on gravel events) were not
  confirmed by any source in this session. Only Monte Carlo ice/snow crews were.
- No quantitative time loss for road sweeping or running order was found.
- No quantitative data on the slide versus tidy time difference by corner class from real
  rally telemetry.

## Drifting and circuit driving: grip versus slide

### Takeaway
The consensus among coaches and drivers is that **on tarmac the fastest technique holds the
tyres at or just below the peak slip angle**. Sliding beyond it costs grip, heats the tyres
and costs time. The accepted exceptions are corner entry, very tight corners where rotation
pays, and low grip. Competition drifting is judged on style, angle and line, not time, so
it gives no evidence on speed. Verified quantitative evidence is scarce in practitioner
sources. The best numbers come from optimal-control studies, including ours.

### Cited Findings
- Tyres reach maximum side force at a certain slip angle. Beyond it grip falls while the
  tyre slides and overheats, which is the definition of over-driving. — [NASA Speed News: Detecting over-driving](https://nasaspeed.news/columns/driver-instruction/detecting-over-driving/). Too little slip leaves grip
  unused and too much loses it. — [Speed Secrets: slip angle](https://speedsecrets.com/q-what-is-slip-angle-how-do-i-use-it/)
- Practitioner forums argue that intentional oversteer can help on tight corners where
  faster rotation pays, or to work around setup compromises. Sliding on entry can help, but
  a mid-corner slide under throttle usually does not. — [LFS forum](https://lfs.net/forum/post/991196); [f1technical: Drift vs grip](https://www.f1technical.net/forum/viewtopic.php?p=82862). Flag: forum opinion only.
- Drifting wears tyres much faster. — [GTPlanet thread](https://www.gtplanet.net/forum/threads/tyrewear-grip-or-drifting.72882/) (forum)
- Tavernini et al. 2013 (above) is the peer-reviewed counterpart: low drift on pavement and
  high drift on loose surfaces. — [Padova](https://research.unipd.it/handle/11577/2835923)
- Context from our own study, not a web source: drift-optimality.md puts the asphalt
  hairpin gain at 0.1-0.4 % (0.03 s at 9 m), and low friction at 1-2 %. The rotation of the
  hairpin, done 0.4-1.0 s sooner, is what the slide buys. This matches the forum
  "exception for tight corners" and the handbrake-hairpin optimality.

### Inferences
- Practice and theory agree on the structure of M3b's table: grip by default on high-mu
  pavement, slide candidates in tight hairpins and on low mu. There is no practitioner
  evidence for drift seeds on open asphalt corners.
- TUM's drift controller (cited in roadmap 061 risks: 1.1 m lateral error in a held slide)
  and real practice both say a slide is harder to track. A planner gain of 0.1-0.4 % on
  asphalt is probably smaller than the tracking penalty. On asphalt the "+ D" rows should
  therefore require the gain to clear the noise band, not merely be positive.

### Gaps
- No verified measured telemetry (real cars) comparing a drift line and a grip line
  through the same tarmac hairpin was found.
- Formula Drift and D1 judging criteria were not fetched. They are irrelevant to time but
  were requested for completeness.

## Formalisation: decision tables, thresholds, hysteresis, expected value, robustness

### Takeaway
Real practice formalises precondition-to-strategy choice in three layers:
(1) **ordinal codes and lookup rules** (pace-note severity, Pirelli's 110-112 % crossover,
compound by temperature and abrasiveness); (2) **simulation of distributions** (Monte Carlo
race simulation that scores a strategy by its outcome distribution, not a point estimate);
and (3) **learned policies** (TUM's VSE, RL, MCTS). Across all three, the recurring
lessons are: keep the option open near the crossover, weigh the asymmetric cost of the
wrong call, look at the precondition's trend, and let live measurement override the prior.

### Cited Findings
- Threshold rule with a reference: change tyre when the lap time exceeds 110-112 % of the
  reference. — [Pirelli](https://www.pirelli.com/global/en-ww/race/when-it-s-time-to-change-from-slick-to-wet-tyres-in-formula-1)
- Ordinal lookup with modifiers: pace-note severity plus tightens/opens/don't cut. —
  [Wikipedia: Pacenotes](https://en.wikipedia.org/wiki/Pacenotes)
- Rule keyed on conditions: soft for cool or damp, hard for warm, dry or abrasive. —
  [Pirelli Scorpion KX, Portugal](https://press.pirelli.com/portuguese-premiere-for-pirellis-scorpion-kx-wrc-tyres/)
- Distributional evaluation: thousands of Monte Carlo runs, scored by the distribution of
  finishing positions. — [Heilmeier et al. 2020, Applied Sciences](https://mdpi.com/2076-3417/10/21/7805) (abstract)
- Learned chooser: one neural network for pit-or-not and one for compound. — [TUM VSE](https://portal.fis.tum.de/en/publications/virtual-strategy-engineer-using-artificial-neural-networks-for-ma/)
- Sequential decisions under uncertainty: MCTS for pit and compound. — [PRL 2021](https://prl-theworkshop.github.io/prl2021/papers/PRL2021_paper_1.pdf)
- Risk framing in a near-tie: "about 50/50" (Seidl), "if we got that wrong we could fall
  right back down in the field" (Elliott), and McLaren's follow-up on "risk-reward
  judgement". — [Formula1.com](https://www.formula1.com/en/latest/article/mclaren-didnt-get-it-together-as-a-team-says-seidl-after-norris-misses-out.21pEf2fFvoBjmfNjyMfaew); [RacingNews365](https://racingnews365.com/mercedes-explain-their-race-winning-russian-gp-strategy-call)

### Inferences
- **Hysteresis:** None of the sources state hysteresis explicitly, but the practice has
  it. A tyre change costs a pit stop, so teams switch only once the delta clearly exceeds
  the pit loss, which acts as a dead band. In M3b the switching cost is the trail-braking
  cut and the tracking risk of a mode flip. Roadmap 061 already names this: "a gate set too
  low flips modes". The suggested analogue is to *add* a D seed on a looser condition than
  the one at which the planner *commits* to a drift plan. The commit should need gain >
  noise p75 (asphalt) and be kept until gain < 0. Seeds are cheap; commitments are not.
- **Expected-value framing for seeds:**
  E[loss of omitting D] = P(drift-optimal | window) x gain + P(G fails | window) x
  fallback cost. This should be compared with the cost of solving D, which is CPU time
  against a 0.9 s p95 budget. From drift-optimality.md, the first term is at most 0.4 % on
  asphalt hairpins and 1-2 % plus an 18 % fallback rate at mu 0.3. That justifies D always
  on low mu and D only in tight hairpins on asphalt, which is the M3b table.
- **Conservative default when the precondition is uncertain:** The F1 failure mode is
  under-reacting to a worsening precondition. The analogue is a map surface of unknown or
  low confidence, or a possible wet patch. The default there should be the larger seed set.
  Only *confident* high-mu, wide, open corners get G only.
- **Live override:** Pit-wall radar, driver feel, and one car's probe sector times give
  three independent precondition estimates, and the decision uses the most pessimistic
  plausible one. Our analogue is map class, ECU friction estimate, and slip observed on the
  last re-plan.

### Gaps
- No source found that publishes explicit hysteresis bands or dwell times for tyre or
  technique decisions. The hysteresis analogy above is inference.
- No published cost-of-wrong-call statistics across many races (e.g. the share of wet
  races decided by tyre-timing errors) were found.
