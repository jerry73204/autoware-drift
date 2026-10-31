# 004: the friction estimate lowers in a planned slide

**Severity:** medium (one M4b chain run of eight with the estimate on left the road at the mud
hairpin, s 265, where the same map without the estimate went on to the ice zone). **Status:**
fixed (roadmap 061 M4c).

## What is wrong

Roadmap 061 M4b, the zone chain, 10 % wrong map seed 2, sched + perception + estimate
(`zones__sched_e10s2_perc_est_r1_fin`): the estimate fell to 0.394 on the mud hairpin (zone 1,
mapped right: mud is not a wrong zone on this seed), the re-plans at 0.39 of mud's friction left
the road at s 265. The design (race-planner spec 4.4a) says the estimate only lowers on a
reading at the limit and holds through a slide; it held, but the reading it took was taken in
the slide.

## Root cause

The car's estimate comes from sim_vehicle_ecu's estimator: at the limit it reads the
acceleration the car reaches, |a|/g + 0.05. Its test for the limit is true through the whole of
a drift (the sideslip past half the ESC's limit). `Driver.mu_scale` believes a reading when the
car slides 2.5 deg more than the plan and the plan asks at least 0.7 of the road's grip there.

In a planned slide the reached acceleration is not the road's friction: the tyres are
saturated, but the front is steered against the slide and the axles' forces point different
ways, much of them making the yaw moment that rotates the car, not accelerating it. Re-run
of the same configuration (M4c, trace of every tick, load 5.3): through the hairpin's reversal
(s 234-241, the plan at 11-18 deg) the estimator reads 0.10-0.17 against the plan's 0.24 g
demand and mud's 0.29: 36 ticks where, had the car been 2.5 deg wider than the plan at that
moment, the scale would have read 0.35-0.59. The M4b run's 0.394 is 0.114 / 0.29, a reading
inside that band. This re-run did not slide wider than the plan during the dip (it took no
reading and went on to s 832): whether the lowering happens is the coincidence of the tracking
error and the reversal.

## Fix

`Driver.mu_scale` takes no reading where the plan itself slides (its sideslip at least
`SLIDE_BETA`, 5 deg); the estimate still holds there (the car slides). An unplanned slide off a
grip plan, which is what a zone mapped grippier than it is produces, is still read.
Test: `test_live.py::test_no_reading_in_a_planned_slide`.

Cost: a zone mapped wrong where the plan drifts is not read until the car leaves the slide
(perception, the next grip corner, or the pass's map correction catch it).
