# 014: the planner does not know what is past the road's edge

**Severity:** medium (no failure measured; a design gap. A plan near a drop is as bold as one
near a grass run-off, and S2 and S3 may buy a drift next to a cliff). **Status:** open.

## What is wrong

The race planner treats both road edges alike everywhere. L1 keeps the road's width less the
car's half width and a fixed margin, with a soft penalty of 10 s per metre past it
(`window.EDGE_PENALTY`); L2 has the same edge rows. Nothing in the course says what lies
beyond an edge: a drop, a guardrail or wall, or a run-off the car can come back from. So:

- the margin is the same on the cliff side and the grass side;
- the driving styles price and buy a drift by its lap time only, never by what leaving the
  road there would cost;
- the test plant ends a run `off_road` on either side, so scores cannot tell a crash from a
  recoverable excursion. In Isaac the terrain and swb's guardrails are physical, but the
  planner still does not know them in advance.

What the models do include: the road's grade (gravity along the road in L1, L2 and the plant,
`track.grade`). Not included: banking (cross slope) and the change in tyre load on crests and
dips.

## What a fix needs

1. **Course data**: a hazard class per side and stretch (`drop`, `barrier`, `runoff`), from
   swb's course builder, which already places the guardrails (`course/usd.py`).
2. **Planner**: a margin per class (wider, or a hard row, beside a drop); no planned slide
   (S2, S3) in a corner beside a drop, or a smaller grip share there.
3. **Plant and scoring**: leaving onto a drop is a crash, onto a run-off a recoverable stop,
   so experiments count them apart.

Banking and load on crests belong with this when the course carries a 3D road surface.
