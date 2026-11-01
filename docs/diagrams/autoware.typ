#import "style.typ": *
#show: setup

#let topic(t) = text(size: 7.5pt, fill: dim, font: "DejaVu Sans Mono", t)

#diagram(
  spacing: (20mm, 9mm),
  edge-stroke: 0.8pt + dim,
  mark-scale: 80%,

  tag((0, -0.75), [Autoware (kept)], s-input, <t-aw>),
  box-node((0, 0), title[Mission planner][route, goal (ADAPI)], c-aw, s-input, <mission>, w: 40mm),
  box-node((0, 1.3), title[Operation modes, MRM][transition manager, diagnostic graph], c-aw, s-input, <modes>, w: 40mm),
  box-node((0, 2.6), title[Localization][NDT on the map], c-aw, s-input, <ndt>, w: 40mm),
  box-node((0, 3.9), title[vehicle_cmd_gate][its limits opened for racing], c-aw, s-input, <gate>, w: 40mm),
  group((<t-aw>, <mission>, <gate>), c-aw, s-input, gap: 9pt),

  tag((2.4, -0.75), [autoware-drift race stack], s-plan, <t-rs>),
  box-node((2.4, 0), title[Race planner][L0 + L1 + driving style], c-plan, s-plan, <rp>, w: 40mm),
  box-node((2.4, 3.9), title[Race controller][L2 tracker], c-track, s-track, <rc>, w: 40mm),
  box-node((4.3, 2), title[Race estimator][vx, vy, yaw rate, pose; friction], c-track, s-track, <est>, w: 38mm),
  group((<t-rs>, <rp>, <rc>, <est>), c-race, s-plan, gap: 9pt),

  box-node((1.2, 5.5), title[Vehicle interface][Isaac via swb, or a real car], c-car, s-car, <veh>, w: 44mm),

  edge(<mission>, <rp>, "-|>", topic[route]),
  edge(<rp>, <modes>, "-|>", topic[/planning/scenario_planning/trajectory], label-side: right, label-pos: 0.45),
  edge(<rp>, <rc>, "-|>", topic[/race/plan], label-side: left, label-pos: 0.3),
  edge(<rc>, <gate>, "-|>", topic[/control/trajectory_follower/control_cmd], label-side: right),
  edge(<ndt>, <est>, "-|>", topic[pose], label-pos: 0.75),
  edge(<est>, <ndt>, "--|>", topic[initial guess], bend: 14deg, label-pos: 0.55, label-side: right),
  edge(<est>, <rp>, "-|>", topic[/race/state], label-side: right),
  edge(<est>, <rc>, "-|>", topic[/race/state], label-side: left),
  edge(<gate>, <veh>, "-|>", topic[control command], label-side: right),
  edge(<veh>, (4.3, 5.5), <est>, "-|>", topic[status, IMU, wheel speeds], label-side: right, label-pos: 0.3),
)
