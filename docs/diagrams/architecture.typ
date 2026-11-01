#import "style.typ": *
#show: setup

#diagram(
  spacing: (9mm, 10mm),
  edge-stroke: 0.8pt + dim,
  mark-scale: 80%,

  // inputs
  box-node((0, 0), title[Course][centre line, width, surface class per stretch], c-input, s-input, <course>),
  box-node((2, 0), title[Driving style][fastest, drift_budget, drift_max], c-input, s-input, <style>),
  box-node((4, 0), title[Vehicle spec][tyres, motor, brakes, steering], c-input, s-input, <spec>),

  // planner
  tag((0, 0.95), [race_planner], s-plan, <t-rp>),
  box-node((0, 1.7), title[L0: whole track][minimum time, offline], c-plan, s-plan, <l0>),
  box-node((2, 1.7), title[Start schedule][which starts, from the surface ahead], c-plan, s-plan, <sched>),

  tag((0.9, 2.75), [L1: window planner, 200-260 m ahead, re-planned each second, IPOPT], s-plan, <t-l1>),
  box-node((0.5, 3.5), title[G][previous plan], c-l1, s-plan, <g>, w: 24mm),
  box-node((1.5, 3.5), title[H][grip at the peak], c-l1, s-plan, <h>, w: 24mm),
  box-node((2.5, 3.5), title[D][rear-out drift], c-l1, s-plan, <d>, w: 24mm),
  box-node((3.5, 3.5), title[F][flick], c-l1, s-plan, <f>, w: 24mm),
  node((2, 4.75), align(center, title[Gate][fastest feasible plan; drift only if > 0.3 % faster]),
       name: <gate>, shape: fletcher.shapes.hexagon, fill: white, stroke: 0.8pt + s-plan, inset: 7pt, width: 56mm),
  group((<t-l1>, <g>, <f>, <gate>), c-l1, s-plan, gap: 8pt, name: <l1>),

  box-node((2, 6.2), title[L2: tracker, 50 Hz][RTI NMPC on HPIPM: line, speed, sideslip, yaw rate], c-track, s-track, <l2>, w: 56mm),
  box-node((4.3, 6.2), title[State + friction][friction only lowers, holds in a slide], c-track, s-track, <est>, w: 36mm),
  group((<t-rp>, <l0>, <sched>, <l1>, <l2>, <est>), c-plan, s-plan, gap: 10pt, name: <rp>),

  // car
  tag((0.6, 7.25), [the car], s-car, <t-car>),
  box-node((1, 7.9), title[ECU][drive, brakes, ABS / TC / ESC, steering], c-car, s-car, <ecu>, w: 40mm),
  box-node((3.3, 7.9), title[Vehicle][Isaac PhysX, or the test plant], c-car, s-car, <veh>, w: 40mm),
  group((<t-car>, <ecu>, <veh>), c-car, s-car),

  // flows
  edge(<course>, <l0>, "-|>"),
  edge(<course>, <sched>, "-|>"),
  edge(<style>, <sched>, "-|>"),
  edge(<spec>, (4, 1.7), "-|>", text(size: 8pt, fill: dim)[limits for every layer], label-side: left),
  edge(<l0>, (0, 3.5), (<g>), "-|>", text(size: 8pt, fill: dim)[speed ref.], label-pos: 0.3, label-side: right),
  edge(<sched>, <l1>, "-|>", lbl[which starts], label-side: left),
  edge(<g>, <gate>, "-|>"), edge(<h>, <gate>, "-|>"), edge(<d>, <gate>, "-|>"), edge(<f>, <gate>, "-|>"),
  edge(<gate>, <l2>, "-|>", lbl[plan: path, speed, sideslip, yaw rate, inputs], label-side: left),
  edge(<l2>, <ecu>, "-|>", lbl[steering, acceleration], label-side: right),
  edge(<ecu>, <veh>, "-|>"),
  edge(<veh>, <est>, "-|>", lbl[pose, velocity, wheel speeds], label-side: right),
  edge(<est>, <l2>, "-|>"),
  edge(<est>, (4.3, 3.5), <f>, "-|>", lbl[state, friction], label-side: right, label-pos: 0.25),
)
