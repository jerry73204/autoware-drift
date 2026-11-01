// Shared look for the README's diagrams. Render: typst compile --format svg <file>.typ
#import "@preview/fletcher:0.5.8" as fletcher: diagram, node, edge

#let ink = rgb("#1f2933")
#let dim = rgb("#52606d")
#let c-input = rgb("#e8eef7")
#let c-plan = rgb("#e6f4ea")
#let c-l1 = rgb("#d3ecdc")
#let c-track = rgb("#fdf1dc")
#let c-car = rgb("#f1e8f7")
#let c-aw = rgb("#e8eef7")
#let c-race = rgb("#e6f4ea")
#let s-input = rgb("#7b93b8")
#let s-plan = rgb("#5f9e74")
#let s-track = rgb("#c89b4a")
#let s-car = rgb("#9a7bb0")

#let setup(body) = {
  set page(width: auto, height: auto, margin: 14pt, fill: white)
  set text(font: "Noto Sans", size: 10pt, fill: ink)
  body
}

#let title(t, sub) = [#text(weight: "bold", t) \ #text(size: 8.5pt, fill: dim, sub)]
#let box-node(pos, body, fill, stroke, name, w: 34mm) = node(
  pos, align(center, body), name: name, fill: fill, stroke: 0.8pt + stroke,
  corner-radius: 4pt, inset: 7pt, width: w)
#let group(nodes, fill, stroke, gap: 10pt, name: none) = node(
  enclose: nodes, fill: fill.transparentize(55%), stroke: (paint: stroke, thickness: 0.6pt, dash: "dashed"),
  corner-radius: 8pt, inset: gap, snap: false, name: name)
#let tag(pos, t, stroke, name) = node(pos, text(size: 8.5pt, weight: "bold", fill: stroke.darken(30%), t),
  name: name, stroke: none, fill: none, inset: 2pt)
#let lbl(t) = text(size: 8pt, fill: dim, t)
