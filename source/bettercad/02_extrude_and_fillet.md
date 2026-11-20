---
title: Extrude and Fillet
description: Turn a sketch into a solid block with Extrude, then round its top edges with Fillet in BetterCad
layout: lesson
type: page
cover: /learn/bettercad/assets/banners/02_extrude_and_fillet.jpg
date_updated: 2026-11-20
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

You have a flat rectangle. Two tools turn it into something you'd want to hold: **Extrude** gives it height, and **Fillet** rounds its edges.

---

## Extrude it 10 mm

> Press Extrude (E), click inside the rectangle, type 10 and press Enter.

1. Press <kbd>E</kbd>.
2. Click inside your rectangle. It fills with colour and an arrow appears.
3. Type `10`.

![The Extrude panel with a 10 mm preview](/learn/bettercad/assets/extrude_preview.jpg){:class="img-fluid w-100"}

4. Press <kbd>Enter</kbd>.

![The extruded block](/learn/bettercad/assets/extruded.jpg){:class="img-fluid w-100"}

That's a solid: 40 × 20 × 10 mm. The Browser now has a **Bodies** folder with **Body 1** in it, and the timeline has a second step.

You could also drag the arrow instead of typing. And a negative number goes the other way, which becomes useful when you extrude *into* a part to cut a pocket.

### What's in the Extrude panel

| Setting | What it does |
|---|---|
| Distance | How far to pull the shape |
| Taper | Lean the sides in or out as they rise |
| Operation: New body | Make a separate part |
| Operation: Join | Add to a part that's already there |
| Operation: Cut | Remove from a part |
| Operation: Intersect | Keep only where the two overlap |
{:class="table table-single"}

For a first block, the defaults are right.

## Round the top edges

> Press Fillet (G), click the top edges, type a radius like 2 mm and press Enter.

A **fillet** is a rounded edge. Sharp corners on a print are uncomfortable to hold and tend to lift off the bed. Rounded ones look finished.

1. Press <kbd>G</kbd>.
2. Click each of the four top edges. Each turns blue and is added to the list.
3. Type `2`.

![Four top edges picked for a 2 mm fillet](/learn/bettercad/assets/fillet_preview.jpg){:class="img-fluid w-100"}

4. Press <kbd>Enter</kbd>.

![The block with rounded top edges](/learn/bettercad/assets/filleted.jpg){:class="img-fluid w-100"}

Clicked the wrong edge? Click it again to drop it.

## Check your work

Click the block. Properties shows its size and volume, and under **Print** an estimate of its weight. Nothing to measure, nothing to work out.

The timeline now has three steps: the sketch, the extrude and the fillet. That's your model's recipe. BetterCad rebuilds the part from those steps every time you change one, and in lesson 9 you'll use that to change your mind.

## Why not round the bottom edges too?

You can, but think about printing first. A rounded bottom edge curves away from the bed, so the first few layers hang in mid-air. If you want to soften a bottom edge, use a **chamfer**, a flat 45° bevel. That's in lesson 8.

---

## Try it Yourself

1. Press <kbd>Ctrl</kbd>+<kbd>Z</kbd> to undo the fillet, then fillet the four upright edges instead. Which looks better?
2. Try a radius of `20`. What does BetterCad tell you?
3. Make a 10 mm block with 2 mm rounded top edges and keep it. You'll export it in the next lesson.

## Common Issues

**Problem**: Extrude says to click inside a closed sketch shape, but nothing happens when I click.
**Solution**: Click *inside* the shape, not on its line.
**Why**: Extrude works on the area a shape encloses.

**Problem**: "A 20 mm fillet does not fit here."
**Solution**: Use a smaller radius. BetterCad suggests the largest that will fit.
**Why**: A fillet can't be bigger than the faces on each side of the edge.

**Problem**: I can't click an edge at the back.
**Solution**: Drag to turn the view while the Fillet tool is still on, then click it.
**Why**: You can only pick what you can see. Turning the view doesn't cancel the tool.

## Check your understanding

{% quiz %}
? What does Extrude do?
- [ ] Rounds an edge
- [x] Pulls a closed sketch shape into a solid
- [ ] Exports a file
> Extrude gives a flat shape a height.

? What is a fillet?
- [x] A rounded edge
- [ ] A flat bevelled edge
- [ ] A kind of hole
> A fillet is rounded. The flat version is a chamfer.

? Why avoid a fillet on the bottom edge of a print?
- [ ] It uses more plastic
- [ ] Slicers can't read them
- [x] The first layers would overhang
> A rounded bottom edge curves away from the bed, so the lowest layers have little under them.
{% endquiz %}
