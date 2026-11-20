---
title: Push, Pull and Shapes
description: Build a part from ready-made BetterCad shapes, size it by typing, and reshape it by pulling on a face
layout: lesson
type: page
cover: /learn/bettercad/assets/banners/06_push_pull_and_shapes.jpg
date_updated: 2026-11-20
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Sketching is precise, but it isn't the only way to start. BetterCad also has a **Library** of ready-made shapes you can drop in and resize, Tinkercad style. Over the next three lessons you'll use them to build a small enclosure for an electronics project, with screw posts and a cable hole.

---

## Add a box

Start a new model (<kbd>Ctrl</kbd>+<kbd>N</kbd>), then click **Library** on the toolbar.

![The Library of shapes](/learn/bettercad/assets/library.jpg){:class="img-fluid w-100"}

There are six shapes: Box, Cylinder, Sphere, Cone, Torus and Wedge. Click **Box**.

![A box added from the Library](/learn/bettercad/assets/box_added.jpg){:class="img-fluid w-100"}

A box appears on the ground at the centre of the model, already selected, with its details in Properties.

## Size it by typing

Under **Size** in Properties there are three boxes: **W**, **D** and **H**, for width, depth and height.

1. Click **W**, type `60` and press <kbd>Enter</kbd>.
2. Set **D** to `40`.
3. Set **H** to `25`.
4. Click **Fit** above the view so you can see all of it.

![The box at 60 by 40 by 25 mm](/learn/bettercad/assets/box_sized.jpg){:class="img-fluid w-100"}

These boxes take sums, just like dimensions. `40/2 + 5` works. You can also use the arrow keys in a size box to step by 1 mm, with <kbd>Shift</kbd> for 10 and <kbd>Alt</kbd> for 0.1.

## Pull a face

The box needs to be a bit taller. You could retype H, but there's a more direct way.

> Press Push/pull (P), press on a face and drag it. Type a distance to be exact.

1. Press <kbd>P</kbd>.
2. Press on the top face and drag upwards.
3. Type `5`.

![Pulling the top face up by 5 mm](/learn/bettercad/assets/pushpull_preview.jpg){:class="img-fluid w-100"}

4. Press <kbd>Enter</kbd>.

![The box after the pull, now 30 mm tall](/learn/bettercad/assets/pulled.jpg){:class="img-fluid w-100"}

BetterCad reports **Pulled a face of Box 1 out 5 mm**, and the box is now 30 mm tall. Drag a face *inwards*, or type a negative number, and you push it in instead.

Push/pull works on any flat face of any part, not just Library shapes. It's the fastest way to say "this side, a bit further out".

## Move and turn

Two more tools put a part where you want it.

- **Move** (<kbd>M</kbd>): click a part and arrows appear. Drag an arrow to slide the part along that direction. Hold <kbd>Shift</kbd> to snap to 5 mm steps
- **Turn** (<kbd>R</kbd>): click a part and rings appear. Drag a ring to rotate around that axis

![The Move tool waiting for you to click a part](/learn/bettercad/assets/move_tool.jpg){:class="img-fluid w-100"}

When you need an exact position, type it. With a part selected, **Position** and **Rotation** in Properties each have X, Y and Z boxes. You'll use those in the next lesson to aim a cable hole.

Press <kbd>Esc</kbd> to put a tool down. Leave the box at the centre for now.

## Sketch or shape?

| Start with | When |
|---|---|
| A Library shape | The part is basically a box, a cylinder or a wedge, or a few of them stuck together |
| A sketch | The outline is the interesting bit: a bracket, a plate with slots, anything with curves |
{:class="table table-single"}

Most real parts use both. Nothing stops you sketching on the face of a Library box.

Keep this model open. In the next lesson you'll put holes in it.

---

## Try it Yourself

1. Add a **Wedge** and a **Torus** from the Library and look at which size boxes each one has.
2. Push the front face of a spare box in by 10 mm using a negative distance.
3. Move a spare shape exactly 30 mm along X by typing in its Position, then delete it with <kbd>Delete</kbd>.

## Common Issues

**Problem**: A new shape landed beside the one I already had, not where I wanted it.
**Solution**: Use Move (<kbd>M</kbd>) to drag it into place, or type a Position.
**Why**: The first Library shape arrives at the centre. Later ones are put down next to what's already there so they don't overlap.

**Problem**: Push/pull moved the wrong face.
**Solution**: Press <kbd>Ctrl</kbd>+<kbd>Z</kbd>, turn the view so the face you want is clearly visible, and press on the middle of it.
**Why**: You pull whichever face is under the pointer when you press.

**Problem**: I pressed R to draw a rectangle and got the Turn tool.
**Solution**: Start a sketch first with <kbd>K</kbd>.
**Why**: Some keys do one job in a sketch and another outside it. R is Rectangle in a sketch and Turn in the model.

## Check your understanding

{% quiz %}
? Where do you type an exact size for a Library shape?
- [ ] On the timeline
- [x] In the Size boxes in Properties
- [ ] In the Browser
> Select the shape and its W, D and H are in Properties.

? You type -5 as a Push/pull distance. What happens?
- [ ] Nothing, negative numbers aren't allowed
- [ ] The whole part moves down
- [x] The face is pushed 5 mm into the part
> Positive pulls out, negative pushes in.

? Which tool rotates a part?
- [ ] Move
- [x] Turn
- [ ] Fillet
> Turn (R) puts rings on the part. Drag a ring to rotate around that axis.
{% endquiz %}
