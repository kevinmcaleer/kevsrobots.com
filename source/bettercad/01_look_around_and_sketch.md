---
title: Look Around and Sketch
description: Learn to move around the 3D view in BetterCad, then draw your first sketch, a 40 by 20 mm rectangle
layout: lesson
type: page
cover: /learn/bettercad/assets/banners/01_look_around_and_sketch.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to move around the 3D view, start a sketch on the ground plane, and draw a rectangle with an exact size.

Every part in this course starts the same way: a flat drawing, called a **sketch**, that you then pull into 3D. In this lesson you'll learn to move around and draw one.

---

## Look around

Start with an empty model. If the start card is in the way, press <kbd>Esc</kbd>.

> Drag to look around. Right-drag or drag two fingers to pan, scroll or pinch to zoom.

Try all three:

| To | Mouse | Trackpad or touch |
|---|---|---|
| Turn the view | Drag | Drag with one finger |
| Slide the view (pan) | Right-drag | Drag with two fingers |
| Zoom | Scroll | Pinch |
{:class="table table-single"}

Lost? Click **Fit** above the view to bring everything back into frame, or click a face of the view cube. The arrow keys turn the view too.

The red line is the **X** axis, green is **Y**, and **Z** points up. Everything in BetterCad is in millimetres.

## Start a sketch

> Press Sketch (K) and click the ground. Draw a rectangle with R, then press Enter to finish the sketch.

Let's take that one piece at a time.

1. Press <kbd>K</kbd>, or click **Sketch** in the toolbar.

BetterCad asks where to draw: *Click a face or a plane to sketch on it.* As you move the pointer, the three planes light up.

![Choosing a plane to sketch on](/learn/bettercad/assets/sketch_pick_plane.jpg){:class="img-fluid w-100"}

2. Click the ground, the flat plane.

The view turns to look straight down, and the toolbar changes to the **Sketch** tools: Line, Rectangle, Circle, Arc and friends.

![Sketch mode, looking straight down at the plane](/learn/bettercad/assets/sketch_mode.jpg){:class="img-fluid w-100"}

The black cross in the middle is the **origin**, the centre of your model's world.

## Draw a rectangle

1. Press <kbd>R</kbd> for the Rectangle tool.
2. Click once for the first corner.
3. Move the pointer away. Don't click yet.
4. Type `40`, press <kbd>Tab</kbd>, type `20`.

![Typing the width and height of the rectangle](/learn/bettercad/assets/rect_typed.jpg){:class="img-fluid w-100"}

5. Press <kbd>Enter</kbd>.

You have a 40 × 20 mm rectangle, with its sizes shown in green.

![The finished rectangle with its two sizes](/learn/bettercad/assets/rect_done.jpg){:class="img-fluid w-100"}

Typing sizes as you draw is the BetterCad way. You can click the second corner by eye and fix the size later, but typing is quicker and the number is exact.

The little **H** and **V** marks say the lines are horizontal and vertical. The status line says *2 degrees of freedom*. Lesson 4 explains both. For now, they mean the rectangle is the right size but could still slide about.

## Finish the sketch

1. Press <kbd>Esc</kbd> to put the Rectangle tool down.
2. Press <kbd>Enter</kbd>, or click **Finish sketch**.

The view swings back to 3D, and your rectangle is lying on the ground. Look at the Browser: there's a **Sketches** folder with **Sketch 1** in it. Look at the timeline: one step.

![The finished sketch lying on the ground plane](/learn/bettercad/assets/sketch_finished.jpg){:class="img-fluid w-100"}

That flat outline is called a **profile**. In the next lesson you'll pull it up into a solid.

---

## Try it Yourself

1. Double-click **Sketch 1** in the Browser to open it again. Double-click the `40 mm` size and change it to `50`.
2. Start a second sketch and draw a circle with <kbd>C</kbd>. Click the centre, move away, type a diameter and press <kbd>Enter</kbd>.
3. Press <kbd>Ctrl</kbd>+<kbd>Z</kbd> until you're back to one 40 × 20 rectangle. You'll need it for the next lesson.

## Common Issues

- **Problem**: Pressing K or R does nothing.
- **Solution**: Click on the 3D view first, then press the key.
- **Why**: Single-letter shortcuts are switched off while the cursor is in a text box, so typing a size never fires a tool by accident.

- **Problem**: BetterCad asks about an open shape when I finish.
- **Solution**: Make sure the ends of your lines meet. Drag one end onto the other to join them.
- **Why**: Only a closed shape can become a solid. An open one has no inside.

- **Problem**: My rectangle is tiny or off screen.
- **Solution**: Scroll to zoom, or right-drag to pan. This works while you're sketching too.
- **Why**: The view starts zoomed out so the whole printer bed fits.

## Check your understanding

{% quiz %}
? Which key starts a sketch?
- [ ] S
- [x] K
- [ ] E
> K is Sketch. S is Shell, and E is Extrude.

? How do you give a rectangle an exact size while drawing it?
- [ ] Drag slowly
- [x] Type the width, press Tab, then type the height
- [ ] You can't until it is finished
> Click one corner, then type width, Tab, height and press Enter.

? What must a sketch shape be before it can become a solid?
- [x] Closed
- [ ] Square
- [ ] Centred on the origin
> A closed shape has an inside, and the inside is what gets extruded.
{% endquiz %}
