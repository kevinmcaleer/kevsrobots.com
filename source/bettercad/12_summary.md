---
title: Summary and Next Steps
description: A recap of everything in Learn BetterCad, a keyboard shortcut cheat sheet and where to go next
layout: lesson
type: page
cover: /learn/bettercad/assets/banners/12_summary.jpg
date_updated: 2026-11-20
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

You started with an empty screen and finished with a parametric phone stand that prints without supports. That's the whole journey from idea to object, and you can repeat it for any part you need.

![The finished phone stand](/learn/bettercad/assets/stand_done.jpg){:class="img-fluid w-100"}

---

## What you made

| Part | What it taught you |
|---|---|
| A filleted block | Sketch, Extrude, Fillet and Export |
| A plate with a screw hole | Constraints, dimensions and "fully constrained" |
| An enclosure with screw posts | Library shapes, Push/pull, the Hole tool, hole shapes, Shell |
| The same enclosure, wider | The timeline and parameters |
| A phone stand | All of it, planned around five parameters |
{:class="table table-single"}

## The ideas worth keeping

- **A sketch is a set of rules.** Blue lines can still move. Black lines are fully constrained and stay where you put them
- **A model is a recipe.** The timeline holds the steps, and changing an early step replays the rest
- **Names beat numbers.** A size typed as a parameter follows that parameter. A size typed as a number stays put
- **Order matters.** Holes before Shell gives you screw posts. Shell before holes doesn't
- **Design for the printer.** Fillet what faces up, chamfer what faces down, and let the overhang check have the last word
- **Keep the .bcad.** An STL is a snapshot. The `.bcad` is the design

## Cheat sheet

| Key | Tool | | Key | Tool |
|---|---|---|---|---|
| <kbd>V</kbd> | Select | | <kbd>K</kbd> | Sketch |
| <kbd>M</kbd> | Move | | <kbd>E</kbd> | Extrude |
| <kbd>R</kbd> | Turn | | <kbd>P</kbd> | Push/pull |
| <kbd>X</kbd> | Section | | <kbd>G</kbd> | Fillet |
| <kbd>I</kbd> | Measure | | <kbd>S</kbd> | Shell |
| <kbd>H</kbd> | Hole | | <kbd>C</kbd> | Combine |
{:class="table table-single"}

In a sketch: <kbd>L</kbd> Line, <kbd>R</kbd> Rectangle, <kbd>C</kbd> Circle, <kbd>D</kbd> Dimension, <kbd>Enter</kbd> to finish.

| Keys | What they do |
|---|---|
| <kbd>/</kbd> or <kbd>Ctrl</kbd>+<kbd>K</kbd> | Find any command |
| <kbd>Esc</kbd> | Back out one step |
| <kbd>Ctrl</kbd>+<kbd>G</kbd> | Group the selected shapes |
| <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>G</kbd> | Ungroup |
| <kbd>Alt</kbd>+<kbd>←</kbd> / <kbd>Alt</kbd>+<kbd>→</kbd> | Step back and forward along the timeline |
| <kbd>Ctrl</kbd>+<kbd>S</kbd> | Save |
| <kbd>Ctrl</kbd>+<kbd>E</kbd> | Export |
| <kbd>Ctrl</kbd>+<kbd>Z</kbd> | Undo |
{:class="table table-single"}

The full list is in the app under **Help**, **Keyboard shortcuts**.

## Things to design next

The best practice is a part you actually need. Some ideas, roughly in order of difficulty:

1. A cable clip for the edge of your desk
2. A lid for the enclosure from lessons 6 to 9, with four countersunk M3 holes that line up with the posts
3. A mount for a Raspberry Pi Pico, with holes on its 11.4 × 47 mm mounting pattern
4. A bracket to hold a servo, measured from the real thing with callipers

## What's next

- **[Print It Right](/learn/bettercad_printing/)** is the follow-on course. It covers calibrating holes and fits to your own printer, choosing an orientation, tolerances for parts that slide or snap together, and exporting for different slicers
- The **[Start with BetterCad](/learn/learning_pathways/bettercad.html)** pathway puts the BetterCad courses in order
- Want to see how other CAD tools approach the same ideas? Try [Introduction to FreeCAD for Beginners](/learn/freecad/)
- Have a design that needs a brain? [Getting Started with Snakie](/learn/snakie_getting_started/) gets a Raspberry Pi Pico running MicroPython

BetterCad is in alpha and changing quickly. **Help**, **What's new** in the app lists what's been added, and **Report a problem** goes straight to the people building it.

---

## Try it Yourself

1. Pick one idea from the list above and design it. Start with a list of parameters.
2. Share a link to something you've made and ask a friend to open their own copy.
3. Go back to your very first block from lesson 2 and rebuild it in under a minute.

## Check your understanding

{% quiz %}
? A size should follow another size when it changes. What do you type?
- [ ] The number, worked out on a calculator
- [x] A parameter name, or a sum that uses one
- [ ] Nothing, BetterCad guesses
> Only a box that contains a parameter name follows that parameter.

? Which gives you screw posts inside an enclosure?
- [x] Holes first, then Shell
- [ ] Shell first, then holes
- [ ] The order makes no difference
> Shell wraps a wall around whatever exists when it runs, including holes.

? Which file do you keep so you can edit the design later?
- [ ] The STL
- [ ] The 3MF
- [x] The .bcad
> The .bcad holds the timeline, sketches and parameters.
{% endquiz %}
