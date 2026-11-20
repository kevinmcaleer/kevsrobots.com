---
title: Holes That Fit
description: Calibrate BetterCad's hole offset to your own 3D printer with a test plate, so screw holes come out the right size every time
layout: lesson
type: page
cover: /learn/bettercad_printing/assets/banners/05_holes_that_fit.jpg
date_updated: 2026-11-20
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Draw a 3 mm hole, print it, and try an M3 screw. It won't go in. Printed holes come out small: the plastic is squeezed slightly wider than the path the nozzle follows, and a circle is drawn as a string of short straight lines that cut the corners.

Every maker meets this. BetterCad deals with it using one number, the **hole offset**, and gives you a quick way to find the right value for your printer.

---

## How BetterCad sizes a hole

The Hole tool (<kbd>H</kbd>) asks for the *screw*, not the hole. With **Fit** set to **Clearance for printing**, it makes the hole the screw's size plus the hole offset.

![The Hole tool's Fit menu](/learn/bettercad_printing/assets/hole_fit_menu.jpg){:class="img-fluid w-100"}

With the default offset of 0.4 mm, an M3 hole is drawn at 3.4 mm, so that it prints at something close to 3.1 or 3.2 and the screw slides in.

| Screw | Drawn at (0.4 mm offset) |
|---|---|
| M2 | 2.4 mm |
| M2.5 | 2.9 mm |
| M3 | 3.4 mm |
| M4 | 4.4 mm |
| M5 | 5.4 mm |
| M6 | 6.4 mm |
{:class="table table-single"}

The offset is in the Print panel under **Holes**, and you can set it anywhere from 0 to 1 mm.

![The Holes and Fits settings in the Print panel](/learn/bettercad_printing/assets/holes_settings.jpg){:class="img-fluid w-100"}

The panel says most printers need 0.2 to 0.6 mm. So which are you?

## Print the test plate

1. In the Print panel, click **Calibrate holes**.

BetterCad opens a ready-made test plate as a new model.

![The hole calibration plate](/learn/bettercad_printing/assets/hole_calibration.jpg){:class="img-fluid w-100"}

It's a 3 mm thick plate with a grid of holes:

- **Rows**, from the front: M2, M2.5, M3, M4, M5, M6
- **Columns**, from the left: offsets of 0.2, 0.3, 0.4, 0.5 and 0.6 mm
- **Notches** on the front edge count the column. One notch is 0.2 mm, five is 0.6 mm

2. Export it (<kbd>Ctrl</kbd>+<kbd>E</kbd>) as a 3MF or STL.
3. Slice it with your normal settings and print it flat. Use the material and layer height you use most.

It's small and quick, and it's the most useful ten grams of plastic you'll print this year.

## Read the result

1. Take the screw size you use most. For most robot builders that's M3.
2. Find its row and try the screw in each hole, starting at the one-notch end.
3. Stop at the **first** hole the screw slides through without being forced or screwed in.
4. Count the notches below that column. That's your offset.

| The screw first slides through at | Your offset |
|---|---|
| One notch | 0.2 mm |
| Two notches | 0.3 mm |
| Three notches | 0.4 mm |
| Four notches | 0.5 mm |
| Five notches | 0.6 mm |
{:class="table table-single"}

Check a second size if you have the screws. They usually agree. If they don't, go with the size you use most.

## Set it

Back in your own model:

1. Open the Print panel and type your number into **Hole offset**.
2. If you saved a printer in lesson 1, click **Save to** it so every future model gets the same value.

From now on, every new **Clearance for printing** hole uses your offset.

## What about holes you've already made?

Each Hole step remembers the offset it was made with. Changing the offset doesn't silently resize anything, because a model that changes shape behind your back is worse than one that's slightly wrong.

When a model has printed-clearance holes made at a different offset, the Print panel shows a button: **Update N holes**. Click it and those steps are edited to the new offset in one go, as a single undo step.

## Holes the offset doesn't touch

| Fit | Why it's left alone |
|---|---|
| Heat-set insert | Insert makers quote the printed hole size directly |
| Clearance, Close | Standard sizes for drilled or machined parts |
| Tap | The drill size for cutting a thread |
{:class="table table-single"}

Only **Clearance for printing** uses your offset. And it applies to holes made with the Hole tool. A circle you sketch and cut yourself is exactly the size you drew, so add the allowance yourself.

---

## Try it Yourself

1. Print the hole calibration plate and find your offset for M3.
2. Set the offset, then make a 20 × 20 × 3 mm plate with one M3 hole. Print it and try the screw.
3. Change the offset by 0.1 mm in a model that already has holes. What does the Print panel offer?

## Common Issues

**Problem**: The screw fits a different column on the M3 row than on the M5 row.
**Solution**: Use the value for the size you use most, or split the difference.
**Why**: Small circles are affected more by how the slicer draws curves.

**Problem**: None of the holes fit, even at five notches.
**Solution**: Check your slicer isn't shrinking holes or over-extruding. Calibrate flow first, then print the plate again.
**Why**: An offset over 0.6 mm usually points to an extrusion problem, not a design one.

**Problem**: My holes fitted in PLA but are tight in PETG.
**Solution**: Print the plate again in PETG and save a second printer for it, such as "A1 Mini PETG".
**Why**: Materials flow and shrink differently.

## Check your understanding

{% quiz %}
? Why do printed holes come out small?
- [ ] The STL file shrinks them
- [x] The plastic spreads wider than the nozzle path, and curves are drawn as short straight lines
- [ ] Screws are bigger than their label
> Both effects eat into the hole, so it needs to be drawn oversize.

? On the test plate, which hole tells you your offset?
- [ ] The biggest one the screw fits
- [x] The first one the screw slides through
- [ ] The one in the middle
> Start at one notch and stop at the first hole that takes the screw without force.

? You change the hole offset. What happens to holes already in the model?
- [ ] They all resize at once
- [ ] They're deleted
- [x] Nothing, until you click Update N holes
> Each Hole step keeps the offset it was made with, so nothing changes without you asking.
{% endquiz %}
