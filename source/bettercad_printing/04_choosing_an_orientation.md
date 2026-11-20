---
title: Choosing an Orientation
description: Decide which way up to print a part using BetterCad's orientation suggestions, balancing supports, strength and surface finish
layout: lesson
type: page
cover: /learn/bettercad_printing/assets/banners/04_choosing_an_orientation.jpg
date_updated: 2026-11-20
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

The same part can be an easy print or a nightmare depending on which face you put on the bed. Orientation decides how much support you need, how strong the part is and which surfaces look good. It costs nothing to change, so it's worth a minute's thought on every print.

---

## The suggestions

Under **Orientation** in the Print panel, BetterCad shows three small pictures: the model **as it lies** now, and two other ways of laying it down.

![The bracket turned to a suggested orientation](/learn/bettercad_printing/assets/orientation_bracket.jpg){:class="img-fluid w-100"}

Click one and the model is turned and placed in the middle of the bed: **Turned the model and laid it on the middle of the bed**. It's a single step, so <kbd>Ctrl</kbd>+<kbd>Z</kbd> takes you back.

BetterCad scores every way the model could rest on a flat face, using three things:

| It prefers | Because |
|---|---|
| More area touching the bed | The print sticks better and is less likely to warp or topple |
| Less overhang | Fewer supports, less waste, cleaner surfaces |
| A lower height | Shorter prints are quicker and wobble less |
{:class="table table-single"}

## See it work

Remember the tee from the overhang lesson? Standing up, it had 400 mm² of overhang.

![The tee standing, with a large overhang](/learn/bettercad_printing/assets/tee_overhang.jpg){:class="img-fluid w-100"}

One suggestion lays it on its side. With the overhang check still on, the result updates straight away.

![The tee lying on its side with no overhang](/learn/bettercad_printing/assets/tee_flat.jpg){:class="img-fluid w-100"}

No support at all. Another suggestion stands it on the end of the bar:

![The tee standing on the end of its bar](/learn/bettercad_printing/assets/tee_other.jpg){:class="img-fluid w-100"}

That one still needs support, 100 mm² of it, and it's tall and narrow. Leave the overhang check switched on while you try orientations and you can compare them in seconds.

## What the suggestions can't know

BetterCad calls these *suggestions* for a reason. It can measure contact, overhang and height. It can't know what the part is for. Three things are yours to judge.

### Strength

A print is a stack of layers, and the bond *between* layers is its weakest point. A part pulls apart along its layers far more easily than across them, like splitting wood along the grain.

So think about where the force goes. The bracket is a good example. Printed standing on its base, the layers run across the upright, and a load on the upright tries to peel them apart right at the corner. Printed on its side, each layer is a complete L shape, and the corner is as strong as the plastic itself.

**Lay the part so the layers run along the direction of the load, not across it.**

### Finish

- The face on the bed comes out flat and smooth, or textured like your build plate
- Top faces show the lines of the last layer
- Upright walls show layer lines, which are finer and neater than any sloped surface
- Gentle slopes show obvious steps
- Anything that touched support is rough

Put the face people will see where it will look best, and hide the supported face.

### Holes and accuracy

Holes are roundest when they point straight up. A hole on its side is built from stepped layers, and its top is a small overhang. If a hole has to be accurate, for a bearing or a shaft, try to print it upright.

## A way to decide

1. What will break it? Turn the part so the layers resist that.
2. Of the orientations that are strong enough, which needs least support? Use the overhang check.
3. Is the visible face in a good place?
4. Does it still fit the bed and have enough touching the plate to stay put?

When strength and support disagree, strength wins for a working part. Supports can be snapped off. A weak layer line can't be fixed.

---

## Try it Yourself

1. Open the **Bracket** sample, switch on the overhang check and click each suggestion. Note the support area for each.
2. Which of those orientations would you choose if the bracket has to hold a heavy shelf? Why?
3. Open the **Phone stand** sample. Is **As it lies** already the best? Try the others.

## Common Issues

**Problem**: None of the suggestions is the way I want it.
**Solution**: Use the **Turn** tool (<kbd>R</kbd>), or type a Rotation in Properties, then click **Centre on bed**.
**Why**: The suggestions only cover ways the model can rest flat.

**Problem**: The suggestion I picked is no longer highlighted.
**Solution**: Nothing is wrong. The picture marked **As it lies** always shows the current position.
**Why**: After turning, the new position becomes "as it lies" and the others are worked out again.

**Problem**: My model has several parts and they all turned together.
**Solution**: Export each part on its own and orient them in the slicer, or turn parts one at a time with the Turn tool.
**Why**: A suggestion turns the whole model as one.

## Check your understanding

{% quiz %}
? Where is a 3D print weakest?
- [ ] In the middle of a layer
- [x] Between layers
- [ ] On the face that touched the bed
> Layers bond less strongly to each other than the plastic holds together within a layer.

? Which three things do BetterCad's suggestions weigh?
- [ ] Colour, weight and cost
- [ ] Strength, finish and speed
- [x] Contact with the bed, overhang and height
> Strength and finish depend on what the part is for, so those are your call.

? Strength says print it one way, supports say another. Which usually wins for a working part?
- [x] Strength
- [ ] Fewer supports
- [ ] Whichever is quicker
> Supports can be removed. A part that splits along its layers is scrap.
{% endquiz %}
