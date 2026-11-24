---
title: Side Profile
description: Sketch the inverted trapezium that gives SMARS its tank-like side
layout: lesson
type: page
cover: /learn/smars_fusion360/assets/banners/08_side.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to sketch a trapezium profile for a side cut-out using three dimensions.

---

If you have ever seen a SMARS robot, you will know its classic side shape: the distinctive notch cut into each side. In this lesson we sketch the outline of that notch, and the next lesson cuts it.

## The side profile

The profile for the side cutout is created next. This is an inverted trapezium shape, with three main dimensions:

* Top width = 28mm
* Bottom width = 16mm
* Depth = 10.5mm

![Chassis Side Profile Cad Drawing](assets/chassis_side_profile.png){:class="img-fluid w-75 m-3"}

A trapezium has two parallel sides of different lengths. Here it is upside down, so the wide side (28mm) is at the top and the narrow side (16mm) is at the bottom, 10.5mm lower down.

## Why a trapezium, and not a rectangle?

The sloped sides are the clever bit. They give SMARS its distinctive look and remove plastic from the sides. The slope is also kinder to a 3D printer than a square notch would be, because a sloped edge is easier to print than a sharp overhang. The 28mm and 16mm numbers set the slope.

## Sketching tips

- Draw the four lines of the trapezium, then add the three dimensions one at a time.
- A symmetrical shape is easiest to draw if you centre it on the middle of the side, as shown in the picture.
- Take a moment to check the shape is closed, as the next lesson needs a closed profile to cut with.

## Common Issues

- **Problem**: My trapezium is lopsided.
- **Solution**: Check the top and bottom are centred on the same middle line, and re-dimension if needed.
- **Why**: A trapezium has only some of its measurements fixed by the three dimensions. The position and centring also have to be fixed.

- **Problem**: The shape looks like a rectangle.
- **Solution**: Check the bottom width is 16mm and not 28mm.
- **Why**: If both widths are equal, the sloped sides become vertical.

## Try it Yourself

1. Sketch it, then change the bottom width to 20mm and look at the slope. Then put it back to 16mm.
2. Work out how much narrower the bottom is than the top: 28mm minus 16mm. How much does each sloped side move in?
3. Challenge: design your own side profile with different numbers. What changes about how the robot would look?

## Check your understanding

{% quiz %}
? What are the three dimensions of the side profile?
- [x] Top width 28mm, bottom width 16mm, depth 10.5mm
- [ ] Top width 16mm, bottom width 28mm, depth 14.5mm
- [ ] Top width 42.5mm, bottom width 28mm, depth 7.5mm
> The inverted trapezium is 28mm at the top, 16mm at the bottom and 10.5mm deep.

? Why is it called an inverted trapezium?
- [ ] It has four equal sides
- [x] It has two parallel sides of different lengths, with the wider one at the top
- [ ] It has no straight sides
> The wide side is at the top and the narrow side is at the bottom.

? Why does the profile need to be a closed shape?
- [x] The next lesson cuts with it, and a cut needs a closed profile
- [ ] Open shapes cannot be dimensioned
- [ ] Closed shapes are lighter
> Extrude and cut need a closed outline.
{% endquiz %}
