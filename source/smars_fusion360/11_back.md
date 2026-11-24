---
title: Back cutout profile
description: Sketch the 52mm by 7.5mm profile for the back opening
layout: lesson
type: page
cover: /learn/smars_fusion360/assets/banners/11_back.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to sketch a rectangular profile for an opening using width and depth dimensions.

---

The front, back and sides have all been shaped. Next we plan the opening at the back of the chassis, the doorway that will later let the Arduino slide in. As with every cut, we first draw the profile and then cut it in the next lesson.

## The back profile

A profile is created for the back cutout section. This has 2 dimensions:

* width = 52mm
* depth = 7.5mm

![Chassis Back profile Cad Drawing](assets/chassis_back_profile.png){:class="img-fluid w-75 m-3"}

## Reading the numbers

The chassis is 58mm wide. A 52mm wide opening leaves the strip of body at each side, so the walls still hold the shape together. The depth of 7.5mm tells you how far the opening reaches in.

Keep an eye on the units. Both numbers are in millimetres, and the 52 and 7.5 are exactly as designed. Do not round them: a printed slot that is half a millimetre off can stop a board sliding in.

## What to check

The sketch should be a closed rectangle, with both dimensions typed in, sitting where the picture shows. Orbit the model to be sure the sketch is on the back of the chassis.

## Common Issues

- **Problem**: The sketch is on the wrong side of the chassis.
- **Solution**: Delete it and start again on the correct plane or face, as shown in the video.
- **Why**: The cut will go wherever the sketch is, so the plane choice decides where the opening ends up.

- **Problem**: The rectangle will not centre.
- **Solution**: Check the picture to see where the measurements start from, and dimension from the edges shown.
- **Why**: Without dimensions or constraints to the edges, the rectangle can slide around the sketch.

## Try it Yourself

1. Work out how much body is left at each side of the opening if it is centred: (58 minus 52) divided by 2.
2. Sketch the rectangle with the wrong width, then fix it by editing the dimension.
3. Challenge: why do you think the opening is almost as wide as the chassis, rather than much narrower?

## Check your understanding

{% quiz %}
? What are the dimensions of the back cutout profile?
- [ ] 42.5mm wide and 14.5mm deep
- [x] 52mm wide and 7.5mm deep
- [ ] 28mm wide and 10.5mm deep
> The back profile is 52mm wide and 7.5mm deep.

? What is the back opening for?
- [ ] To hold the wheels
- [x] To let the Arduino slide in
- [ ] To make the walls thicker
> The cut at the back is where the Arduino will slide in.

? Why not round 7.5mm up to 8mm?
- [x] A printed opening that is out by half a millimetre may stop parts fitting
- [ ] Fusion only accepts decimals
- [ ] 8mm is too big for Fusion
> Fit matters on small printed parts, so use the numbers exactly.
{% endquiz %}
