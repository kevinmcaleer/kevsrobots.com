---
title: Servo Arms
description: >-
    Print the four servo arms
layout: lesson
type: page
cover: /learn/smars_quad/assets/banners/quad_05.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to print the four servo arms and say what they do.

## Print the servo arms

Print out the servo arms, you will need 4 of them.

![Print the servo arms](assets/instruction05.png){:class="img-fluid w-25"}

## What the arm does

The servo arm bolts to a servo horn (the small plastic part that comes in the servo's bag), so when the servo turns, the arm swings. Four legs, one arm each, means four arms.

```mermaid
flowchart LR
    Servo["Servo output shaft"] --> Horn["Servo horn"]
    Horn --> Arm["Printed servo arm"]
    Arm --> Leg["Moving leg"]
```

Printed arms carry the load of a leg, so they are worth printing a little stronger than the other parts. More perimeters (walls) in your slicer usually helps more than more infill, because most breaks start at the outside. Print one first and flex it gently to check.

## Try it Yourself

1. Print one arm and hold it against a servo horn. Do the screw holes line up?
2. Print the other three and count your four arms.
3. Challenge: change one printer setting, such as the number of walls, on a spare arm and compare how much it flexes.

## Common Issues

- **Problem**: An arm snapped when I flexed it
- **Solution**: Reprint with more walls, and check the layer lines run along the arm, not across it
- **Why**: Layers are weakest between each other, so the print orientation matters

- **Problem**: The screw holes are too small
- **Solution**: Open them out slightly with a small drill bit, or reprint
- **Why**: Printers often make small holes a little undersized

## Check your understanding

{% quiz %}
? How many servo arms do you print?
- [ ] 8
- [x] 4
- [ ] 2
> The lesson says you will need 4 of them.

? What does the servo arm connect to?
- [ ] The PCA9685
- [x] The servo horn
- [ ] The Raspberry Pi
> The arm is screwed to the horn that fits on the servo shaft.

? Which print change usually makes arms stronger?
- [x] More perimeters (walls)
- [ ] A different colour
- [ ] A bigger nozzle only
> Most breaks start at the outside of the print.
{% endquiz %}
