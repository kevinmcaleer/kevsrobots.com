---
title: Calibrate Servos
description: >-
    Check the servo rotation before tightening the screw
layout: lesson
type: page
cover: /learn/smars_quad/assets/banners/quad_07.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to check that a servo can move equally far each side of its middle before you tighten the horn.

## Check the servo rotation

Ensure the servo can move equally in each direction before tightening the screw.

![Check the servo rotation](assets/instruction07.png){:class="img-fluid w-25"}

## Why equal in each direction?

A hobby servo turns between two end stops. A leg should swing the same amount forward and backward, so we want the arm to sit in the middle of that travel when the servo is told to go to its centre position. If the horn goes on off-centre, the leg will run out of travel on one side while it still has plenty on the other.

```mermaid
flowchart LR
    A["End stop 1"] ---|"half the travel"| M["Middle: leg points straight"] ---|"half the travel"| B["End stop 2"]
```

Here is the check:

1. With the horn only loosely fitted, gently move the arm by hand to one end of its travel. Stop as soon as you feel it stop, and do not force it.
2. Do the same to the other end and picture the halfway point.
3. Is the arm resting at that halfway point? If not, pull the horn off, turn it one spline step and push it back on.

Turning by hand is only a rough check. The precise way is to power the servo from the PCA9685 and send it to its centre position. You will have a script for that in lesson 13, and you can come back here to adjust the horn.

> **Teachers**: Warn pupils never to force a servo by hand. Moving a servo's arm against its motor can damage the plastic gears in cheap servos. Let the board do the moving once it is wired up.

## Try it Yourself

1. Check one servo by hand, gently, and describe where the middle is.
2. Move the horn by one spline and check again. Is it better or worse?
3. Challenge: once you have run lesson 13's `centre.py`, check whether the arm points where you expect.

## Common Issues

- **Problem**: The arm moves much further one way than the other
- **Solution**: Pull the horn off, turn it one or two splines, and refit
- **Why**: The splines let you change the starting angle in small steps

- **Problem**: I heard grinding from the servo
- **Solution**: Stop and release the arm. Do not force it past its end stop
- **Why**: The servo gears are plastic and can chip if pushed against the stop

## Check your understanding

{% quiz %}
? Why check the servo can move equally in both directions before tightening?
- [x] So the leg can swing the same amount each side of the middle
- [ ] To test the battery
- [ ] Servos only work in one direction
> If the horn goes on off-centre, the leg will run out of movement on one side.

? How do you change the angle the horn sits at?
- [ ] Use a bigger screw
- [x] Pull it off, turn it a spline or two, and push it back on
- [ ] Print a new arm
> The splines let you change the angle in small steps.

? Why not force a servo past its end stop by hand?
- [ ] It makes the leg longer
- [x] It can damage the plastic gears
- [ ] It resets the Raspberry Pi
> Servo gears are small and can be damaged by force.
{% endquiz %}
