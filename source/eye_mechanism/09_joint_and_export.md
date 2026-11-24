---
title: Joints and Export
date_updated: 2026-11-24
description: Connect the eyelids with a joint, test the movement and export the parts for 3D printing
layout: lesson
type: page
cover: /learn/eye_mechanism/assets/banners/09_joint_and_export.jpg
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to join two components with a revolute joint, test the movement and export each part as an STL for printing.

## Join the eyelids

The two eyelids pivot around the same screw hole. A **joint** tells Fusion 360 how they're allowed to move, so you can check they open and close without crashing into each other.

* **As-built Joint** - Click the `ASSEMBLE` toolbar menu, then `As-built Joint`. We use an as-built joint because the parts are already in the right place.
* **Select the components** - Click the `Upper Eyelid`, then the `Lower Eyelid`.
* **Set the type** - Choose `Revolute` as the joint type, so the parts can only rotate.
* **Pick the axis** - Click the edge of the screw hole in the hinge. The pivot is the centre of that hole.
* **Click Ok** - Click `Ok` to create the joint.

---

## Test the movement

* **Drive Joints** - Right-click the joint in the browser (or the timeline) and choose `Drive Joints`, then drag the slider or type an angle such as `-60` to watch the lower eyelid swing open. Try a few angles and look for the point where the eyelids touch.
* **Animate** - Right-click the joint again and choose `Animate Joint` to see it open and close on its own. Press `Esc` to stop.

If the eyelids hit each other, check the hinge dimensions in [lesson 8](08_hinge.html).

---

## Export the parts

Each eyelid is printed separately.

* **Save As Mesh** - In the browser, right-click the `Upper Eyelid` body and choose `Save As Mesh`.
* **Choose STL** - Set the format to `STL` and the refinement to `High`, then save.
* **Repeat** - Do the same for the `Lower Eyelid`.

Open the STL files in your slicer. Print each eyelid with its open side facing up so the dome doesn't need support, and use an M2 screw (2mm, the size of our screw hole) as the hinge pin. Test-print the hinge first and check the fit before printing everything.

---

## Try it Yourself

1. Drive the joint to several angles and note where the eyelids touch.
2. Add an eyeball: a 38mm sphere in its own component, centred on the origin, so it sits inside the 40mm inner circle of the eyelids.
3. Challenge: design a small arm on the lower eyelid that a servo horn can push, so one servo can blink the eye.

## Common Issues

- **Problem**: The eyelids do not move when you drive the joint.
- **Solution**: Check the joint type is `Revolute` and that the axis is the centre of the screw hole.
- **Why**: A joint only allows the movement you define, so a wrong axis or type blocks it.

- **Problem**: The eyelids collide when they open.
- **Solution**: Go back to the hinge dimensions in [lesson 8](08_hinge.html) and check the circles and offsets.
- **Why**: The hinge geometry decides how close the two eyelids can get.

> **Teachers**: Printing is the part to plan. Print heads and beds are hot, so learners should not reach into a printer while it is running, and the parts should cool before handling. Printing a pair of eyelids takes a while on a shared printer, so queue the jobs or print one set per group. The hinge pin is a small screw, so count them out and keep them away from younger learners. When the eyelids are assembled, the hinge is a pinch point for fingers. If a class goes on to add a servo, power it from a separate supply and not from the 3V3 pin of the microcontroller board, which cannot supply a servo's current. A class with no printer can finish at the joint and test the movement on screen.
{:class="bg-blue"}

## Check your understanding

{% quiz %}
? Which joint type lets the eyelids pivot around the hinge?
- [ ] Rigid
- [ ] Slider
- [x] Revolute
> A revolute joint allows rotation around one axis, just like a door hinge.

? Which file format do you export to 3D print the eyelids?
- [x] STL
- [ ] DXF
- [ ] F3D
> F3D is Fusion's own format, DXF is for 2D drawings, and STL is what slicers read.

? What do you pick as the axis of the joint?
- [x] The centre of the screw hole in the hinge
- [ ] The outer edge of the eyelid
- [ ] The origin point of the sketch
> The eyelids pivot around the hinge screw, so the screw hole is the axis.
{% endquiz %}

## What's next

- Watch the video at the start of the course to see the finished mechanism in action
- [Design a SMARS Robot in Fusion 360](/learn/smars_fusion360/): more Fusion 360 practice
- [Introduction to FreeCAD for Beginners](/learn/freecad/): try a free, open-source CAD program
