---
title: Joints and Export
description: Connect the eyelids with a joint, test the movement and export the parts for 3D printing
layout: lesson
type: page
---

## Join the eyelids

The two eyelids pivot around the same screw hole. A **joint** tells Fusion 360 how they're allowed to move, so you can check they open and close without crashing into each other.

* **As-built Joint** - Click the `ASSEMBLE` toolbar menu, then `As-built Joint`. We use an as-built joint because the parts are already in the right place.
* **Select the components** - Click the `Upper Eyelid`, then the `Lower Eyelid`.
* **Set the type** - Choose `Revolute` as the joint type, so the parts can only rotate.
* **Pick the axis** - Click the edge of the screw hole in the hinge. The pivot is the centre of that hole.
* **Click Ok** - Click `Ok` to create the joint.

---

## Test the movement

* **Drive Joints** - Right-click the joint in the browser (or the timeline) and choose `Drive Joints`, then type an angle such as `-60` to watch the lower eyelid swing open.
* **Animate** - Right-click the joint again and choose `Animate Joint` to see it open and close on its own. Press `Esc` to stop.

If the eyelids hit each other, check the hinge dimensions in [lesson 8](08_hinge.html).

---

## Export the parts

Each eyelid is printed separately.

* **Save As Mesh** - In the browser, right-click the `Upper Eyelid` body and choose `Save As Mesh`.
* **Choose STL** - Set the format to `STL` and the refinement to `High`, then save.
* **Repeat** - Do the same for the `Lower Eyelid`.

Open the STL files in your slicer. Print each eyelid with its open side facing up so the dome doesn't need support, and use an M2 screw (2mm, the size of our screw hole) as the hinge pin.

---

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
{% endquiz %}

## Try it yourself

1. Add an eyeball: a 38mm sphere in its own component, centred on the origin, so it sits inside the eyelids.
2. Challenge: design a small arm on the lower eyelid that a servo horn can push, so one servo can blink the eye.

## What's next

- Watch the video at the start of the course to see the full mechanism with its servo
- [Design a SMARS Robot in Fusion 360](/learn/smars_fusion360/): more Fusion 360 practice
- [Introduction to FreeCAD for Beginners](/learn/freecad/): try a free, open-source CAD program
