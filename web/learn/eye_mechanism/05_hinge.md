---
layout: lesson
title: Extrude The Hinge
author: Kevin McAleer
type: page
cover: /learn/eye_mechanism/assets/banners/05_hinge.jpg
date: 2023-01-29
previous: 04_create_hinges.html
next: 06_lower_eyelid_sketch.html
description: Create the hinge from the profile and join it to the eyelid
percent: 60
duration: 4
quiz_total: 27
date_updated: 2026-11-24
navigation:
- name: Robot Eye Mechanism
- content:
  - section: Overview
    content:
    - name: Overview
      link: 00_intro.html
  - section: Initial Design
    content:
    - name: Create a Component
      link: 01_create_component.html
    - name: Create a Sketch
      link: 02_create_sketch.html
    - name: Revolve
      link: 03_revolve.html
  - section: Refine the Top Eyelid detail
    content:
    - name: Create Hinge Sketch
      link: 04_create_hinges.html
    - name: Extrude The Hinge
      link: 05_hinge.html
  - section: Create the Lower Eyelid
    content:
    - name: Create the Lower Eyelid Component
      link: 06_lower_eyelid_sketch.html
    - name: Creating the Lower Eyelid
      link: 07_create_lower_sketch.html
    - name: Create Hinge from Profile
      link: 08_hinge.html
  - section: Joints and Export
    content:
    - name: Joints and Export
      link: 09_joint_and_export.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to extrude, cut and mirror features to build a hinge with a screw hole on both sides of the eyelid.

* **Extrude Join** - Extrude the middle and inner circle out towards the outer edge of the eyelid by `2mm`.

![Extrude](assets/eye31.jpg){:class="img-fluid w-50"}

Make sure the operation is set to `join` in the `Edit Feature` dialog box, then click `Ok`.

![Extrude](assets/eye32.jpg){:class="img-fluid w-50"}

---

When you Clicked `ok` in the last operation, Fusion hid the hinge sketch that we had created. To make this visible again we need to click the little `eye` icon next to the sketch.

![Extrude](assets/eye33.jpg){:class="img-fluid w-50"}

* **Make the Sketch Visible** - Click the `eye` icon next to the hinge sketch

![Extrude](assets/eye34.jpg){:class="img-fluid w-50"}

---

* **Extrude Cut** - Select the same profile again, press `e` to enter Extrude mode and this time type `1mm` as the measurement. You will notice that the extrusion of the profile turns red indicating that this is a `cut` operation.

![Extrude](assets/eye35.jpg){:class="img-fluid w-50"}

---

* **Extrude Cut** - Select the small profile from the sketch, press `e` to enter Extrude mode, and set the:

  * `Direction` to `Symmetric`
  * `Extent Type` to `All`
  * `Distance` to `All`

* **Click Ok** - Then click `Ok`

This will cutout the screw hole in both directions.

![Extrude](assets/eye36.jpg){:class="img-fluid w-100"}

---

> To make this next step easier, move the camera so you can see the outside edge of the eyelid. Use the [viewcube](04_create_hinges.html#the-viewcube) to move the camera.
{:class="bg-blue"}

* **Extrude Cut** - Select the square shape along with the rest of the profile from the sketch, press `e` to enter Extrude mode, and set the:

  * `Start` to `Object` and click the flat outside surface of the eyelid
  * `Extent Type` to `All`
  * `Distance` to `All`
  * `Operation` to `Cut`

![Extrude](assets/eye37.jpg){:class="img-fluid w-100"}

---

The finished result:

![Extrude](assets/eye38.jpg){:class="img-fluid w-50"}

---

> ## Mirror function
>
> We can mirror `bodies`, `components`, `features` and `faces` using the `Mirror` function. This is a huge timesaver, especially when parts need to be symmetrical; we can build one half and mirror the rest.
>
> ![Extrude](assets/eye39.jpg){:class="img-fluid w-100"}

---

* **Mirror** - Click the `Mirror` function from the `CREATE` toolbar menu.

![Extrude](assets/eye40.jpg){:class="img-fluid w-100"}

* **Select the Feature** - Select the last for features from the Timeline (they are all the Extrude operations we just did)

* **Select the Mirror Plane** - Click the `Mirror Plane` option from the `Edit Feature` Dialog box and then click the `YZ` Origin plane.

* **Click Ok** - Click `Ok` to complete the mirror operation.

The Eyelid Model should look like this:

![Extrude](assets/eye40.jpg){:class="img-fluid w-100"}

---

## Try it Yourself

1. Edit the 1mm cut depth to 0.5mm in the timeline and see how the hinge changes. Put it back afterwards.
2. Click the eye icon beside a sketch to hide it, then show it again.
3. Challenge: explain why we mirror features instead of drawing the second hinge from scratch.

## Common Issues

- **Problem**: The sketch disappeared after the extrude.
- **Solution**: Click the little eye icon beside the sketch in the browser to show it again.
- **Why**: Fusion hides a sketch once a feature has used it.

- **Problem**: The cut does not go all the way through.
- **Solution**: Set the Extent Type and Distance to `All`, and make sure the operation is `Cut`.
- **Why**: Without `All`, the cut stops at the distance you typed.

## Check your understanding

{% quiz %}
? What does **Slice** in the Sketch Palette do?
- [ ] Cuts the model in half permanently
- [x] Temporarily hides the part of the model between you and the sketch so you can see inside
- [ ] Splits a line into pieces
> Slice only changes the view while you're sketching. Nothing is removed from the model.

? Why do we **Mirror** the hinge features?
- [ ] To make them bigger
- [x] So the other side of the eyelid gets an identical hinge without drawing it again
- [ ] To turn them into construction lines

? What does the Extrude Cut operation do to the eyelid?
- [x] Removes material from it
- [ ] Adds material to it
- [ ] Mirrors it
> The extrusion turns red when it is a cut, because it removes material.
{% endquiz %}
