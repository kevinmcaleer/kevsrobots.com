---
layout: lesson
title: Revolve
author: Kevin McAleer
type: page
cover: /learn/eye_mechanism/assets/banners/03_revolve.jpg
date: 2023-01-29
previous: 02_create_sketch.html
next: 04_create_hinges.html
description: Revolve to create the eyelid
percent: 40
duration: 3
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

By the end of this lesson you will be able to turn a flat sketch profile into a curved 3D eyelid with the Revolve tool.

Revolve is the heart of this design. Instead of modelling a dome by pushing and pulling blocks, we draw its cross-section once and let Fusion 360 spin it around an axis, like a potter's wheel. The two circles from lesson 2 become the inside and outside of a hollow shell, which is exactly what an eyelid is.

* **Select the profile** - Click within the newly created profile to select it

![Profile](assets/eye18.jpg){:class="img-fluid w-50"}

---

* **Revolve the profile** - Click the Revolve Tool from the 'CREATE' Menu

![Profile](assets/eye19.jpg){:class="img-fluid w-50"}

* **Select the Axis** - Select the Blue `Z` Axis and you will see the new shape appear.

![Profile](assets/eye20.jpg){:class="img-fluid w-50"}

* **Set the rotation** - Set the rotation to 180° so the shape looks like the quarter of a hollow sphere.

![Profile](assets/eye21.jpg){:class="img-fluid w-50"}

---

## Try it Yourself

1. Revolve the profile by 90 degrees instead of 180, then undo (`Ctrl+Z`) and go back to 180.
2. Edit the Revolve feature in the timeline and change the angle to watch the eyelid update.
3. Challenge: revolve a full 360 degrees and describe the shape you get.

## Common Issues

- **Problem**: The Revolve tool will not let you pick a profile.
- **Solution**: Click inside the closed shape, not on its edge, so the profile highlights.
- **Why**: Revolve works on closed profiles, so Fusion needs to know which area to spin.

- **Problem**: The shape sweeps the wrong way.
- **Solution**: Check you picked the blue Z axis, then try the opposite angle.
- **Why**: The direction depends on the axis and the sign of the angle.

## Check your understanding

{% quiz %}
? What does the **Revolve** tool do?
- [ ] Pushes a sketch straight up into a solid
- [x] Spins a sketch profile around an axis to make a round solid
- [ ] Rotates the camera around the model
> Revolve is how we turn a flat curved profile into the domed eyelid.

? What does pressing `x` do to a selected sketch line?
- [x] Toggles it between a normal line and a construction line
- [ ] Deletes it
- [ ] Mirrors it
> Construction lines help you position things but aren't used to make solid shapes.

? What does it mean when a sketch line turns black?
- [ ] It has an error
- [x] It's fully defined and locked in place
- [ ] It's a construction line
{% endquiz %}
