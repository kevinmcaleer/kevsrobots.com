---
layout: lesson
title: Revolve
author: Kevin McAleer
type: page
cover: /learn/eye_mechanism/assets/eye_mechanism.jpg
date: 2023-01-29
previous: 02_create_sketch.html
next: 04_create_hinges.html
description: Revolve to create the eyelid
percent: 40
duration: 2
quiz_total: 9
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
