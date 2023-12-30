---
layout: lesson
title: Create Hinge from Profile
author: Kevin McAleer
type: page
cover: /learn/eye_mechanism/assets/eye_mechanism.jpg
date: 2023-01-29
previous: 07_create_lower_sketch.html
description: Lets create the lower eyelid hinge from the new profile
percent: 100
duration: 1
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
---


## Lower Hinge

The lower eyelid hinge needs to fit over the upper eyelid hinge, so we can use that to extrude from.

---

* **Extrude** - Select the circle profiles and extrude by 2mm symmetrically.

![Eye profile](assets/eye52.jpg){:class="img-fluid w-50"}

---

* **Join** - Extrude the profile below by -2mm:
  
  * Set the `start` to `object`, and use the flat section as the new `object` profile

![Offset plane](assets/eye53.jpg){:class="img-fluid w-50"}

---

* **Extrude the screw hole** - extrude the middle screw hole profile, setting the distance to `all`

![Offset plane](assets/eye54.jpg){:class="img-fluid w-50"}

---

## Extrude to Object

There is a small gap between our parts that we need to fill.

![Offset plane](assets/eye55.jpg){:class="img-fluid w-50"}

Fusion has a handy feature for this in the extrude dialog box in the extent type called `to object`.

![Offset plane](assets/eye56.jpg){:class="img-fluid w-100"}

![Offset plane](assets/eye57.jpg){:class="img-fluid w-50"}

---

## Mirror

Lets mirror this new feature.

* **Mirror** - Click on the `CREATE` and Mirror function, then select the 4 features that have just been created and the origin as the mirror plane.

---
