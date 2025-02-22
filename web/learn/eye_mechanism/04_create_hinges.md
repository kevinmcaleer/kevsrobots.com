---
layout: lesson
title: Create Hinge Sketch
author: Kevin McAleer
type: page
cover: /learn/eye_mechanism/assets/eye_mechanism.jpg
date: 2023-01-29
previous: 03_revolve.html
next: 05_hinge.html
description: Create the hinge profile
percent: 55
duration: 3
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


* **Create new Sketch** - Create a new sketch on the edge of the eyelid by clicking the `CONSTRUCT` Menu and selecting the `offset plane` option.

![Offset plane](assets/eye22.jpg){:class="img-fluid w-50"}

---

* **Select the `YZ` plane** - select the `YZ` plane, this is th eone that is facing the right side of our model.

![Offset plane](assets/eye23.jpg){:class="img-fluid w-50"}

---

* **Click on the point side the eyelid** - Click on the point thats highlighted in the image below. The new plane should then be offset by `20mm` from the `YZ` plane.

![Offset plane](assets/eye24.jpg){:class="img-fluid w-100"}

---

* **Create the hinge profile** - Select the newly created offset plane and click on the `Create Sketch` button to create a new sketch on this plane.

> ## The ViewCube
>
> Its best to move the camera rotation to inside the eyelid so you create the sketch in the correct orientation. Move the camera by using the `viewcube`.
>
> ![Offset plane](assets/eye25.jpg){:class="img-fluid w-50"}

---

![Offset plane](assets/eye26.jpg){:class="img-fluid w-50"}

---

* **Slice** - Click on the `Slice` option from the `Sketch Palette`. Slice will remove the parts of the model that are between the camera and the selected sketch profile. This allows us to see inside of the model.

![Offset plane](assets/eye27.jpg){:class="img-fluid w-50"}

---

* **Create three circles** - Using the circle creation mode (press `c`), create three circles:

  * one that is 2mm
  * one that is 6mm
  * one that is 8mm
  
The origin of each circle is the point where the horizonal and verticle lines of our profile meet (the origin).

![Offset plane](assets/eye28.jpg){:class="img-fluid w-50"}

---

* **Create a square** - Use the rectangle drawing mode by pressing `r`. Start the rectangle at the origin, and drag it until the bottom line is touches the bottom of the circle, and the leftmost side touches the left side of the circle. You can also use the Tangent tool to ensure the line and circles are aligned.

![Offset plane](assets/eye29.jpg){:class="img-fluid w-50"}

> ### The `Tangent` Tool
>
> * **Select the line** - Select the left line of the rectangle
> * **Click Tangent** - and then click the `tangent` tool button from the toolbar.
> * **Select the circle** - Select the circle to create the tangent constraint which alings the line to the circle.

![Offset plane](assets/eye30.jpg){:class="img-fluid w-50"}

---

> ## Constraints
>
> Constraints **allow you to relate one sketch entity to another sketch entity**. If you look at. the constraint icons in the sketch palette you'll see that the sketch constraints use geometric expressions, with the exception of fix/unfix.
{:class="bg-blue"}

---

* Finish Sketch**** - Click `the Finish Sketch` button

---
