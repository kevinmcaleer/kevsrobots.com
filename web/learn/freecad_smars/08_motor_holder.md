---
layout: lesson
title: Motor Holder
author: Kevin McAleer
type: page
cover: /learn/freecad_smars/assets/cover.jpg
date: 2025-12-12
previous: 07_wheel_stubs.html
next: 09_save_as_stl.html
description: Learn how to create the motor holder for the SMARS robot using FreeCAD.
percent: 80
duration: 3
navigation:
- name: Building SMARS with FreeCAD
- content:
  - section: Navigating around FreeCAD
    content:
    - name: Navigation in FreeCAD
      link: 01_navigation.html
  - section: Designing the SMARS Base
    content:
    - name: Creating the Base
      link: 02_creating_the_base.html
    - name: Shell and Fillet the Base
      link: 03_shell_and_fillet.html
    - name: Side Holes
      link: 04_sides_and_holes.html
    - name: Front and Rear Profiles
      link: 05_front_rear_profiles.html
    - name: Arduino Slots
      link: 06_arduino_slots.html
    - name: Wheel Stubs
      link: 07_wheel_stubs.html
    - name: Motor Holder
      link: 08_motor_holder.html
  - section: Exporting for 3D Printing
    content:
    - name: Save as STL
      link: 09_save_as_stl.html
  - section: Summary
    content:
    - name: Summary
      link: 10_summary.html
---


1. Create a new sketch on the floor face of the base. Click on the front face to select it, then click on `Create Sketch` in the toolbar.

    ![Select front face](assets/motor01.png){:class="img-fluid w-100"}

2. Using the `Rectangle` tool, draw 8 rectangles; we'll add the dimensions and constraints in the next step.

    ![Draw rectangles](assets/motor02.png){:class="img-fluid w-100"}

3. Add dimensions to the rectangles using the `Dimensions` tool. Follow the dimensions shown in the image below.

    ![Dimension rectangles](assets/motor03.png){:class="img-fluid w-100"}

---

We need to add two small cutouts for the motor holder tabs to fit into. We'll do this with a new sketch and a pocket operation.

4. Create a new sketch on the inside right face of the base. Click on the right inside face to select it, then click on `Create Sketch` in the toolbar.

    ![Select front face](assets/motor04.png){:class="img-fluid w-100"}

5. Use the `Clipping plane` menu item from the `View` menu to create a clipped view so we can see the surface we're working on. 

    - Click the `Clipping Y` option to create a vertical clipping plane.

    ![Clip plane](assets/motor05.png){:class="img-fluid w-100"}

6. Using the `Rectangle` tool, draw two rectangles; we'll add the dimensions in the next step.
    
    ![Draw rectangles](assets/motor06.png){:class="img-fluid w-100"}

7. We'll need to bring in the inner edges of the base to constrain the rectangles to. Use the `Create external geometry` tool to project in the inner edges of the base.

    ![Project geometry](assets/motor07.png){:class="img-fluid w-100"}

8. Add dimensions to the rectangles using the `Dimensions` tool. Follow the dimensions shown in the image below.
    
    ![Dimension rectangles](assets/motor08.png){:class="img-fluid w-100"}

9. Constrain the bottom edges of the rectangles to be coincident with the projected inner edge of the base using the `Constrain Coincident` tool.

    ![Constrain rectangles](assets/motor09.png){:class="img-fluid w-100"}

10. Close the sketch by clicking on the `Close` button in the toolbar.

    ![Close sketch](assets/motor10.png){:class="img-fluid w-100"}

    - Also close the clipping plane by clicking the `Y Clipping`, then select the `Clipping plane` menu item from the `View` menu again.

11. Rotate the view and replicate the sketch on the left inside face of the base by create a new sketch on the left inside face and using the `Create sketch` tool.

    ![Select left face](assets/motor11.png){:class="img-fluid w-100"}

    ![Select left face](assets/motor12.png){:class="img-fluid w-100"}

    - close the sketch when done.

    - close the clipping plane again if needed.

12. We can now pocket the cutouts from the base. With the sketch selected in the Model tree on the left, click on the `Pocket` button in the toolbar.

    ![Select left face](assets/motor13.png){:class="img-fluid w-100"}

    - In the `Pocket` dialog, set the `Length` value to `1.5mm` to cut through the side of the base. Click `OK` to apply the pocket.

---
