---
layout: lesson
title: Save as STL
author: Kevin McAleer
type: page
cover: /learn/freecad_smars/assets/cover.jpg
date: 2025-12-12
previous: 08_motor_holder.html
next: 10_summary.html
description: Learn how to export the SMARS robot base as an STL file using FreeCAD.
percent: 90
duration: 1
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


In this lesson we'll learn how to export the SMARS robot base as an STL file for 3D printing.

1. First, ensure that the SMARS robot base model is selected in the Model tree.

    ![Select model](assets/stl01.png){:class="img-fluid w-100"}

2. Next, click on the `File` menu in the top left corner of FreeCAD.

    ![File menu](assets/stl02.png){:class="img-fluid w-100"}

3. From the dropdown menu, select `Export...`.

    ![Export option](assets/stl03.png){:class="img-fluid w-100"}

4. In the file dialog that appears, navigate to the folder where you want to save the STL file.

    - In the `Save as type` dropdown, select `STL Mesh (*.stl)`.

    ![Select STL type](assets/stl04.png){:class="img-fluid w-100"}

5. Enter a name for your STL file, such as `smars_robot_base.stl`, and click the `Save` button.

---
