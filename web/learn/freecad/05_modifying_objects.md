---
layout: lesson
title: Modifying Objects in FreeCAD
author: Kevin McAleer
type: page
cover: assets/4.png
date: 2024-04-17
previous: 04_creating_basic_shapes.html
next: 06_boolean_operations.html
description: Learn advanced techniques to modify and manipulate objects to create
  complex designs in FreeCAD.
percent: 45
duration: 2
navigation:
- name: Introduction to FreeCAD for Beginners
- content:
  - section: Getting Started with FreeCAD
    content:
    - name: Introduction to FreeCAD
      link: 01_introduction_to_freecad.html
    - name: Installing FreeCAD
      link: 02_installing_freecad.html
    - name: Overview of the FreeCAD User Interface
      link: 03_user_interface_overview.html
  - section: Basic Modeling Techniques
    content:
    - name: Creating Basic Shapes in FreeCAD
      link: 04_creating_basic_shapes.html
    - name: Modifying Objects in FreeCAD
      link: 05_modifying_objects.html
    - name: Mastering Boolean Operations in FreeCAD
      link: 06_boolean_operations.html
  - section: Advanced Design and Assembly
    content:
    - name: Assembly Design in FreeCAD
      link: 07_assembly_design.html
    - name: Creating Technical Drawings in FreeCAD
      link: 08_creating_drawings.html
    - name: Building and Testing Your FreeCAD Models Locally
      link: 09_exporting_models.html
  - section: Real-World Applications and Best Practices
    content:
    - name: Exporting and Sharing Your FreeCAD Models
      link: 10_practical_examples.html
    - name: Practical Examples and Best Practices in FreeCAD
      link: 11_tips_and_tricks.html
---


![Modifying Objects in FreeCAD]({{ page.cover }}){:class="cover"}

## Introduction to Modifying Objects

This lesson builds on the previous one by exploring more advanced modification tools available in FreeCAD. You will learn how to edit existing shapes and use Boolean operations to combine or subtract shapes to achieve your design goals.

---

## Key Modification Tools

1. **Placement Tool:**
   - Use this tool to position, rotate, and scale objects precisely. Access it through the property editor under `Placement`.

2. **Boolean Operations:**
   - **Union:** Combine two or more shapes into one.
   - **Difference:** Subtract one shape from another.
   - **Intersection:** Create a shape from the overlapping volume of two shapes.

3. **Fillet and Chamfer:**
   - **Fillet:** Round the edges of an object.
   - **Chamfer:** Create a beveled edge on one or more edges of an object.

---

## Using Boolean Operations

- **Task:** Combine a cylinder and a cube to form a T-shaped object.
- **Steps:**
  - Create a cube and a cylinder.
  - Position the cylinder so that it intersects part of the cube.
  - Select both objects, and use the `Boolean Union` operation to fuse them into a single shape.

---

## Practical Exercise: Modifying a Simple Model

- **Objective:** Modify a basic chair design using fillets and Boolean operations.
- **Steps:**
  - Use a box to create the seat.
  - Create four cylinders for the legs and position them appropriately.
  - Use the `Boolean Difference` to create cutouts in the seat for back support.
  - Apply `Fillet` to all sharp edges for a more realistic appearance.

---

## Lesson Assignment

Create a model of a simple desk using the techniques learned in this lesson. Incorporate Boolean operations to add drawers and modify their shapes to fit the desk's design. Use fillets to soften the edges of the desk.

---

## Additional Resources

- [Detailed Guide on Boolean Operations in FreeCAD](https://wiki.freecadweb.org/Boolean_Operation)
- [Tutorials on Modifying Objects](https://www.youtube.com/results?search_query=freecad+modifying+objects)

---
