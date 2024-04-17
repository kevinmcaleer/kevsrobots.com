---
layout: lesson
title: Assembly Design in FreeCAD
author: Kevin McAleer
type: page
cover: assets/6.png
date: 2024-04-17
previous: 06_boolean_operations.html
next: 08_creating_drawings.html
description: Learn how to use FreeCAD's Assembly workbench to combine parts into functional
  assemblies using constraints.
percent: 63
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


![Assembly Design in FreeCAD]({{ page.cover }}){:class="cover"}

## Introduction to Assembly Design

Assembly design is a crucial phase in CAD modeling where you combine multiple parts into a single functional unit. This lesson covers the basics of using the Assembly workbench in FreeCAD to connect parts with constraints, ensuring they move together as intended.

---

## Using the Assembly Workbench

1. **Overview:**
   - The Assembly workbench in FreeCAD allows you to create mechanical assemblies by defining relationships (constraints) between separate parts.

2. **Key Concepts:**
   - **Parts:** Independent models created in other workbenches.
   - **Constraints:** Rules that define how parts align and move relative to each other.

---

## Steps to Create an Assembly

### Example Project: Constructing a Swivel Chair

- **Components:**
  - Base, seat, backrest, and wheels.

---

### Assembly Process

1. **Import Parts:**
   - Start by importing your previously designed components into the Assembly workbench.

2. **Add Constraints:**
   - Use constraints like `axial align`, `angular`, `coincident`, and `distance` to position parts relative to each other.
   - For example, align the wheels symmetrically around the base using `circular pattern` constraints.

3. **Check Movements:**
   - Simulate movements to ensure parts operate without interference, adjusting constraints as necessary.

---

## Practical Exercise: Assembling a Simple Engine

- **Objective:** Use the Assembly workbench to construct a simple piston engine.
- **Steps:**
  - Import parts such as the crankshaft, pistons, and cylinder block.
  - Apply `axial` and `rotational` constraints to ensure the pistons move correctly within the cylinder as the crankshaft rotates.

---

## Lesson Assignment

Create an assembly of a small gearbox. Design or import gears, a casing, and a shaft. Use constraints to assemble the gearbox, ensuring that gears mesh correctly and the shaft rotates freely within the casing.

---

## Additional Resources

- [FreeCAD Assembly Workbench Guide](https://wiki.freecadweb.org/Assembly_Workbench)
- [Interactive Tutorials on Assembly Design](https://www.youtube.com/results?search_query=freecad+assembly+tutorial)

---
