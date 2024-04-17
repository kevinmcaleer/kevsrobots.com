---
title: Assembly Design in FreeCAD
description: Learn how to use FreeCAD's Assembly workbench to combine parts into functional assemblies using constraints.
layout: lesson
type: page
cover: assets/6.png
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
