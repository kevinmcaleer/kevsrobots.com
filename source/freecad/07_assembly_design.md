---
title: Assembly Design in FreeCAD
description: Learn how to use FreeCAD's Assembly workbench to combine parts into functional assemblies using constraints.
layout: lesson
type: page
cover: assets/6.png
date_updated: 2026-11-24
---

![Assembly Design in FreeCAD]({{ page.cover }}){:class="cover"}

By the end of this lesson you will be able to explain what an assembly is and describe the steps for joining parts together.

> **Teachers**: Assembly tools changed a lot in FreeCAD 1.0 (late 2024), and older school installs may have a different version or rely on add-on assembly tools. Check the version on your machines first, and treat this lesson as the overview of an assembly workflow. Names of tools and joints below may not match older versions exactly.

## Introduction to Assembly Design

Assembly design is where you combine multiple parts into a single functional unit, such as a wheel that really turns on an axle. This lesson covers the basics of using the Assembly workbench to connect parts so they move together as intended. Tool and joint names differ between FreeCAD versions, so look for the same idea in your version.

```mermaid
flowchart LR
    A[Design each part] --> B[Bring parts into the assembly]
    B --> C[Fix one part in place]
    C --> D[Join other parts to it]
    D --> E[Move and test]
    E -->|Collision or gap| D
```

---

## Using the Assembly Workbench

1. **Overview:**
   - The Assembly workbench in FreeCAD allows you to create mechanical assemblies by defining relationships (joints or constraints) between separate parts.

2. **Key Concepts:**
   - **Parts:** Independent models created in other workbenches.
   - **Constraints / joints:** Rules that define how parts align and move relative to each other. Examples are fixing a part in place, letting a part rotate like a wheel, or letting it slide like a drawer.

---

## Steps to Create an Assembly

### Example Project: Constructing a Swivel Chair

- **Components:**
  - Base, seat, backrest, and wheels.

---

### Assembly Process

1. **Import Parts:**
   - Start by bringing your previously designed components into the assembly.

2. **Add Constraints:**
   - Use alignment, distance and rotation relationships to position parts relative to each other. Fix one part first (such as the base) so everything else is placed relative to it.

3. **Check Movements:**
   - Try moving the parts to make sure they operate without interference, adjusting constraints as necessary.

---

## Practical Exercise: Assembling a Simple Engine

- **Objective:** Use the Assembly workbench to construct a simple piston engine.
- **Steps:**
  - Import parts such as the crankshaft, pistons, and cylinder block.
  - Apply alignment and rotation relationships so the pistons sit correctly in the cylinder as the crankshaft rotates.

---

## Try it Yourself

1. Model two parts, a plate with a hole and a cylinder pin, and save them.
2. Bring both into an assembly and fix the plate in place.
3. Challenge: join the pin so it can spin freely in the hole, then test it.

---

## Common Issues

- **Problem**: Parts jump to strange places when I add a relationship.
- **Solution**: Fix one part first and add relationships one at a time.
- **Why**: Over-constrained or unconstrained parts leave the solver guessing.

- **Problem**: I cannot find the Assembly workbench.
- **Solution**: Update FreeCAD, or check for an add-on assembly workbench.
- **Why**: The built-in assembly tools depend on the FreeCAD version.

---

## Check your understanding

{% quiz %}
? Why do you fix one part first in an assembly?
- [x] So everything else is positioned relative to it
- [ ] To save file size
- [ ] To make it invisible
> A fixed part is the anchor the others are placed against.

? What do constraints (joints) define?
- [ ] The colour of parts
- [x] How parts align and move relative to each other
- [ ] The printer settings
> They are rules for alignment and movement.

? What should you do if parts jump to odd places?
- [ ] Delete all parts
- [x] Add relationships one at a time and check each
- [ ] Rotate the view
> Adding them one at a time shows which one caused the problem.
{% endquiz %}

---

## Additional Resources

- [FreeCAD Assembly Workbench Guide](https://wiki.freecad.org/Assembly_Workbench)
- [Interactive Tutorials on Assembly Design](https://www.youtube.com/results?search_query=freecad+assembly+tutorial)
