---
title: Modifying Objects in FreeCAD
description: Learn advanced techniques to modify and manipulate objects to create complex designs in FreeCAD.
layout: lesson
type: page
cover: assets/4.png
date_updated: 2026-11-24
---

![Modifying Objects in FreeCAD]({{ page.cover }}){:class="cover"}

By the end of this lesson you will be able to position shapes precisely with Placement and soften edges with fillets and chamfers.

## Introduction to Modifying Objects

This lesson builds on the previous one by exploring the tools for editing existing shapes. Tool names below follow the Part workbench; they can differ slightly between FreeCAD versions.

---

## Key Modification Tools

1. **Placement:**
   - Use the `Placement` property to position and rotate objects precisely. Find it in the property editor, in the Data tab. It sets position and rotation; to change size, edit the shape's own dimensions.

2. **Boolean Operations:**
   - **Union (Fuse):** Combine two or more shapes into one.
   - **Difference (Cut):** Subtract one shape from another.
   - **Intersection (Common):** Create a shape from the overlapping volume of two shapes.

3. **Fillet and Chamfer:**
   - **Fillet:** Round the edges of an object.
   - **Chamfer:** Create a beveled edge on one or more edges of an object.

---

## Using Boolean Operations

- **Task:** Combine a cylinder and a cube to form a T-shaped object.
- **Steps:**
  - Create a cube and a cylinder.
  - Position the cylinder so that it intersects part of the cube.
  - Select both objects (hold Ctrl while clicking), and use the `Union` (Fuse) operation to fuse them into a single shape.

---

## Practical Exercise: Modifying a Simple Model

- **Objective:** Modify a basic chair design using fillets and Boolean operations.
- **Steps:**
  - Use a box to create the seat.
  - Create four cylinders for the legs and position them appropriately.
  - Use `Difference` (Cut) to create cutouts in the seat for back support.
  - Apply `Fillet` to all sharp edges for a more realistic appearance.

---

## Try it Yourself

1. Make a box and use Placement to move it 20 mm along the X axis.
2. Fuse a box and a cylinder into a T shape, then fillet one edge.
3. Challenge: make a rounded robot body from a box, chamfer the top edges and fillet the vertical ones.

---

## Common Issues

- **Problem**: The Fillet fails or shows an error.
- **Solution**: Try a smaller radius.
- **Why**: A fillet cannot be larger than the faces it rounds.

- **Problem**: Union did nothing visible.
- **Solution**: Make sure the shapes really touch or overlap.
- **Why**: Fusing two separate shapes does not join them.

---

## Check your understanding

{% quiz %}
? Which operation rounds the edges of an object?
- [ ] Chamfer
- [x] Fillet
- [ ] Common
> A fillet rounds an edge; a chamfer bevels it.

? How do you fuse two objects into one?
- [ ] Select one object only
- [x] Select both objects and use Union (Fuse)
- [ ] Rotate them
> Select both, then apply the operation.

? Does the Placement property change an object's size?
- [x] No, it sets position and rotation
- [ ] Yes, it scales it
- [ ] It deletes the object
> Size comes from the shape's own dimensions.
{% endquiz %}

---

## Additional Resources

- [Detailed Guide on Boolean Operations in FreeCAD](https://wiki.freecad.org/Part_Boolean)
- [Tutorials on Modifying Objects](https://www.youtube.com/results?search_query=freecad+modifying+objects)
