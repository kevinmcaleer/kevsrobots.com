---
layout: lesson
title: Wheel Stubs
author: Kevin McAleer
type: page
cover: assets/cover.jpg
date: 2025-12-12
previous: 06_arduino_slots.html
next: 08_motor_holder.html
description: Create rotational features with Revolve and duplicate them with Polar
  Pattern
percent: 72
duration: 7
navigation:
- name: Building SMARS with FreeCAD
- content:
  - section: Getting Started
    content:
    - name: Introduction to Building SMARS with FreeCAD
      link: 00_intro.html
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
    - name: Summary and Next Steps
      link: 10_summary.html
---


## What We're Building

The wheel stubs are circular posts that connect to SMARS wheels. Each stub has:

- A **cylindrical shaft** that inserts into the wheel
- A **keyed slot** that prevents wheel spin (the wheel locks to the stub)

| Feature | Purpose |
|---------|---------|
| Circular profile | Creates a round stub when revolved |
| 135° chamfer | Easy wheel insertion |
| Keyed slot | Prevents wheel from spinning on shaft |
| Polar pattern | Creates both stubs from one design |
{: .table .table-single }


---

## Understanding Revolve and Polar Pattern

### The Revolve Tool

**Pad** pushes a shape straight out. **Revolve** spins a shape around an axis - perfect for round parts!

Think of a pottery wheel: your sketch is the profile, and revolving spins it into a 3D shape.

### Polar Pattern

Like Mirror creates a copy across a plane, **Polar Pattern** creates copies around a center point. Perfect for:
- Multiple holes in a circle (like bolt patterns)
- Symmetric features on opposite sides
- Any rotational repetition

---

## Step-by-Step: Create the Wheel Stub Profile

### 1. Start a Sketch on the Left Side Face

Click on the left face of the base, then click `Create Sketch`.

![Select bottom face](assets/wheel01.png){:class="img-fluid w-100"}

### 2. Project the Motor Holes

Using **External Geometry**, project both circular motor holes into the sketch.

![Project geometry](assets/wheel02.png){:class="img-fluid w-100"}

### 3. Create Reference Points

Using the **Point** tool, create a point at the bottom of each projected circle. The cursor shows a red indicator when you're at the tangent point.

![Add points](assets/wheel03.png){:class="img-fluid w-100"}

### 4. Draw a Vertical Reference Line

Using the **Line** tool, draw a vertical line between these two points.

Select the line and make it **construction geometry**.

![Draw line](assets/wheel04.png){:class="img-fluid w-100"}

![Make line vertical](assets/wheel05.png){:class="img-fluid w-100"}

### 5. Apply Vertical Constraint

Select the line and apply a **Vertical Constraint** to ensure it's perfectly vertical.

![Close sketch](assets/wheel06.png){:class="img-fluid w-100"}

### 6. Close the Sketch

Click the **Close** button.

![Close sketch](assets/wheel07.png){:class="img-fluid w-100"}

---

## Create a Datum Plane for the Stub

### 7. Create Datum Plane at Reference Point

Click **Create a datum plane** in the toolbar.

In the Data tab, set `Attachment mode` to `Translate Origin`, then click on the point at the bottom of the vertical line.

![Create datum plane](assets/wheel08.png){:class="img-fluid w-100"}

Click **Close**.

![Close datum plane](assets/wheel09.png){:class="img-fluid w-100"}

---

## Design the Stub Profile

### 8. Sketch on the Datum Plane

Select the datum plane and click `Create Sketch`.

![Select datum plane](assets/wheel10.png){:class="img-fluid w-100"}

### 9. Draw the Profile Shape

Create the profile shown below - this is half of the wheel stub cross-section.

![Draw wheel stub shape](assets/wheel11.png){:class="img-fluid w-100"}

Use **Horizontal** and **Vertical** constraints to align the shape properly.

### 10. Add Dimensions

Using the **Dimensions** tool, set the angle of the slanted line to `135 degrees` (this creates the chamfer for easy wheel insertion).

![Dimension wheel stub shape](assets/wheel12.png){:class="img-fluid w-100"}

### 11. Complete the Dimensions

Add the remaining dimensions as shown.

![Complete dimensions](assets/wheel13.png){:class="img-fluid w-100"}

---

## Align to Motor Hole

### 12. Project Motor Hole Geometry

Click **Create external geometry** and project the motor hole edges.

![Project geometry](assets/wheel14.png){:class="img-fluid w-100"}

You may need to rotate the view slightly to select the circular edges.

Press `3` to return to the side view.

Also project the edge of the base.

![Select top edges](assets/wheel15.png){:class="img-fluid w-100"}

### 13. Create Symmetry Reference

Draw a horizontal construction line roughly in the middle of the motor hole area.

Using **Constrain Symmetric**, select the line and the two points from the projected motor hole geometry to center it.

![Select top edges](assets/wheel16.png){:class="img-fluid w-100"}

### 14. Position the Stub

Set the distance from the stub profile's top edge to the motor hole edge at `8mm`.

Use **Constrain Coincident** to attach the top right point of the profile to the base edge.

![Constrain top edge](assets/wheel17.png){:class="img-fluid w-100"}

### 15. Close the Sketch

Click **Close**.

![Close sketch](assets/wheel18.png){:class="img-fluid w-100"}

---

## Revolve the Stub

### 16. Apply Revolve

Select the sketch and click the **Revolve** button.

Set:
- Angle: `360 degrees` (full rotation)
- Axis: Select the appropriate edge reference

Click `OK`.

![Revolve the sketch](assets/wheel19.png){:class="img-fluid w-100"}

**Note**: The axis edge number may vary in your model - select the vertical edge you created as the axis.

---

## Merge with Base

### 17. Pad into Base

Hide the datum plane. Select the flat face of the wheel stub.

![Select flat face](assets/wheel20.png){:class="img-fluid w-100"}

### 18. Extrude Inward

Use the **Pad** tool with length `2mm` to merge the stub into the base wall.

![Pad the wheel stub](assets/wheel21.png){:class="img-fluid w-100"}

Click **Close**.

![close stub](assets/wheel22.png){:class="img-fluid w-100"}

Press `0` to return to isometric view.

![Default view](assets/wheel23.png){:class="img-fluid w-100"}

---

## Add the Key Slot

### 19. Sketch on Stub Face

Create a new sketch on the flat end of the wheel stub.

![Select flat face](assets/wheel24.png){:class="img-fluid w-100"}

### 20. Draw Key Rectangle

Draw a rectangle for the keyed slot.

![Draw rectangle](assets/wheel25.png){:class="img-fluid w-100"}

### 21. Dimension the Key

Set the rectangle width to `1mm`.

![Dimension rectangle](assets/wheel26.png){:class="img-fluid w-100"}

### 22. Position the Key

Project the inner motor hole circle.

Add a midpoint to the top of the rectangle (watch for the `> <` cursor).

![Project geometry](assets/wheel27.png){:class="img-fluid w-100"}

### 23. Add Reference Line

Draw a construction line from the rectangle midpoint to the bottom of the projected circle.

![Draw line](assets/wheel28.png){:class="img-fluid w-100"}

### 24. Constrain to Outer Edge

Project the outer circular edge of the stub.

Use **Constrain Tangent** to make the top and bottom edges of the rectangle tangent to this circle.

![Constrain tangent](assets/wheel29.png){:class="img-fluid w-100"}

![Constrain tangent](assets/wheel30.png){:class="img-fluid w-100"}

![Constrain tangent](assets/wheel31.png){:class="img-fluid w-100"}

### 25. Close and Pocket

Close the sketch.

![Close sketch](assets/wheel32.png){:class="img-fluid w-100"}

Select the sketch and use **Pocket** with depth `10mm`.

![Pocket the sketch](assets/wheel33.png){:class="img-fluid w-100"}

---

## Duplicate with Polar Pattern

### 26. Select Features to Pattern

Select the Pocket in the Model tree.

![Select wheel stub](assets/wheel34.png){:class="img-fluid w-100"}

### 27. Apply Polar Pattern

Click the **Polar Pattern** tool.

Set:
- Axis: `Z Axis`
- Number of occurrences: `2`
- Enable **Symmetric** checkbox

Click `OK`.

![Select wheel stub](assets/wheel35.png){:class="img-fluid w-100"}

**Note**: You may need to create additional Polar Patterns for the Revolve and Pad features if they don't copy automatically.

![Select wheel stub](assets/wheel36.png){:class="img-fluid w-100"}

---

## Understanding the Design Choices

| Feature | Reason |
|---------|--------|
| 135° chamfer | Makes wheel insertion easy |
| Key slot | Prevents wheel from spinning |
| Tangent constraints | Key extends to exact edge |
| Polar Pattern | Ensures both stubs are identical |

---

## Try It Yourself

1. **Inspect the stub**: Rotate to see the keyed slot - would a matching wheel lock onto it?
2. **Measure**: Use View → Measure to verify the stub dimensions
3. **Edit test**: Change the revolve angle to 270° - what shape results?

---

## Common Issues

### "Revolve creates wrong shape"
**Problem**: The revolved shape doesn't look like a stub.
**Solution**: Check your revolve axis. It should be the vertical edge, not a horizontal one. Also verify your profile is on the correct side of the axis.

### "Polar Pattern only copies some features"
**Problem**: The stub appears but the key slot doesn't.
**Solution**: Apply separate Polar Patterns to each feature (Revolve, Pad, Pocket) if needed.

### "Tangent constraint fails"
**Problem**: Can't make rectangle tangent to circle.
**Solution**: The rectangle edges must be able to touch the circle. If it's too big or small, adjust dimensions first.

### "Key slot is in wrong position"
**Problem**: The keyed slot isn't centered or is rotated wrong.
**Solution**: Verify the construction line from midpoint to circle center is vertical.

---

## What You Learned

In this lesson, you mastered:

- **Revolve** - Creating rotational 3D shapes from profiles
- **Datum planes** - Positioning sketches at specific locations
- **Polar Pattern** - Duplicating features around a center axis
- **Tangent constraints** - Positioning elements to touch circles precisely

---

## Next Up

Time to add the motor holder tabs - the final structural feature! Then we'll export for 3D printing.

---
