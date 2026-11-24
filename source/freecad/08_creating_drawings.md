---
title: Creating Technical Drawings in FreeCAD
description: Learn how to use the TechDraw workbench in FreeCAD to produce precise technical drawings from your 3D models.
layout: lesson
type: page
cover: assets/7.png
date_updated: 2026-11-24
---

![Technical Drawings in FreeCAD]({{ page.cover }}){:class="cover"}

By the end of this lesson you will be able to describe how a 3D part becomes a technical drawing with views and dimensions.

## Introduction to Technical Drawings

Technical drawings are essential for communicating mechanical details in a clear and standard format, for example when someone else needs to make your part. This lesson introduces the TechDraw workbench in FreeCAD, which creates drawings from your 3D models. Menu and button names can differ between versions.

---

## Using the TechDraw Workbench

1. **Overview:**
   - The TechDraw workbench is designed to produce drawings of your models on a page, suitable for manufacturing.

2. **Key Features:**
   - **Views and Projections:** Generate top, front, side, and isometric views of your model.
   - **Annotations:** Add dimensions, labels, and other text annotations to clarify details.
   - **Section and detail views:** Show a cut-through view to reveal internal features, or zoom in on a small area.

---

## Steps to Create a Technical Drawing

### Example Project: Drafting a Mechanical Part

- **Component:** A simple mechanical bracket.

---

### Drawing Process

1. **Create New Drawing:**
   - Open the TechDraw workbench and add a new page. Choose a standard sheet size (for example A4).

2. **Add Views:**
   - Select your 3D part, then insert principal views of the bracket onto the page. Adjust scale and orientation for clarity.

3. **Annotate:**
   - Use the dimensioning tools to add necessary measurements. Label important features.

4. **Detail and Section Views:**
   - Include a detailed view of complex areas for better understanding. Add a section view to show the bracket's profile.

---

## Practical Exercise: Drawing a Gear Assembly

- **Objective:** Produce a set of technical drawings for a gear assembly.
- **Steps:**
  - Insert different views of the assembly into a drawing.
  - Dimension the critical features, including spacing.
  - Add annotations to specify materials and finish.

---

## Try it Yourself

1. Make a simple box with a hole and insert a front view of it on an A4 page.
2. Add dimensions for the box's length and height.
3. Challenge: add a top view and a section view, and check the dimensions match your 3D model.

---

## Common Issues

- **Problem**: My view appears but is tiny or runs off the page.
- **Solution**: Change the view's scale property.
- **Why**: The scale decides how big the part appears on the sheet.

- **Problem**: The page is blank.
- **Solution**: Select the 3D object first, then insert the view.
- **Why**: TechDraw needs to know which object to project.

---

## Check your understanding

{% quiz %}
? What does TechDraw produce?
- [ ] 3D prints
- [x] Technical drawings from your 3D models
- [ ] Animations
> It creates drawings suited to manufacturing.

? What is a section view for?
- [x] Showing the inside of a part
- [ ] Changing the colour
- [ ] Exporting an STL
> A section view reveals internal features.

? What should you do before inserting a view?
- [ ] Close the 3D model
- [x] Select the 3D object
- [ ] Rename the page
> TechDraw needs a selected object to project.
{% endquiz %}

---

## Additional Resources

- [FreeCAD TechDraw Documentation](https://wiki.freecad.org/TechDraw_Workbench)
- [Tutorial Videos on Creating Technical Drawings](https://www.youtube.com/results?search_query=freecad+techdraw+tutorial)
