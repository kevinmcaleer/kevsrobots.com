---
title: Exporting and Sharing Your FreeCAD Models
description: Learn how to prepare and share your FreeCAD models through various platforms, including GitHub Pages.
layout: lesson
type: page
cover: assets/9.png
---

![Exporting FreeCAD Models]({{ page.cover }}){:class="cover"}

## Introduction to Exporting and Sharing

Successfully exporting and sharing your FreeCAD models is essential for collaboration and presentation. This lesson will explain the process of exporting models in different formats suitable for various uses, such as 3D printing, rendering, or viewing online.

---

## Key Export Formats

1. **STL and OBJ:**
   - Commonly used for 3D printing. These formats focus on the model's surface geometry.

2. **Step and IGES:**
   - Suitable for professional engineering, where precise geometry (including internal structures) is necessary.

3. **SVG and PDF:**
   - Best for creating documentation or promotional materials, as they are ideal for 2D representations.

---

## Preparing Your Model for Export

1. **Check Geometry:**
   - Ensure there are no errors in your model, such as non-manifold edges or reversed normals, which could affect the export.

2. **Simplify Design:**
   - Reduce complexity where possible to minimize file size and improve compatibility with other software.

3. **Apply Materials and Textures:**
   - If the export format supports it, add materials or textures to enhance the appearance of your model in renderings or visualizations.

---

## Steps to Export a Model

### Example Project: Exporting a Mechanical Part for Online Viewing

- **Component:** A customized gear.

---

### Export Process

1. **Select Export Format:**
   - Choose a format that supports online viewing, such as WebGL or 3MF.

2. **Export the Model:**
   - Use FreeCAD's export function to save your model. Ensure all settings match the requirements of the format you've chosen.

3. **Verify the Export:**
   - Open the exported file in another software or viewer to ensure it appears as intended.

---

## Practical Exercise: Exporting for Different Applications

- **Objective:** Prepare and export a previously designed assembly for both 3D printing and online viewing.
- **Steps:**
  - Export the assembly in STL for 3D printing.
  - Export the same assembly in WebGL for online viewing.

---

## Lesson Assignment

Choose a model you have created earlier in this course. Prepare it for export by performing a thorough geometry check and simplifying the design as needed. Export it in two different formats: one for 3D printing (STL) and one for online sharing (WebGL or 3MF). Test the exports to ensure they meet your standards.

---

## Additional Resources

- [Guide to Exporting Models in FreeCAD](https://wiki.freecadweb.org/Export_to_other_file_formats)
- [Tips for Preparing Models for 3D Printing](https://www.freecadweb.org/wiki/3D_printing)

---
