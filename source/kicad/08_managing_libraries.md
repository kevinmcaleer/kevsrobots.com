---
title: Managing Libraries
description: Explore how to manage libraries and create custom footprints in KiCad.
layout: lesson
type: page
cover: assets/cover_managing_libraries.png
---

![Cover](assets/cover_managing_libraries.png){:class="cover"}

## Managing Libraries

In this lesson, we will focus on managing component libraries in KiCad. Efficient library management is crucial for keeping your components organized and ensuring that you have the necessary parts available for your designs.

---

### Understanding KiCad Libraries

1. **Symbol Libraries**:
   - These libraries contain schematic symbols for components, such as resistors, capacitors, and ICs.

2. **Footprint Libraries**:
   - These libraries contain footprints, which are the physical representations of components on the PCB.

3. **3D Model Libraries**:
   - These libraries contain 3D models used for visualizing the PCB in three dimensions.

### Adding and Managing Symbol Libraries

1. **Open the Library Manager**:
   - Go to `Preferences > Manage Symbol Libraries` to open the Library Manager.

2. **Add New Libraries**:
   - Click the `Browse Libraries` button to add new symbol libraries from your computer or the internet. Select the desired library files and click `Open`.

3. **Organize Libraries**:
   - Use the Library Manager to organize and prioritize your symbol libraries. You can enable or disable libraries as needed.

4. **Updating Libraries**:
   - Regularly update your libraries to ensure you have the latest components. This can be done by downloading updated library files from the KiCad website or other sources.

### Creating Custom Symbol Libraries

1. **Open Symbol Editor**:
   - Go to `Tools > Symbol Editor` to create or edit symbols.

2. **Create a New Library**:
   - In the Symbol Editor, click `File > New Library` and choose a location to save your new library.

3. **Create a New Symbol**:
   - Click `File > New Symbol` and enter a name for your new symbol. Use the drawing tools to create the symbol and add pins.

4. **Save Your Symbol**:
   - Save your new symbol to the custom library you created.

### Adding and Managing Footprint Libraries

1. **Open the Footprint Library Manager**:
   - Go to `Preferences > Manage Footprint Libraries` to open the Footprint Library Manager.

2. **Add New Libraries**:
   - Click the `Browse Libraries` button to add new footprint libraries from your computer or the internet. Select the desired library files and click `Open`.

3. **Organize Libraries**:
   - Use the Footprint Library Manager to organize and prioritize your footprint libraries.

4. **Updating Libraries**:
   - Regularly update your footprint libraries to ensure you have the latest components.

### Creating Custom Footprints

1. **Open Footprint Editor**:
   - Go to `Tools > Footprint Editor` to create or edit footprints.

2. **Create a New Library**:
   - In the Footprint Editor, click `File > New Library` and choose a location to save your new library.

3. **Create a New Footprint**:
   - Click `File > New Footprint` and enter a name for your new footprint. Use the drawing tools to create the footprint, including pads, outlines, and silkscreen markings.

4. **Save Your Footprint**:
   - Save your new footprint to the custom library you created.

### Using Libraries in Your Design

1. **Assigning Symbols and Footprints**:
   - In the Schematic Editor, assign symbols and footprints to your components. Ensure that each component has the correct footprint for the PCB layout.

2. **Library References in Netlist**:
   - When generating the netlist, ensure that the library references are correct to avoid issues during the PCB layout process.

### Best Practices for Library Management

1. **Regular Backups**:
   - Regularly back up your custom libraries to prevent data loss.

2. **Consistent Naming Conventions**:
   - Use consistent naming conventions for your libraries, symbols, and footprints to make them easy to search and organize.

3. **Document Custom Components**:
   - Keep documentation for custom components, including datasheets and usage notes, to ensure proper use in designs.

---

## Summary

In this lesson, you have learned how to manage libraries in KiCad, including adding and organizing symbol and footprint libraries, creating custom symbols and footprints, and best practices for library management.

Next, we will explore creating and using custom footprints in more detail. Ready to continue? Let's move on to [Creating Custom Footprints](09_custom_footprints.md)!

---

![KiCad](assets/kicad_logo.png){:class="kicad-logo"}
