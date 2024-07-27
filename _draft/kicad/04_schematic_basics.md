---
title: Schematic Editor Basics
description: Understand how to use the schematic editor to create circuit diagrams.
layout: lesson
type: page
cover: assets/cover_schematic_basics.png
---

![Cover](assets/cover_schematic_basics.png){:class="cover"}

## Schematic Editor Basics

In this lesson, we will delve into the Schematic Editor in KiCad, where you create and edit circuit diagrams. This is a crucial step in the PCB design process, as the schematic represents the electrical connections between components.

---

### Creating a New Schematic

1. **Open the Schematic Editor**:
   - From the KiCad main window, click on the Schematic Editor icon or select `File > New Schematic`.

2. **Save Your Schematic**:
   - Save your schematic by clicking `File > Save` and giving it a meaningful name. This ensures you don't lose your work.

### Adding Components

1. **Open the Component Library**:
   - Press the `A` key or click on the “Place Symbol” icon to open the component library.

2. **Search for Components**:
   - Use the search bar to find the component you need. For example, search for "resistor" to find a resistor component.

3. **Place Components on the Schematic**:
   - Select the component and click on the schematic canvas to place it. Repeat this process to add all necessary components.

### Connecting Components with Wires

1. **Select the Wire Tool**:
   - Press the `W` key or click on the “Place Wire” icon.

2. **Draw Wires Between Components**:
   - Click on a component pin and drag to the pin of another component to create a wire. Repeat to connect all components as needed.

### Editing Component Properties

1. **Select a Component**:
   - Click on a component to select it.

2. **Edit Properties**:
   - Press the `E` key or right-click and select `Properties` to edit the component’s properties, such as its value, reference, and footprint.

### Adding Power and Ground Symbols

1. **Place Power Symbols**:
   - Press the `A` key, search for "power", and select the appropriate power symbol (e.g., VCC). Place it on the schematic.

2. **Place Ground Symbols**:
   - Similarly, search for "ground" and place the ground symbol on the schematic.

### Annotating the Schematic

1. **Open the Annotate Tool**:
   - Click on the “Annotate Schematic” icon in the top toolbar.

2. **Annotate Components**:
   - Use the annotation tool to automatically assign unique references to each component (e.g., R1, C1, U1).

### Electrical Rules Check (ERC)

1. **Open the ERC Tool**:
   - Click on the “Perform Electrical Rules Check” icon.

2. **Run ERC**:
   - Run the ERC to identify any electrical issues in your schematic, such as unconnected pins or conflicting connections. Review and fix any errors.

### Saving and Organizing Your Schematic

1. **Save Your Work**:
   - Regularly save your schematic by clicking `File > Save`.

2. **Organize Your Schematic**:
   - Keep your schematic organized by arranging components neatly and using labels and notes for clarity.

---

## Summary

In this lesson, you have learned the basics of creating schematic diagrams in KiCad’s Schematic Editor. You now know how to add and connect components, edit their properties, and perform an electrical rules check to ensure your schematic is correct. 

Next, we will dive deeper into adding and managing components in your schematic. Ready to continue? Let's move on to [Adding and Managing Components](05_adding_components.md)!

---

![KiCad](assets/kicad_logo.png){:class="kicad-logo"}

---
