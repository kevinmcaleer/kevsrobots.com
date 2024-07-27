---
title: Adding and Managing Components
description: Learn how to add and manage components in your KiCad schematic.
layout: lesson
type: page
cover: assets/cover_adding_components.png
---

![Cover](assets/cover_adding_components.png){:class="cover"}

## Adding and Managing Components

In this lesson, we will focus on adding and managing components in your KiCad schematic. Understanding how to efficiently work with components is essential for creating accurate and reliable circuit diagrams.

---

### Adding Components to Your Schematic

1. **Open the Component Library**:
   - Press the `A` key or click on the “Place Symbol” icon to open the component library.

2. **Search for Components**:
   - Use the search bar to find specific components. For example, search for "capacitor" to find a capacitor component.

3. **Place Components**:
   - Select the desired component and click on the schematic canvas to place it. Repeat this process to add all the necessary components for your circuit.

4. **Move and Rotate Components**:
   - To move a component, click and drag it to the desired location. To rotate a component, press the `R` key while the component is selected.

### Managing Component Libraries

1. **Open Library Manager**:
   - Go to `Preferences > Manage Symbol Libraries` to open the Library Manager.

2. **Add New Libraries**:
   - Click the `Browse Libraries` button to add new component libraries from your computer or the internet.

3. **Organize Libraries**:
   - Use the Library Manager to organize and prioritize libraries, ensuring that frequently used components are easily accessible.

### Editing Component Properties

1. **Select a Component**:
   - Click on a component to select it.

2. **Edit Properties**:
   - Press the `E` key or right-click and select `Properties` to edit the component’s attributes, such as its value, reference designator, and footprint.

3. **Update Multiple Components**:
   - To update properties for multiple components simultaneously, use the `Edit > Edit Text & Graphics Properties` menu.

### Creating Custom Components

1. **Open Symbol Editor**:
   - Go to `Tools > Symbol Editor` to create custom components.

2. **Design the Symbol**:
   - Use the drawing tools to create the graphical representation of your custom component. Add pins and labels as needed.

3. **Save Custom Component**:
   - Save your custom component to a personal library by selecting `File > Save As` and choosing an appropriate library.

### Assigning Footprints to Components

1. **Open Footprint Assignment Tool**:
   - Click on the “Assign Footprints” icon in the Schematic Editor toolbar.

2. **Assign Footprints**:
   - For each component, select an appropriate footprint from the library. This defines the physical layout of the component on the PCB.

3. **Verify Footprint Assignments**:
   - Ensure that each component has a correctly assigned footprint to avoid issues during PCB layout.

### Using Component References and Values

1. **Reference Designators**:
   - Each component should have a unique reference designator (e.g., R1, C1, U1). Use the annotation tool to assign these automatically.

2. **Component Values**:
   - Enter the correct value for each component (e.g., 10k for a resistor, 100nF for a capacitor) to ensure the schematic is accurate and understandable.

### Organizing Your Schematic

1. **Group Related Components**:
   - Group related components together to make your schematic easier to read and understand.

2. **Use Labels and Notes**:
   - Add labels and notes to clarify the function of different sections of your circuit.

3. **Maintain Neatness**:
   - Keep wires straight and components aligned to maintain a clean and professional-looking schematic.

---

## Summary

In this lesson, you have learned how to add and manage components in your KiCad schematic. You now know how to search for and place components, manage component libraries, edit component properties, create custom components, assign footprints, and organize your schematic for clarity.

Next, we will move on to designing the PCB layout. Ready to continue? Let's dive into the [PCB Editor Basics](06_pcb_editor_basics.md)!

---

![KiCad](assets/kicad_logo.png){:class="kicad-logo"}

---
