---
title: PCB Editor Basics
description: Discover the basics of the PCB editor and how to transfer your schematic to the PCB layout.
layout: lesson
type: page
cover: assets/cover_pcb_editor_basics.png
---

![Cover](assets/cover_pcb_editor_basics.png){:class="cover"}

## PCB Editor Basics

In this lesson, we will explore the PCB Editor in KiCad. You will learn how to transfer your schematic to the PCB layout and begin designing the physical layout of your printed circuit board.

---

### Transferring Schematic to PCB Layout

1. **Generate Netlist from Schematic**:
   - Open your schematic in the Schematic Editor.
   - Click on `Generate Netlist` (icon looks like a document with a net symbol).
   - Save the netlist file.

2. **Open PCB Editor (Pcbnew)**:
   - From the KiCad main window, click on the PCB Editor icon or select `File > New Board` to open Pcbnew.

3. **Import Netlist**:
   - In the PCB Editor, click on `Tools > Update PCB from Schematic`.
   - Select the netlist file you generated and click `Update PCB`.
   - Components from your schematic will now appear on the PCB layout.

### Placing Components

1. **Arrange Components**:
   - Click and drag components to arrange them on the PCB canvas. Consider the physical layout and connections.

2. **Rotate Components**:
   - Select a component and press the `R` key to rotate it. Ensure that component orientation matches your design requirements.

3. **Align Components**:
   - Use the alignment tools to neatly arrange components. This helps in making a clear and professional layout.

### Creating the Board Outline

1. **Select Edge.Cuts Layer**:
   - In the layers manager on the right, select the `Edge.Cuts` layer. This layer is used to define the physical outline of your PCB.

2. **Draw Board Outline**:
   - Use the drawing tools to create the outline of your PCB. Click on the `Add Graphic Line` tool and draw the shape of your board.

### Routing Traces

1. **Select the Route Tool**:
   - Click on the `Route Tracks` tool (icon looks like a pen with a trace).

2. **Route Traces**:
   - Click on a pad of a component and draw the trace to the corresponding pad of another component. Ensure that traces are neat and avoid crossing each other.

3. **Use Different Layers**:
   - For more complex boards, use different layers for routing. Switch layers using the PgUp/PgDn keys. Typical boards use the front layer (F.Cu) and the back layer (B.Cu).

### Adding Vias

1. **Select the Via Tool**:
   - While routing a trace, press the `V` key to place a via. This allows the trace to jump between layers.

2. **Route on Different Layers**:
   - After placing a via, continue routing on the new layer.

### Managing Design Rules

1. **Set Design Rules**:
   - Go to `Design Rules > Design Rules Manager` to set the spacing and size rules for your PCB. This ensures manufacturability and electrical performance.

2. **Run Design Rule Check (DRC)**:
   - Click on `Tools > Design Rule Check` to validate your PCB layout. Fix any errors or violations that are reported.

### Adding Silkscreen

1. **Add Text and Graphics**:
   - Use the `Add Text` and `Add Graphic Line` tools to place text and graphics on the silkscreen layer (F.SilkS or B.SilkS). This includes labels, logos, and other markings.

2. **Position Silkscreen Elements**:
   - Ensure that silkscreen elements do not overlap with pads or vias.

### Saving and Reviewing Your PCB

1. **Save Your Work**:
   - Regularly save your PCB layout by clicking `File > Save`.

2. **Review the Layout**:
   - Use the 3D viewer (`View > 3D Viewer`) to inspect your PCB in three dimensions. This helps in verifying component placement and overall board design.

---

## Summary

In this lesson, you have learned the basics of using the PCB Editor in KiCad. You now know how to transfer your schematic to the PCB layout, place and arrange components, create the board outline, route traces, manage design rules, and add silkscreen elements.

Next, we will explore routing techniques and best practices in more detail. Ready to continue? Let's move on to [Routing and Placement](07_routing_placement.md)!

---

![KiCad](assets/kicad_logo.png){:class="kicad-logo"}

---
