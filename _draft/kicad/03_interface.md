---
title: Exploring the KiCad Interface
description: Get acquainted with the KiCad interface and its various tools.
layout: lesson
type: page
cover: assets/cover_interface.png
---

![Cover](assets/cover_interface.png){:class="cover"}

## Exploring the KiCad Interface

Now that you have KiCad installed and set up, it’s time to explore the interface. In this lesson, we will guide you through the main components of the KiCad interface, helping you become familiar with the tools you will use to design your PCBs.

---

### KiCad Main Window

When you open KiCad, you are greeted by the main window, which serves as the central hub for your projects. Here’s a breakdown of the key elements:

- **Project Manager**: This is where you manage your KiCad projects. You can create new projects, open existing ones, and access project-specific tools.
- **Toolbar**: Located at the top of the window, the toolbar provides quick access to common functions such as creating a new project, opening an existing project, and configuring settings.
- **Project Files Panel**: This panel displays the files associated with your current project, such as schematics, PCB layouts, and footprints.
- **Message Panel**: At the bottom, this panel shows status messages, errors, and other notifications.

![KiCad Main Window](assets/kicad_main_window.png){:class="screenshot"}

### Key Tools in KiCad

#### 1. Project Manager

The Project Manager is your starting point for all PCB design activities in KiCad. It allows you to create, open, and manage your projects. 

- **Create a New Project**:
  - Click on `File > New Project` and provide a name and location for your project.

- **Open an Existing Project**:
  - Click on `File > Open Project` and navigate to the location of your existing project.

#### 2. Schematic Editor (Eeschema)

The Schematic Editor is where you create and edit circuit schematics. 

- **Creating a New Schematic**:
  - Click on the Schematic Editor icon in the Project Manager or go to `File > New Schematic`.

- **Adding Components**:
  - Use the component library to add parts to your schematic. You can search for components, place them on the schematic, and connect them with wires.

- **Editing Components**:
  - Select components to move, rotate, or edit their properties. 

![Schematic Editor](assets/schematic_editor.png){:class="screenshot"}

#### 3. PCB Editor (Pcbnew)

The PCB Editor is where you design the physical layout of your PCB.

- **Creating a New PCB Layout**:
  - Click on the PCB Editor icon in the Project Manager or go to `File > New Board`.

- **Placing Components**:
  - Transfer your schematic components to the PCB layout and arrange them on the board.

- **Routing Traces**:
  - Use the routing tools to connect the components with traces, defining the electrical paths on the PCB.

![PCB Editor](assets/pcb_editor.png){:class="screenshot"}

#### 4. Footprint Editor (CvPcb)

The Footprint Editor allows you to create and manage footprints, which are the physical representations of components on the PCB.

- **Assigning Footprints**:
  - In the Schematic Editor, assign footprints to your components. You can choose from existing libraries or create custom footprints.

- **Editing Footprints**:
  - Open the Footprint Editor to modify existing footprints or create new ones.

![Footprint Editor](assets/footprint_editor.png){:class="screenshot"}

#### 5. Library Manager

The Library Manager helps you organize and manage your component libraries.

- **Adding Libraries**:
  - You can add new libraries to KiCad, whether they are downloaded from the internet or created by you.

- **Organizing Components**:
  - Group components into libraries for easy access and management.

---

## Summary

In this lesson, you have explored the main components of the KiCad interface, including the Project Manager, Schematic Editor, PCB Editor, Footprint Editor, and Library Manager. Understanding these tools is crucial as you begin your journey in PCB design with KiCad.

Next, we will dive deeper into creating schematic diagrams. Ready to continue? Let's move on to the [Schematic Editor Basics](04_schematic_basics.md)!

---

![KiCad](assets/kicad_logo.png){:class="kicad-logo"}

---
