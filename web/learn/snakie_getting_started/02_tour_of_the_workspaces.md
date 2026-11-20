---
layout: lesson
title: A Tour of the Workspaces
author: Kevin McAleer
type: page
cover: /learn/snakie_getting_started/assets/banners/02_tour_of_the_workspaces.jpg
date: 2026-11-20
previous: 01_web_or_desktop.html
next: 03_first_program_on_the_simulator.html
description: Find your way around Snakie - the toolbar, the side bar, and the Blocks,
  Code, Electronics and Build workspaces
percent: 24
duration: 4
quiz_total: 20
date_updated: 2026-11-20
navigation:
- name: Getting Started with Snakie
- content:
  - section: Overview
    content:
    - name: Getting Started with Snakie
      link: 00_intro.html
    - name: Web App or Desktop App
      link: 01_web_or_desktop.html
    - name: A Tour of the Workspaces
      link: 02_tour_of_the_workspaces.html
  - section: Your First Programs
    content:
    - name: Your First Program on the Simulator
      link: 03_first_program_on_the_simulator.html
    - name: The REPL and the Console
      link: 04_the_repl_and_console.html
    - name: Seeing Your Pins
      link: 05_seeing_your_pins.html
  - section: Real Hardware
    content:
    - name: Finding Your Board
      link: 06_finding_your_board.html
    - name: Flashing MicroPython
      link: 07_flashing_micropython.html
    - name: Connect, Run and Stop
      link: 08_connect_run_stop.html
    - name: Files on Your Computer and on the Board
      link: 09_files_local_and_device.html
  - section: Project
    content:
    - name: Project - Traffic Light
      link: 10_project_traffic_light.html
    - name: Summary
      link: 11_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

Snakie can look busy the first time you open it. It's really four workspaces sharing one toolbar, and once you know what each is for, everything has an obvious home.

---

## The toolbar

Across the top, from left to right:

- **New file**, **Save** and **Export the project as a PDF**
- **Run** (green) sends the file you're editing to the board and starts it
- **Reset** (red) restarts the board. While a program is running this button becomes **Stop**
- The workspace switcher: **Blocks**, **Code**, **Electronics** and **Build**

## The side bar

Down the left edge are the panels that sit beside your work:

| Icon | Panel | What it's for |
|---|---|---|
| Folder | Files | Files on your computer and on the board |
| Cube | Packages | Install MicroPython libraries onto the board |
| Magnifier | Inspect | An outline of your file and the variables on the board |
| Bug | Report Bug | Tell Kev when something goes wrong |
| Mortarboard | Learn | Short guided tutorials that run inside the app |
| Question mark | Help | The built-in help library |
| Cog | Settings | Themes, editor options and more |
{:class="table table-single"}

## Code

**Code** is the editor-first workspace: files on the left, the editor in the middle, the console underneath, and the board and instruments on the right. This is where you'll spend most of this course.

![The Code workspace](/learn/snakie_getting_started/assets/snakie_code_workspace.jpg){:class="img-fluid w-100"}

## Blocks

**Blocks** is drag-and-drop programming, the way Scratch works, for a real microcontroller. The clever part is that the blocks and the Python are the same program. Open a Python file in Blocks and Snakie draws it as blocks; drag blocks and Snakie writes the Python.

The small dot between **Blocks** and **Code** in the switcher is a third stop: blocks and Python side by side.

![Blocks and Python side by side](/learn/snakie_getting_started/assets/workspace_blocks.jpg){:class="img-fluid w-100"}

You'll learn Blocks properly in [Snakie Blocks: From Blocks to Python](/learn/snakie_blocks/).

## Electronics

**Electronics** is where you wire things up on screen. Pick your board, drag parts from the **Library** on the right, and draw wires between pins. Switch between a life-like **Breadboard** and a **Schematic** with the tabs at the top left.

![The Electronics workspace](/learn/snakie_getting_started/assets/workspace_electronics.jpg){:class="img-fluid w-100"}

## Build

**Build** is the 3D workspace. Here you assemble a robot from parts, add joints, and pose it. A new project shows a demo arm until you start your own robot or open a URDF file, which is a standard way to describe a robot. You can design the parts in [BetterCAD](https://bettercad.kevsrobots.com/), Kev's free CAD app.

![The Build workspace](/learn/snakie_getting_started/assets/workspace_build.jpg){:class="img-fluid w-100"}

## The status bar

Along the bottom, Snakie tells you what it's connected to, shows a rotating tip, the length of your file, whether it's saved, and the version number. The **Flash firmware** button at the far right puts MicroPython onto a board. We'll use it in a later lesson.

## Learn and Help

Two panels are worth opening on day one.

**Learn** has a robot build checklist and four short in-app courses that run on the simulator.

![The Learn panel](/learn/snakie_getting_started/assets/learn_panel.jpg){:class="img-fluid w-100"}

**Help** is a library of articles on Snakie, its instruments and the Python language. It follows the board you're connected to, so a MicroPython board gets MicroPython help.

![The Help library](/learn/snakie_getting_started/assets/help_library.jpg){:class="img-fluid w-100"}

---

## Try it Yourself

1. Click each of the four workspaces in turn and name one thing you can only do there.
2. Click the dot between **Blocks** and **Code**. What changes?
3. Open **Help** and find the article called **Connect your board**.

## Common Issues

**Problem**: A panel has disappeared.
**Solution**: Click its icon in the side bar again, or look for a small arrow at the edge of the window to bring it back.
**Why**: Every panel can be collapsed to make room, and Snakie remembers the layout for each workspace.

## Check your understanding

{% quiz %}
? Which workspace would you use to draw the wires between a sensor and your Pico?
- [ ] Code
- [x] Electronics
- [ ] Build
> Electronics has the breadboard and schematic views, and the parts library.

? What happens to the red Reset button while a program is running?
- [ ] It disappears
- [ ] It turns into Save
- [x] It turns into Stop
> While a program runs the button stops it. When nothing is running it resets the board.
{% endquiz %}
