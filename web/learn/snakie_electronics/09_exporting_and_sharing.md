---
layout: lesson
title: Exporting and Sharing
author: Kevin McAleer
type: page
cover: /learn/snakie_electronics/assets/banners/09_exporting_and_sharing.jpg
date: 2026-11-20
previous: 08_connections_and_measurements.html
next: 10_project_obstacle_robot.html
description: Export a Snakie wiring project as a picture, a bill of materials, a pinout
  table or a complete PDF build guide, and send it to a kevsrobots project
percent: 80
duration: 6
quiz_total: 33
date_updated: 2026-11-24
navigation:
- name: Wire It Up - Electronics in Snakie
- content:
  - section: Overview
    content:
    - name: Wire It Up - Electronics in Snakie
      link: 00_intro.html
    - name: The Electronics Workspace
      link: 01_the_electronics_workspace.html
  - section: Parts
    content:
    - name: The Parts Library
      link: 02_the_parts_library.html
    - name: Placing Parts
      link: 03_placing_parts.html
  - section: Wiring
    content:
    - name: Wiring with Noodles
      link: 04_wiring_with_noodles.html
    - name: The Schematic View
      link: 05_the_schematic_view.html
    - name: Checking Your Wiring
      link: 06_checking_your_wiring.html
  - section: From Wiring to Code
    content:
    - name: Drivers, Help and Code
      link: 07_drivers_help_and_code.html
    - name: Connections and Measurements
      link: 08_connections_and_measurements.html
    - name: Exporting and Sharing
      link: 09_exporting_and_sharing.html
  - section: Project
    content:
    - name: Project - An Obstacle-Avoiding Robot
      link: 10_project_obstacle_robot.html
    - name: Summary and Next Steps
      link: 11_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to export your wiring as a picture, a bill of materials and a printable PDF build guide.

A circuit that only lives on your screen isn't much use at the workbench, or to anyone else. Snakie can turn it into pictures, parts lists and a build guide you can print.

---

## Name it first

Whatever you export carries the project's name, so check the Browser panel. Ours is called **Obstacle Robot**, with the description *A 2WD robot that turns away from walls*.

![The named project](/learn/snakie_electronics/assets/project_named.jpg){:class="img-fluid w-100"}

## The export menu

Click **Export image** in the toolbar, the button with the downward arrow.

![The export menu](/learn/snakie_electronics/assets/export_menu.jpg){:class="img-fluid w-100"}

| Choice | What you get | Good for |
|---|---|---|
| PNG image | A picture of the wiring | Blog posts, chat, slides |
| SVG image | The same picture as a vector drawing | Printing large, or editing in a drawing app |
| PDF document | The picture as a single PDF page | Printing the diagram on its own |
| BOM (Markdown) | The parts list as a table | A README or a shopping list |
| Pinouts (Markdown) | Every connection as a table | Documentation beside your code |
| Send to kevsrobots… | Puts the diagram in one of your kevsrobots projects | Sharing a whole project |
{:class="table table-single"}

## A picture

**PNG image** saves the diagram cropped to the parts, with none of Snakie's panels in the way.

![The exported PNG](/learn/snakie_electronics/assets/robot_export.jpg){:class="img-fluid w-100"}

Whichever view you're in is the one that's exported, so switch to **Schematic** first if you want the circuit diagram.

## A bill of materials

A **bill of materials**, or BOM, is the list of everything you need to buy or gather. Choose **BOM (Markdown)** and Snakie writes it for you:

| Qty | Part | Manufacturer | Family |
|---|---|---|---|
| 1 | Raspberry Pi Pico 2 W | Raspberry Pi | Microcontroller |
| 1 | 4×AA Battery Pack | Generic | Power |
| 1 | HC SR04 | Generic | Sensor |
| 1 | MX1508 | | Breakout |
| 2 | Yellow TT Motor | Generic | Motor |
{:class="table table-single"}

Notice it counted the two motors as one line with a quantity of 2.

**Pinouts (Markdown)** does the same for the wiring: a table of which board pin goes to which part pin. Both are Markdown, so they paste straight into a GitHub README.

## The build guide

Now the best one, and it isn't in the export menu. Click the **printer** button in the toolbar at the very top of Snakie, beside New and Save. Its tooltip reads *Export the project as a PDF*.

![Four pages of the PDF build guide](/learn/snakie_electronics/assets/pdf_pages.jpg){:class="img-fluid w-100"}

Snakie writes a five-page build guide:

1. A **cover** with the project's name and the date
2. **What you will need**: the parts list, including 15 jumper wires, one for each connection
3. **Electronics**: the wiring diagram
4. **Connections**: every wire as a row with a tick-box, to mark off as you build
5. A closing page with a link back to Snakie

Page 4 is the one that earns its keep. Building from a list with tick-boxes is much more reliable than building from a picture, especially in a classroom where thirty robots are being wired at once.

## Send to kevsrobots

If you have a kevsrobots.com account, **Send to kevsrobots…** puts the wiring diagram and its parts list straight into one of your projects on the site.

![The Send to kevsrobots dialog](/learn/snakie_electronics/assets/send_dialog.jpg){:class="img-fluid w-100"}

You sign in once, pick the project, and it's there for others to build from. As the dialog says, signing in is only for sending, and Snakie never sends your code. Your project links to that on GitHub.

> **Teachers**: The PDF build guide, with a tick-box for every wire, is ideal when many robots are being wired at once. Sending to kevsrobots needs a kevsrobots.com account and puts the diagram and parts list into a project on the site, so use the PNG, BOM and PDF exports instead where learners do not have accounts or work should stay inside the class.

---

## Try it Yourself

1. Export the **Schematic** view as a PNG and compare it with the breadboard one.
2. Export the BOM and work out roughly what the robot would cost to build.
3. Make the PDF and read the Connections page. Is every wire from lesson 4 there?

## Common Issues

**Problem**: The PDF is titled "Untitled project".
**Solution**: Type a name at the top of the Browser panel and export again.
**Why**: The cover and the file name come from the project name.

**Problem**: The picture shows the wrong view.
**Solution**: Switch to the view you want, Breadboard or Schematic, then export.
**Why**: Snakie exports what's on the canvas.

**Problem**: The jumper wire count looks too high.
**Solution**: Nothing to fix. It's one lead per connection.
**Why**: Fifteen connections need fifteen leads, even when several meet at the same ground pin.

## Check your understanding

{% quiz %}
? What is a bill of materials?
- [ ] A receipt from a shop
- [x] The list of parts a project needs
- [ ] A kind of circuit diagram
> A BOM lists every part and how many of each you need.

? Which export is best for building a circuit step by step?
- [ ] The SVG image
- [ ] The BOM
- [x] The build guide from the printer button, with a tick-box for every connection
> Ticking off one wire at a time is the most reliable way to build.

? What does Send to kevsrobots send?
- [x] The wiring diagram and its parts list
- [ ] Your code
- [ ] Your password
> It shares the diagram and parts. Your code stays where it is.
{% endquiz %}
