---
layout: lesson
title: Create the Laser-Cut Cluster Panel
author: Kevin McAleer
type: page
cover: /learn/mini_rack/assets/banners/09_cluster_panel.jpg
date: 2025-02-16
previous: 08_top_panel.html
next: 07_print.html
description: Design the laser-cut cluster panel frame that carries the Pi trays at
  the front of the mini-rack
percent: 90
duration: 3
quiz_total: 33
date_updated: 2026-11-24
navigation:
- name: Mini-Rack 3D Design Tutorial
- content:
  - section: Overview
    content:
    - name: Introduction
      link: 00_intro.html
    - name: Components
      link: 01_components.html
  - section: Part Designs
    content:
    - name: Create the 2U Corner
      link: 02_2u_design.html
    - name: Create the Bottom Corner
      link: 03_bottom.html
    - name: Create the Top Corner
      link: 04_top.html
    - name: Create the Handle
      link: 05_handle.html
    - name: Create the Cluster Panel Plate
      link: 06_panel.html
    - name: Create the Pi-Tray
      link: 07_pi_tray.html
    - name: Create the Top Panel
      link: 08_top_panel.html
    - name: Create the Laser-Cut Cluster Panel
      link: 09_cluster_panel.html
  - section: Printing
    content:
    - name: Print the parts
      link: 07_print.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to design a laser-cut frame with rounded corners, and say how it differs from the printed panel in lesson 6.

Ahoy there makers! In lesson 6 we made a plate at `4mm`. Here we make the cluster panel again, this time as a frame with an open window in the middle, so the Pis behind it have room to breathe. It is cut from sheet, so its thickness matches your material.

This is the blue frame at the front of the rack in the components photo.

1. Create a new file named `cluster_panel` in Fusion 360

1. Create a new sketch on the XY plane, named `panel_base`

    ![cluster panel](/learn/mini_rack/assets/cluster_panel_01_sketch.png){:class="img-fluid w-100"}

1. Extrude the body profile by `2mm` (or whatever the depth of the wood you're looking to use is):

    ![Extrude](/learn/mini_rack/assets/cluster_panel_02_extrude.png){:class="img-fluid w-100"}

1. Create a new sketch on the top of the new part, named `holes`

    ![Sketch](/learn/mini_rack/assets/cluster_panel_03_sketch.png){:class="img-fluid w-100"}

1. Extrude-Cut the `holes` sketch:

    ![Extrude](/learn/mini_rack/assets/cluster_panel_04_extrude.png){:class="img-fluid w-100"}

    ![Extrude](/learn/mini_rack/assets/cluster_panel_05_extrude.png){:class="img-fluid w-100"}

1. Use the Fillet tool to round the outside edges of the frame by `4mm`:

    ![Fillet](/learn/mini_rack/assets/cluster_panel_06_fillet.png){:class="img-fluid w-100"}

## Why a frame?

A solid plate would block the front of the rack. A frame keeps the stiff edges where the holes are, and leaves the middle open. The fillet in the last step rounds the outside edges by `4mm`, which looks neater, and stops sharp corners catching a sleeve or a cable.

Like the top panel, set the extrude thickness to match your material: `2mm`, or whatever the depth of the wood you're looking to use is. If you want to cut it, use the same approach as lesson 8: project the edges into a sketch, and export a DXF.

## Try it Yourself

1. Compare this panel with the one in lesson 6. List two differences you can see.
2. Change the fillet from `4mm` to `2mm`. Which looks better to you?
3. Challenge: follow the `dxf` method from lesson 8 and export a DXF of your cluster panel.

## Common Issues

- **Problem**: The fillet will not apply to all the outside corners.
- **Solution**: Select the four vertical edges at the outside corners, one by one, and keep the radius at `4mm`.
- **Why**: A fillet only rounds the edges that you choose.

- **Problem**: The holes are too tight on the real bolts.
- **Solution**: Cut a test piece from offcuts, then adjust the hole diameter slightly.
- **Why**: The laser burns away a little material along its path, and wood can swell with moisture.

## Check your understanding

{% quiz %}
? What radius does the last step use for the outside fillet?
- [ ] 2mm
- [x] 4mm
- [ ] 6mm
> The Fillet tool rounds the outside edges by 4mm.

? How thick is the extrude in this lesson?
- [x] 2mm, or whatever thickness your wood is
- [ ] 20mm
- [ ] 15.875mm
> The lesson suggests 2mm, or the depth of the wood you plan to use.

? Why is this panel a frame?
- [ ] It is cheaper to print
- [x] It keeps the edges with the holes and leaves the middle open
- [ ] The laser cannot cut solid sheet
> The frame keeps the stiff edges where the holes are.
{% endquiz %}
