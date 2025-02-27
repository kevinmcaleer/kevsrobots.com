---
layout: lesson
title: Create the Top Panel
author: Kevin McAleer
type: page
cover: /learn/mini_rack/assets/cover.jpg
date: 2025-02-16
previous: 07_pi_tray.html
next: 07_print.html
description: Learn how to create the top panel for the mini-rack
percent: 90
duration: 1
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
    - name: Create the Cluster Panel
      link: 06_panel.html
    - name: Create the Pi-Tray
      link: 07_pi_tray.html
    - name: Create the Top Panel
      link: 08_top_panel.html
  - section: Printing
    content:
    - name: Print the parts
      link: 07_print.html
---


## How to create the top Panel for the Mini-Rack

1. Create a new file named `top_panel` in Fusion 360

1. Create a new sketch on the XY plane, named `panel_base`

    ![dimensions](/learn/mini_rack/assets/top_panel_01_sketch.png){:class="img-fluid w-100"}

1. Extrude the body profile by `2mm` (or whatever the depth of the wood you're looking to use is):

    ![Extrude](/learn/mini_rack/assets/top_panel_02_extrude.png){:class="img-fluid w-100"}

    ![Extrude](/learn/mini_rack/assets/top_panel_03_extrude.png){:class="img-fluid w-100"}

1. Create a new sketch on the top of the new part, named `holes`

    ![Sketch](/learn/mini_rack/assets/top_panel_04_sketch.png){:class="img-fluid w-100"}

1. Extrude-Cut the `holes` sketch:

    ![Extrude](/learn/mini_rack/assets/top_panel_05_extrude.png){:class="img-fluid w-100"}

    ![Extrude](/learn/mini_rack/assets/top_panel_06_extrude.png){:class="img-fluid w-100"}

1. Create a new sketch on the top of the new part, named `slots`

    ![Sketch](/learn/mini_rack/assets/top_panel_07_sketch.png){:class="img-fluid w-100"}

1. Extrude-Cut the `slots` sketch:

    ![Extrude](/learn/mini_rack/assets/top_panel_08_extrude.png){:class="img-fluid w-100"}

    ![Extrude](/learn/mini_rack/assets/top_panel_09_extrude.png){:class="img-fluid w-100"}

1. Use the rectangular pattern tool to create the remaining slots:

    ![Pattern](/learn/mini_rack/assets/top_panel_10_pattern.png){:class="img-fluid w-100"}

    ![Pattern](/learn/mini_rack/assets/top_panel_11_pattern.png){:class="img-fluid w-100"}

1. Finally, to create a DXF file for laser-cutting, create a new sketch on the top of the new part, named `dxf` and then use the projection (press the `p` key) tool to project all the edges of the part:

    ![DXF](/learn/mini_rack/assets/top_panel_12_sketch.png){:class="img-fluid w-100"}

---
