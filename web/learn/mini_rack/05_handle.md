---
layout: lesson
title: Create the Handle
author: Kevin McAleer
type: page
cover: /learn/mini_rack/assets/cover.jpg
date: 2025-02-16
previous: 04_top.html
next: 06_panel.html
description: Learn how to create the handle for the mini-rack
percent: 54
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
    - name: Create the Cluster Panel
      link: 09_cluster_panel.html
  - section: Printing
    content:
    - name: Print the parts
      link: 07_print.html
---


![Handle Design](assets/handle_design.png){:class="img-fluid w-100"}

1. Create a new file named `handle` in Fusion 360

1. Create a new sketch on the XY plane, named `handle_base`

    ![dimensions](/learn/mini_rack/assets/handle_01_sketch.png){:class="img-fluid w-100"}

1. Extrude the body profile by `20mm`:

    ![Extrude](/learn/mini_rack/assets/handle_02_extrude.png){:class="img-fluid w-100"}

1. Create a new sketch on the top of the new part, named `top_profile`

    ![Sketch](/learn/mini_rack/assets/handle_03_sketch.png){:class="img-fluid w-100"}

1. Extrude-cut the profile:

    ![Extrude](/learn/mini_rack/assets/handle_04_extrude.png){:class="img-fluid w-100"}

1. Fillet the handle edges:

    ![Fillet](/learn/mini_rack/assets/handle_05_fillet.png){:class="img-fluid w-100"}

1. Create a new sketch on the bottom of the body, named `bottom holes`

    ![Sketch](/learn/mini_rack/assets/handle_06_sketch.png){:class="img-fluid w-100"}

1. Extrude the sketch up by `3mm` to create the platform the screws will sit on:

    ![Extrude](/learn/mini_rack/assets/handle_07_extrude.png){:class="img-fluid w-100"}

---
