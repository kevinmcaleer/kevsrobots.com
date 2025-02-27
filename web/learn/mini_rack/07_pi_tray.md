---
layout: lesson
title: Create the Pi-Tray
author: Kevin McAleer
type: page
cover: /learn/mini_rack/assets/cover.jpg
date: 2025-02-16
previous: 06_panel.html
next: 08_top_panel.html
description: Raspberry Pi 5 Cluster Panel Tray
percent: 72
duration: 2
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


1. Create a new file named `pi_tray` in Fusion 360

1. Create a new sketch on the XY plane, named `tray_base`

    ![dimensions](/learn/mini_rack/assets/pi_tray_01_sketch.png){:class="img-fluid w-100"}

1. Extrude the body profile by `3mm`:

    ![Extrude](/learn/mini_rack/assets/pi_tray_02_extrude.png){:class="img-fluid w-100"}

    ![Extrude](/learn/mini_rack/assets/pi_tray_03_extrude.png){:class="img-fluid w-100"}

1. Create a new sketch on the top of the new part, named `holes`

    ![Sketch](/learn/mini_rack/assets/pi_tray_04_sketch.png){:class="img-fluid w-100"}

1. Extrude-Cut the `holes` sketch:

    ![Extrude](/learn/mini_rack/assets/pi_tray_05_extrude.png){:class="img-fluid w-100"}

1. Turn the tray_base Sketch back on and extrude the 3mm edge to create the face of the tray:

    ![Extrude](/learn/mini_rack/assets/pi_tray_06_extrude.png){:class="img-fluid w-100"}

    ![Extrude](/learn/mini_rack/assets/pi_tray_07_extrude.png){:class="img-fluid w-100"}

1. Create a new sketch on the Face of the last step, named `mounting_holes`

    ![Sketch](/learn/mini_rack/assets/pi_tray_08_sketch.png){:class="img-fluid w-100"}

1. Extrude-Join the `mounting_holes` sketch:

    ![Extrude](/learn/mini_rack/assets/pi_tray_09_extrude.png){:class="img-fluid w-100"}

    ![Extrude](/learn/mini_rack/assets/pi_tray_10_extrude.png){:class="img-fluid w-100"}

1. Extrude-Cut the `mounting_holes` sketch:

    ![Extrude](/learn/mini_rack/assets/pi_tray_11_extrude.png){:class="img-fluid w-100"}

    ![Extrude](/learn/mini_rack/assets/pi_tray_12_extrude.png){:class="img-fluid w-100"}

1. Create a new sketch on the base of the tray, named `cooling`

    ![Sketch](/learn/mini_rack/assets/pi_tray_13_sketch.png){:class="img-fluid w-100"}

1. Extrude-Cut the `cooling` sketch:

    ![Extrude](/learn/mini_rack/assets/pi_tray_14_extrude.png){:class="img-fluid w-100"}

1. on the Inside edges of the tray, use the Chamfer tool to add the edges:

    ![Extrude](/learn/mini_rack/assets/pi_tray_15_chamfer.png){:class="img-fluid w-100"}

    ![Extrude](/learn/mini_rack/assets/pi_tray_16_chamfer.png){:class="img-fluid w-100"}

## Creating the Pi 4 Tray

We can quickly create the Pi 4 version of the tray by mirroring the Pi 5 tray - 

1. Create an offset plane 20mm from the front face - we will use this to mirror the body of the tray:

    
    ![Extrude](/learn/mini_rack/assets/pi_tray_17_plane.png){:class="img-fluid w-100"}

1. Use the Mirror tool to mirror the `tray_base` body:

    ![Extrude](/learn/mini_rack/assets/pi_tray_18_mirror.png){:class="img-fluid w-100"}

    ![Mirror](/learn/mini_rack/assets/pi_tray_19_mirror.png){:class="img-fluid w-100"}