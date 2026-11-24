---
layout: lesson
title: Create the Top Corner
author: Kevin McAleer
type: page
cover: /learn/mini_rack/assets/banners/04_top.jpg
date: 2025-02-16
previous: 03_bottom.html
next: 05_handle.html
description: Design the top corner piece that caps each mini-rack post and carries
  the top panel
percent: 45
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

By the end of this lesson you will be able to design the top corner, and say how it differs from the bottom corner.

Ahoy there makers! The top corner finishes each post and supports the top panel and handle. If you made the bottom corner in the last lesson, this one will go quickly. It reuses the same pattern, with a fillet added at the end to round off the inside edge.

Here is the finished part.

![Top Design](assets/top_design.png){:class="img-fluid w-100"}

1. Create a new file named `top_corner` in Fusion 360

1. Create a new sketch on the XY plane, named `top_base`

1. Create the following sketch:

    ![dimensions](/learn/mini_rack/assets/top_01_sketch.png){:class="img-fluid w-100"}

1. Extrude the body profile by `4mm`:

    ![Extrude](/learn/mini_rack/assets/top_02_extrude.png){:class="img-fluid w-100"}

1. Create a sketch on the top of the new part, named `top_profile`

    ![Sketch](/learn/mini_rack/assets/top_03_sketch.png){:class="img-fluid w-100"}

1. Extrude the sketch:

    ![Extrude](/learn/mini_rack/assets/top_04_extrude.png){:class="img-fluid w-100"}

1. Create a new sketch on the top of the body, named `side holes`

    ![Sketch](/learn/mini_rack/assets/top_05_sketch.png){:class="img-fluid w-100"}

1. Extrude the sketch:

    ![Extrude](/learn/mini_rack/assets/top_06_extrude.png){:class="img-fluid w-100"}

1. Extrude the sketch:

    ![Extrude](/learn/mini_rack/assets/top_07_extrude.png){:class="img-fluid w-100"}

1. Extrude the sketch:

    ![Extrude](/learn/mini_rack/assets/top_08_extrude.png){:class="img-fluid w-100"}

1. Create a new sketch on the top of the body, named `ribs`

    ![Sketch](/learn/mini_rack/assets/top_09_sketch.png){:class="img-fluid w-100"}

1. Extrude the sketch:

    ![Extrude](/learn/mini_rack/assets/top_10_extrude.png){:class="img-fluid w-100"}

1. Create a construction plane:

    ![Extrude](/learn/mini_rack/assets/top_11_plane.png){:class="img-fluid w-100"}

1. Extrude the `ribs` sketch:

    ![Extrude](/learn/mini_rack/assets/top_12_extrude.png){:class="img-fluid w-100"}

1. Fillet the inside bottom edge:

    ![Fillet](/learn/mini_rack/assets/top_13_fillet.png){:class="img-fluid w-100"}

## What is different from the bottom corner?

The workflow is the same: a 4mm base, a `top_profile`, `side holes`, then `ribs`. The differences are in the shapes, and in the last step, where we fillet the inside bottom edge. A fillet rounds a sharp inside corner. On a printed part that helps, because sharp inside corners are where stress builds up and cracks start.

Where the steps say "Extrude the sketch" several times in a row, you are extruding different profiles of the same sketch. Select each profile in turn, and compare your result with the picture after each one.

## Try it Yourself

1. Before you start the fillet, check the inside edge you will select. Which edge is "inside bottom"?
2. Try a larger fillet and a smaller one. What changes in the picture?
3. Challenge: look at the components photo in lesson 1. Which face of the top corner meets the top panel?

## Common Issues

- **Problem**: The fillet fails with an error.
- **Solution**: Make the fillet radius smaller, or select just one edge at a time.
- **Why**: A fillet cannot be bigger than the face next to it, so Fusion 360 refuses.

- **Problem**: I extruded the wrong profile in one of the repeated steps.
- **Solution**: Press Ctrl+Z (Cmd+Z on a Mac) and select the profile that matches the picture.
- **Why**: A sketch with many closed areas offers each of them as a profile.

## Check your understanding

{% quiz %}
? What does the last step of the top corner do?
- [ ] Adds ribs
- [x] Fillets the inside bottom edge
- [ ] Cuts side holes
> The final step rounds the inside bottom edge with the Fillet tool.

? Why do designers round inside corners?
- [x] Sharp inside corners concentrate stress and can crack
- [ ] It makes the part lighter
- [ ] It is needed to export an STL
> Rounded corners spread the load more evenly.

? Which plane is the `top_base` sketch drawn on?
- [ ] The top face
- [x] The XY plane
> Step 2 creates the sketch on the XY plane.
{% endquiz %}
