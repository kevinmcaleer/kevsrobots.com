---
layout: lesson
title: Tracing a Board Photo
author: Kevin McAleer
type: page
cover: /learn/snakie_part_editor/assets/banners/06_tracing_a_board_photo.jpg
date: 2026-11-21
previous: 05_holes_components_and_labels.html
next: 07_schematic_symbol_and_electrical_details.html
description: Use a photo of a real board as the picture for your Snakie part, remove
  its background and line the pins up on top
percent: 49
duration: 5
quiz_total: 41
date_updated: 2026-11-21
navigation:
- name: Make Your Own Parts and Boards in Snakie
- content:
  - section: Overview
    content:
    - name: Make Your Own Parts and Boards in Snakie
      link: 00_intro.html
    - name: What a Part Is
      link: 01_what_a_part_is.html
  - section: Drawing the Part
    content:
    - name: A Tour of the Part Editor
      link: 02_tour_of_the_part_editor.html
    - name: Board Size and Details
      link: 03_board_size_and_details.html
    - name: Pins, Types and Capabilities
      link: 04_pins_types_and_capabilities.html
    - name: Holes, Components and Labels
      link: 05_holes_components_and_labels.html
    - name: Tracing a Board Photo
      link: 06_tracing_a_board_photo.html
  - section: Making It Smart
    content:
    - name: Schematic Symbol and Electrical Details
      link: 07_schematic_symbol_and_electrical_details.html
    - name: Drivers and Built-In Help
      link: 08_drivers_and_built_in_help.html
    - name: Using Your Part in a Project
      link: 09_using_your_part_in_a_project.html
    - name: Giving Your Part Its Own Blocks
      link: 10_giving_your_part_its_own_blocks.html
  - section: Boards and Sharing
    content:
    - name: Making a Microcontroller Board
      link: 11_making_a_microcontroller_board.html
    - name: Sharing Your Parts
      link: 12_sharing_your_parts.html
    - name: Summary and Next Steps
      link: 13_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

Drawn shapes are quick, but nothing beats the real board. The Part Editor can put a **photo** on your part, cut away the background and scale it to fit. Then you just check the pins sit on top of the pads.

---

## Get a good picture

A good source photo makes this easy.

- Shoot from **straight above**, so the board isn't a wedge shape.
- Put the board on a **plain, contrasting background**. White paper works well.
- Use even light, with no hard shadow along one edge.
- Keep the board **the same way up** as your part: pins along the bottom.

No photo yet? Practise with the picture we used: [vl53l0x_board.png](/learn/snakie_part_editor/assets/vl53l0x_board.png).

> Using someone else's product photo? Check you're allowed to, especially if you plan to share the part.

---

## Add the photo

1. In **Layers**, find the **Background** row at the bottom.
2. Click its **＋** button and choose your picture. PNG, JPEG, WebP and SVG all work.

The photo appears on the board with a handle at each corner, white background and all.

![A photo added to the board, before clean-up](/learn/snakie_part_editor/assets/photo_added.jpg){:class="img-fluid w-100"}

The Inspector now shows **Image layer**, with **x**, **y**, **w**, **h**, an **Opacity** slider and **Lock aspect ratio**.

---

## Clean it up

Next to **Background** there's now a **BG** button. Click it for the photo tools.

![The photo tools menu](/learn/snakie_part_editor/assets/photo_menu.jpg){:class="img-fluid w-100"}

| Tool | What it does |
|---|---|
| **Crop to a box…** | Drag a box around the part of the photo to keep |
| **Remove background after crop** | Tick this to clear the background as part of the crop |
| **Zoom to fit** | Trims the empty margin and scales the photo to fill the board |
| **Auto remove (from edges)** | Clears the background, working inwards from every edge |
| **Click to erase a spot** | Click a leftover patch to erase it |
| **Tolerance** | How different a colour must be to survive. Raise it to erase more |
| **Reset image** | Undo all the erasing |
{:class="table table-single"}

For a board on plain paper, two clicks do the job:

1. Click **Auto remove (from edges)**. The white disappears.
2. Open **BG** again and click **Zoom to fit**.

The photo now fills the board exactly, and the pads in the photo sit under the pins you placed.

![The finished part with its photo](/learn/snakie_part_editor/assets/photo_fit.jpg){:class="img-fluid w-100"}

If patches of background are left behind, choose **Click to erase a spot** and click each one. If it eats into the board, lower **Tolerance** and try again.

---

## Line up the pins

Zoom in and look at each pin. It should sit in the middle of its pad in the photo.

- If one is off, drag it. Hold **Shift** to move it freely, off the grid.
- If they're *all* off by the same amount, the board size is probably wrong. Measure again and fix **Width** and **Height**.

Because we measured first and used the grid, ours lined up straight away.

---

## Tidy the layers

The photo already shows the chip and the printed text, so the shapes from the last lesson are now doubled up.

- Click the eye on the **Components** row to hide them, or delete them.
- Use the tick box on the **Background** row to hide the photo while you work on pins.
- Click **Replace** to swap in a better photo later. Your pins stay where they are.

---

## The other side

Does your board have parts on the back? Click **Flip to back** below the canvas and add a second photo there. Any pin can be moved to the back by setting **Side** to **Rear** in the Inspector.

---

## Try it Yourself

1. Add a photo of your board, or our practice picture.
2. Remove the background and zoom to fit.
3. Slide **Opacity** down to about half. You can now see the grid through the photo, which helps when checking pins.
4. Put Opacity back to full, hide the Components layer and save.

---

## Common Issues

**Problem**: Auto remove leaves the gaps inside the mounting holes white.
**Solution**: Use **Click to erase a spot** on each one.
**Why**: Auto remove works inwards from the edges and can't reach enclosed areas.

**Problem**: Half my white silkscreen text vanished.
**Solution**: **Reset image**, lower the Tolerance and try again. A coloured background behind the board helps too.
**Why**: White text and a white background are the same colour to the eraser.

**Problem**: The photo looks stretched.
**Solution**: Check the board's width and height match the real board.
**Why**: Zoom to fit fills the board, so a wrong board shape squeezes the photo.

---

## Check your understanding

{% quiz %}
? What makes a photo easiest to clean up?
- [ ] A dark room
- [x] A plain background that contrasts with the board, shot from straight above
- [ ] A photo taken at an angle
> Even colour is easy to remove, and a square-on shot keeps the pads where the grid expects them.

? What does Zoom to fit do?
- [ ] Zooms the canvas
- [x] Trims the empty margin and scales the photo to fill the board
- [ ] Makes the board bigger
> It's the step that makes the photo and your measured board agree.

? Every pin is slightly off its pad by the same amount. What's the likely cause?
- [x] The board's width or height is wrong
- [ ] The pins are the wrong type
- [ ] The photo is too bright
> Positions are fractions of the board, so a wrong size shifts everything together.
{% endquiz %}
