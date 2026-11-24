---
layout: lesson
title: Standoffs for a Board
author: Kevin McAleer
type: page
cover: /learn/bettercad_robot_parts/assets/banners/05_standoffs_for_a_board.jpg
date: 2026-11-21
previous: 04_cutting_a_pocket.html
next: 06_horn_mounts.html
description: Add standoffs for a Raspberry Pi Pico or Pi with the BetterCad Mount
  tool, using built-in hole patterns and screw or heat-set insert holes
percent: 42
duration: 6
quiz_total: 42
date_updated: 2026-11-24
navigation:
- name: Design Robot Parts with BetterCad
- content:
  - section: Overview
    content:
    - name: Design Robot Parts with BetterCad
      link: 00_intro.html
  - section: Designing Around Real Parts
    content:
    - name: Reference Bodies
      link: 01_reference_bodies.html
    - name: The Robot Parts Catalogue
      link: 02_the_robot_parts_catalogue.html
    - name: Measure Before You Trust
      link: 03_measure_before_you_trust.html
    - name: Cutting a Pocket
      link: 04_cutting_a_pocket.html
  - section: Mounts
    content:
    - name: Standoffs for a Board
      link: 05_standoffs_for_a_board.html
    - name: Horn Mounts
      link: 06_horn_mounts.html
  - section: Gears and Drives
    content:
    - name: Your First Gear
      link: 07_your_first_gear.html
    - name: Bores, Hubs and Small Gears
      link: 08_bores_hubs_and_small_gears.html
    - name: Meshing Gears
      link: 09_meshing_gears.html
    - name: Racks, Pulleys and Belts
      link: 10_racks_pulleys_and_belts.html
  - section: Parts You Already Have
    content:
    - name: From Mesh to Editable Part
      link: 11_from_mesh_to_editable_part.html
  - section: Project
    content:
    - name: Project - A Geared Servo Deck
      link: 12_project_geared_servo_deck.html
    - name: Summary and Next Steps
      link: 13_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to add standoffs for a board with the Mount tool, choosing a built-in hole pattern and either Through or Insert holes.

A circuit board shouldn't sit flat on a printed plate. Its solder joints need air under them, and it needs something to screw into. The answer is a **standoff**: a short post with a hole down the middle.

A Pico needs four, 47 mm apart one way and 11.4 mm the other. You don't need to remember that. The **Mount** tool knows.

---

## Start the Mount tool

1. Click the word **Insert** and choose **Mount**.

The bar says "Click a flat face where the mount goes."

2. Click the top of the deck, in the open area in front of the servo.

One standoff appears where you clicked, and the Mount panel opens.

| Setting | What it does |
|---|---|
| **Kind** | Standoffs, or a Horn mount (next lesson) |
| **Screw** | The screw the holes are sized for |
| **Layout** | One standoff, a pattern, or the points you click |
| **Hole** | Through, or Insert |
| **Height** and **Width** | The size of each post |
| **Standoff X** and **Y** | Where the last one is, from the face's centre |
{:class="table table-single"}

You can keep clicking to add more posts one at a time, and **Remove last** takes the newest one away. For a board there's a faster way.

## Pick a pattern

1. Open the **Layout** menu.

It lists three general patterns and one for every catalogue part that has screw holes:

- One standoff
- Rectangle, 40 × 30 mm
- Circle of 4, 30 mm
- SG90 and MG90S micro servo (27.8 mm), MG996R servo (49.5 × 10 mm)
- Raspberry Pi Pico and Pico W (47 × 11.4 mm)
- Raspberry Pi Zero 2 W (58 × 23 mm)
- Raspberry Pi 4 Model B and Raspberry Pi 5 (58 × 49 mm)

2. Choose **Raspberry Pi Pico (47 × 11.4 mm)**.

![Four standoffs in a Pico pattern, with the Mount panel](/learn/bettercad_robot_parts/assets/standoffs.jpg){:class="img-fluid w-100"}

Four posts replace the one, centred where it was. Notice **Screw** changed to **M2** by itself: picking a part's pattern also picks its screw.

## Through or Insert

**Hole** decides what's inside each post.

- **Through** makes a printed clearance hole all the way down, through the post and the plate. A screw goes in from one side and a nut holds it on the other.
- **Insert** makes a hole at the top sized for a **heat-set insert**: a small brass thread you press in with a soldering iron. The board then screws down from above, and you can take it off and put it back as often as you like.

Inserts are the neater choice for anything you'll service. Through holes need no extra parts. Pick **Insert** for the project if you have M2 inserts, **Through** if you don't.

## Size and place

- **Height** is how far the board sits off the plate. The default 6 mm leaves room for solder joints underneath. Use more if your board has header pins pointing down.
- **Width** is the post's diameter. 6 mm is solid enough for M2 and M2.5.

For the project, the pattern should be centred 20 mm to the right of the middle and 38 mm towards the front. Click the deck near there. If you want it exact, the posts' positions are stored in the step and you can retype **Standoff X** and **Y**, but placing by eye is fine here: the board only has to miss the gears.

A word on those X and Y numbers. They're measured from the **centre of the face you clicked**. On a plain plate that's the middle. On a face with a pocket cut out of one end the centre shifts slightly towards the other end, because it's the middle of what's left. If a mount has to be exact, add it before you cut pockets in the same face.

3. Click **OK**, or press <kbd>Enter</kbd>.

The posts are joined to the deck, and one **Standoffs** step sits on the timeline. Right-click it and choose **Edit** to change anything later.

## Check it with the real board

Drop the board on to see it fit.

1. Open **Catalogue** from the Insert menu and click **Raspberry Pi Pico**.
2. Click the deck near the standoffs.
3. In Properties, set the Pico's **Position** so it's centred over the pattern, and set **Z** to `10`: the 4 mm deck plus 6 mm of standoff.

![A Pico sitting on its four standoffs](/learn/bettercad_robot_parts/assets/pico_on.jpg){:class="img-fluid w-100"}

The Pico's four corner holes line up with the four posts. It's a reference body, so it's only there for you to look at.

---

> **Teachers**: the Insert option makes holes for heat-set inserts, which are pressed in with a soldering iron. That is a hot tool, so supervise it under your usual workshop rules. The lesson says Through holes need no extra parts, so choose Through if you have no inserts or no soldering irons in the room.

## Try it Yourself

1. On a new 100 × 70 × 3 mm box, add standoffs with the **Raspberry Pi Zero 2 W** pattern. Which screw does it choose?
2. Edit the step and change **Height** to 10. Do all four posts change?
3. Add a second Mount step using **Circle of 4, 30 mm**. What might that suit?

## Common Issues

- **Problem**: I clicked and nothing happened.
- **Solution**: Click a flat face of a printed body. The tool doesn't place mounts on the ground.
- **Why**: A standoff has to be joined to something.

- **Problem**: The pattern landed half off the plate.
- **Solution**: Click nearer the middle of the space you have, or retype the X and Y.
- **Why**: A pattern is centred on the point you clicked.

- **Problem**: My heat-set inserts are loose in the printed holes.
- **Solution**: Measure your inserts and print a single test post before the whole plate. Sizes vary between makers.
- **Why**: The Insert hole is sized for a typical insert of that thread.

## Check your understanding

{% quiz %}
? What happens to the Screw setting when you pick the Raspberry Pi Pico pattern?
- [ ] It stays as it was
- [x] It changes to M2, the Pico's screw
- [ ] It's switched off
> Picking a part's pattern also picks its screw.

? What is the Insert hole option for?
- [x] A heat-set threaded insert at the top of each post
- [ ] Inserting the board without screws
- [ ] A hole right through for a nut and bolt
> Through is the nut-and-bolt option.

? Where are Standoff X and Y measured from?
- [ ] The model's origin
- [ ] The corner of the plate
- [x] The centre of the face you clicked
> And that centre shifts a little if a pocket has been cut out of the face.
{% endquiz %}
