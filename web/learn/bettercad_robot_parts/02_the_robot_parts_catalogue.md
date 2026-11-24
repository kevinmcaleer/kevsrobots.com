---
layout: lesson
title: The Robot Parts Catalogue
author: Kevin McAleer
type: page
cover: /learn/bettercad_robot_parts/assets/banners/02_the_robot_parts_catalogue.jpg
date: 2026-11-21
previous: 01_reference_bodies.html
next: 03_measure_before_you_trust.html
description: Place servos, motors, boards, sensors and bearings from the BetterCad
  robot parts catalogue onto your model
percent: 21
duration: 5
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

By the end of this lesson you will be able to open the robot parts catalogue, place a servo on a plate, and move it to an exact position.

Importing an STL works when you have one. For the parts nearly every small robot uses, you don't need to go looking: BetterCad has them built in.

---

## Open the catalogue

Start a new model. This is the first step of the course project, so give it a base to build on.

1. Open the **Library** and click **Box**.
2. In Properties set the size to `120`, `100` and `4`.
3. Press <kbd>F2</kbd> and rename the box `Deck`.
4. Click the word **Insert** and choose **Catalogue**.

![The robot parts catalogue beside the deck](/learn/bettercad_robot_parts/assets/catalogue.jpg){:class="img-fluid w-100"}

The catalogue opens where the Browser was. Here's what's in it today:

| Category | Parts |
|---|---|
| Servos | SG90 micro servo, MG90S micro servo, MG996R servo |
| Motors | N20 gear motor, TT gear motor |
| Boards | Raspberry Pi Pico, Pico W, Zero 2 W, 4 Model B and 5 |
| Sensors | HC-SR04 ultrasonic sensor |
| Bearings | 608 bearing, 623 bearing |
| Power | 18650 cell |
{:class="table table-single"}

The number beside each part is its mass in grams. The search box matches every word you type against the name and description, so `micro servo` finds the SG90 and the MG90S.

## Place a servo

1. Click **SG90 micro servo**.
2. Move the pointer over the deck.

![Placing the SG90, with the Turn 90° button on the bar](/learn/bettercad_robot_parts/assets/placing.jpg){:class="img-fluid w-100"}

A bar appears: "Click a flat face, or the ground. Turned 0°. Sizes aren't checked: measure yours."

3. Click the top of the deck, towards the left end.

![The servo placed on the deck](/learn/bettercad_robot_parts/assets/placed.jpg){:class="img-fluid w-100"}

The servo lands where you clicked, snapped to a 0.5 mm grid. A few things to notice:

- It's a **reference body** already. You'll never print it by accident.
- Properties has a **Part** row: "SG90 micro servo, 9 g. Sizes aren't checked: measure yours."
- The Size section says "It's a catalogue part, so it keeps the part's size."
- The timeline gained an **Insert part** step named after the servo.

### Which way up?

Every catalogue part has a sensible zero. For a servo it's the underside of the mounting tabs, so the tabs sit on the face you clicked and the body hangs **below** it, through the plate. That's how a micro servo is normally fitted. Right now the body is buried in the deck because there's no hole for it yet. Lesson 4 fixes that.

### Turn it first

Before you click, press <kbd>T</kbd> or click **Turn 90°** on the bar to turn the part a quarter turn. Press it again for another quarter. <kbd>Esc</kbd> cancels.

## Move it exactly

A placed part moves like any other body. For the project, the servo needs to be in a known place.

1. With the servo selected, find **Position** in Properties.
2. Set **X** to `-35.2` and **Y** to `0`. Leave **Z** at `4`, the top of the deck.

Why 35.2 and not 35? A servo's output shaft is off-centre, 5.2 mm from the middle of this one. Putting the body at -35.2 puts the shaft at a round -30, which makes the gear sums easier later.

## Old models stay put

The Insert part step keeps its own copy of the part's details. If the catalogue's SG90 is corrected next month, the one in this model doesn't move and nothing you built round it breaks.

The catalogue is part of the app, so it works offline too.

---

## Try it Yourself

1. Place a **608 bearing** on the deck, then delete it.
2. Place an **HC-SR04** and press <kbd>T</kbd> before you click. Which way does it face?
3. Search the catalogue for `pico`. How many parts match?

## Common Issues

- **Problem**: The part lands on the ground, not on my plate.
- **Solution**: Click on a flat face of the plate itself. Turn the view so its top is clearly visible first.
- **Why**: Clicking empty space places the part on the ground.

- **Problem**: The servo seems to be sunk into the plate.
- **Solution**: That's correct. Its body hangs below the tabs and needs a pocket.
- **Why**: The part's zero is the surface it screws to.

- **Problem**: I can't change the servo's size.
- **Solution**: You can't, by design. Pick a different part if you need one.
- **Why**: A catalogue part keeps the part's size.

## Check your understanding

{% quiz %}
? What kind of body is a part from the catalogue?
- [ ] A printed body
- [ ] A hole shape
- [x] A reference body
> It's there to design round, so it never prints or exports.

? What does pressing T do while placing a part?
- [x] Turns it 90° before you click
- [ ] Opens the timeline
- [ ] Toggles it between reference and printed
> Press it again for another quarter turn.

? Where is an SG90's zero height?
- [ ] The tip of its output shaft
- [x] The underside of its mounting tabs
- [ ] The bottom of its body
> The tabs sit on the face you click and the body hangs below.
{% endquiz %}
