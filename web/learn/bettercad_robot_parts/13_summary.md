---
layout: lesson
title: Summary and Next Steps
author: Kevin McAleer
type: page
cover: /learn/bettercad_robot_parts/assets/banners/13_summary.jpg
date: 2026-11-21
previous: 12_project_geared_servo_deck.html
description: A recap of designing robot parts in BetterCad with the catalogue, pockets,
  mounts and gears, and where to go next
percent: 100
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

By the end of this lesson you will be able to recall the tools from the course, and use the checklist to plan your next robot part.

You started with a bare plate and a bag of bought parts. You finish with a printed deck that holds a servo and a microcontroller and drives a gear train, and you looked up almost no dimensions to get there.

![The finished servo deck](/learn/bettercad_robot_parts/assets/project_done.jpg){:class="img-fluid w-100"}

---

## What you learned

| Lesson | The idea to keep |
|---|---|
| Reference bodies | Bought parts live in the model to design around, and never print |
| The catalogue | Servos, motors, boards, sensors and bearings are one click away |
| Measure before you trust | Checked parts follow a drawing. The rest need callipers |
| Cutting a pocket | One command makes a sketch, a cut at a sliding fit, and the screw holes |
| Standoffs | Pick a board's pattern and its screw comes with it |
| Horn mounts | Bolt to the horn. Never print splines |
| Your first gear | Module is the tooth size, and pitch diameter is module times teeth |
| Bores and small gears | D-flat for motors, horn bore for servos, profile shift under 17 teeth |
| Meshing gears | A mating gear sits at the right distance and follows its partner |
| Racks, pulleys and belts | Racks slide, belts reach, and stock belts set the spacing |
| Mesh to editable | Recognise features turns an STL's holes into steps you can change |
{:class="table table-single"}

## A checklist for any robot part

Before you press print on a part that holds something:

- Is every bought part in the model as a reference?
- Did you measure the ones the catalogue hasn't checked?
- Are pockets cut with a fit, and holes made with the Hole tool, so your printer's calibration applies?
- Do gears that mesh share a module, and has the small one got 17 teeth or a profile shift?
- With the references hidden, is what's left exactly what you mean to print?

## Quick reference

| To do this | Use |
|---|---|
| Bring in a part you own | Insert, Import as a reference |
| Place a catalogue part | Insert, Catalogue. <kbd>T</kbd> turns it |
| Cut its pocket | Right-click the part, Cut pocket |
| Add standoffs or a horn mount | Insert, Mount |
| Make a gear, rack or pulley | Gear, then choose the Form |
| Add a meshed partner | Right-click a gear, Add mating gear or Add rack |
| Mesh two existing gears | Select both, <kbd>Ctrl</kbd>+<kbd>K</kbd>, Mesh gears |
| Fit a belt | Select two pulleys, right-click, Fit a belt |
| Change a gear later | Right-click its step on the timeline, Edit gear |
| Make an STL's holes editable | Right-click it, Recognise features |
{:class="table table-single"}

## What's Next

Your deck's gears mesh on screen, but they don't turn yet. BetterCad can do that too. The **Assemble** tab has joints, poses, a clash check that sweeps every joint through its range, and a URDF export for simulators.

- [Make It Move with BetterCad](/learn/bettercad_joints_urdf/) is the next course. It builds a two-joint arm, and one lesson comes back to this deck to make its gears drive each other

Also worth a look:

- [Learn BetterCad](/learn/bettercad/) if you skipped the basics
- [Print It Right with BetterCad](/learn/bettercad_printing/) to calibrate the fits and holes this course relies on
- [Wire It Up - Electronics in Snakie](/learn/snakie_electronics/) to wire the servo and Pico you've just mounted
- [Getting Started with Snakie](/learn/snakie_getting_started/) to write the MicroPython that sweeps the servo

And for build videos, robot projects and livestreams, subscribe to [Kevin McAleer on YouTube](https://www.youtube.com/c/kevinmcaleer28).

---

## Try it Yourself

1. Design a mount for a part that isn't in the catalogue. Find or measure its size, model it roughly from Library shapes, make it a reference, and build round it.
2. Design a two-stage gearbox: two 3 : 1 pairs for 9 : 1 overall. How small can you make it?
3. Design a gripper: one servo, a gear, and two racks that move in opposite directions.

## Check your understanding

{% quiz %}
? Which two things let your printer's calibration apply to a mount automatically?
- [ ] Reference bodies and the catalogue
- [x] Cutting pockets with a fit and making holes with the Hole tool
- [ ] Using module 1 gears and flanged pulleys
> Cut pocket does both for you: a sliding fit for the opening and printed clearance for the screws.

? What is the safest way to attach a printed part to a servo?
- [ ] Print the splines into the part
- [ ] Glue it to the shaft
- [x] Bolt it to the horn that came with the servo
> A horn mount or a gear's servo horn bore gives you the recess and the screw holes.

? What do two gears need in common to mesh?
- [ ] Their width
- [ ] Their bore
- [x] Their module
> And the same pressure angle, which a mating gear takes from its partner.
{% endquiz %}
