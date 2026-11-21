---
layout: lesson
title: Meshing Gears
author: Kevin McAleer
type: page
cover: /learn/bettercad_robot_parts/assets/banners/09_meshing_gears.jpg
date: 2026-11-21
previous: 08_bores_hubs_and_small_gears.html
next: 10_racks_pulleys_and_belts.html
description: Add a mating gear in BetterCad, read the gear ratio, and keep a pair
  meshed at the right centre distance as you change it
percent: 70
duration: 5
quiz_total: 42
date_updated: 2026-11-21
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

One gear does nothing. Two gears are a machine. The hard part of a gear pair is the distance between their centres: a fraction too close and they jam, a fraction too far and they skip. BetterCad places the second gear for you.

---

## Add a mating gear

1. Start a new model and make a gear with **16** teeth. Press <kbd>Enter</kbd>.
2. In the Browser, right-click **Gear 16T** and choose **Add mating gear**.

A second gear appears beside the first, already meshed, with its panel open.

3. Set **Teeth** to `32`.

![A mating gear's panel with the ratio shown. This pair is the 24 and 48 from the project](/learn/bettercad_robot_parts/assets/mate.jpg){:class="img-fluid w-100"}

The panel for a mating gear is a little different:

- There's no **Module**. A mating gear takes its partner's module and pressure angle, because it has to.
- **Angle** sets where it sits round the first gear. 0 puts it along X, 90 swings it a quarter of the way round. You can also click in the view to move it round.
- A line tells you what you've made: "Meshes with Gear 16T, module 1. 2 : 1, the small gear turns twice as fast."
- **Meshed** has an **Unmesh** button.

The mating gear copies its partner's bore to start with. Change it if the two sit on different things.

## Reading the ratio

The ratio is just the two tooth counts.

```python
driver_teeth = 16      # the gear on the motor
driven_teeth = 32      # the gear it turns

ratio = driven_teeth / driver_teeth    # 2.0, written 2 : 1

# The driven gear turns slower by the ratio...
servo_sweep = 180                       # degrees
output_sweep = servo_sweep / ratio      # 90 degrees

# ...and pushes harder by the same amount.
servo_torque = 1.8                      # kg·cm, a typical SG90
output_torque = servo_torque * ratio    # 3.6 kg·cm, before friction
```

Small gear driving big gear: slower and stronger. Big driving small: faster and weaker. And every pair of gears reverses the direction of the turn.

The centre distance follows from the same numbers:

```python
module = 1
centre_distance = module * (16 + 32) / 2    # 24 mm
```

You never need to type that, but it's worth knowing when you're planning where the axles go.

## It stays meshed

Here's the real payoff. A meshed gear **follows** its partner.

1. Press <kbd>Enter</kbd> to keep the pair.
2. Right-click the first gear's step on the timeline and choose **Edit gear**.
3. Change its teeth from 16 to 20.

The big gear moves outward to stay in mesh, and turns so its teeth still fall into the gaps. Move or turn the first gear and the second goes with it.

**Unmesh** cuts the link and leaves the gear where it is, free to move on its own.

## Mesh two gears you already have

If both gears exist already, select the two of them, press <kbd>Ctrl</kbd>+<kbd>K</kbd> and run **Mesh gears**. The newer gear moves beside the older one.

It only works if they can truly mesh. Try it with a module 1 gear and a module 2 gear:

![BetterCad refusing to mesh gears with different modules](/learn/bettercad_robot_parts/assets/mesh_refused.jpg){:class="img-fluid w-100"}

"These gears have different modules (1 and 2), so their teeth won't mesh". Fix the module on one of them and try again.

## Choosing tooth counts

A few habits that save reprints:

- Keep the small gear at **17 teeth or more** when you can, to avoid undercut.
- For a ratio you'll run hard, pick tooth counts that **don't divide evenly**, such as 17 and 36. The same teeth then don't meet on every turn and wear is spread out.
- Need a big ratio in a small space? Use two stages. Two 3 : 1 pairs give 9 : 1 and take less room than one 9 : 1 pair.

---

## Try it Yourself

1. Make a 20-tooth gear and add a mating gear with 50 teeth. What ratio does the panel report?
2. Set the mating gear's **Angle** to 90, then 180. Watch it swing round.
3. Edit the first gear and change its module to 1.5. What happens to the second?

## Common Issues

**Problem**: There's no Add mating gear in the menu.
**Solution**: Right-click a gear made with the Gear tool.
**Why**: BetterCad needs the gear's settings to build a partner. An imported gear STL has none.

**Problem**: The mating gear has holes I didn't ask for.
**Solution**: Change its **Bore**. It copied the first gear's.
**Why**: A new mate starts with its partner's settings.

**Problem**: Printed gears are too tight at the distance BetterCad chose.
**Solution**: Raise **Backlash** on both gears. Don't move them apart by hand.
**Why**: The centre distance is right. The teeth are printing fat.

## Check your understanding

{% quiz %}
? A 15-tooth gear drives a 45-tooth gear. What's the ratio?
- [ ] 1 : 3, the big gear turns faster
- [x] 3 : 1, the small gear turns three times as fast
- [ ] 30 : 1
> Divide the driven teeth by the driver's teeth.

? Why does a mating gear have no Module box?
- [ ] Mating gears don't have a module
- [ ] It's hidden under More settings
- [x] It takes its partner's module, because it must match to mesh
> It takes the partner's pressure angle too.

? You change the first gear's teeth after meshing. What does the second gear do?
- [x] Moves and turns to stay in mesh
- [ ] Stays where it was and now overlaps
- [ ] Is deleted
> A meshed gear follows its partner until you Unmesh it.
{% endquiz %}
