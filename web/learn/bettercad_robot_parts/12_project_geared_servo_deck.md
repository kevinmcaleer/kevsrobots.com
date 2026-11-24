---
layout: lesson
title: Project - A Geared Servo Deck
author: Kevin McAleer
type: page
cover: /learn/bettercad_robot_parts/assets/banners/12_project_geared_servo_deck.jpg
date: 2026-11-21
previous: 11_from_mesh_to_editable_part.html
next: 13_summary.html
description: Build a 3D printable deck in BetterCad that holds an SG90 servo and a
  Raspberry Pi Pico and drives a 2 to 1 gear pair
percent: 91
duration: 8
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

By the end of this lesson you will be able to design a deck that holds an SG90 servo and a Raspberry Pi Pico and drives a 2 to 1 gear pair.

Time to put the whole course on one plate. You'll build a deck that could be the top of a small robot: a micro servo dropped into a pocket, a Pico on standoffs, and a pair of gears that halve the servo's speed and double its strength. Put a sensor or a head on the big gear and you have a pan mount with finer, stronger movement than the bare servo gives.

If you followed lessons 2, 4 and 5 in one model, you've done the first half already. Skip to step 4.

![The finished servo deck](/learn/bettercad_robot_parts/assets/project_done.jpg){:class="img-fluid w-100"}

---

## The plan

| Part | What it is | Printed? |
|---|---|---|
| Deck | A 120 × 100 × 4 mm plate with a servo pocket, four standoffs and a gear post | Yes |
| Drive gear | 24 teeth, module 1, bolted to the servo's horn | Yes |
| Output gear | 48 teeth, turning on an M3 screw through the post | Yes |
| SG90 servo | From the catalogue | No, reference |
| Raspberry Pi Pico | From the catalogue | No, reference |
{:class="table table-single"}

The numbers that tie it together:

```python
module = 1
drive_teeth = 24
output_teeth = 48

ratio = output_teeth / drive_teeth                         # 2 : 1
centre_distance = module * (drive_teeth + output_teeth) / 2   # 36 mm

servo_shaft_x = -30
post_x = servo_shaft_x + centre_distance                   # 6 mm
```

So the servo's shaft goes at X = -30 and the post for the big gear at X = 6.

## Step 1: the deck and the servo

1. Start a new model. Add a **Box**, size `120`, `100`, `4`, and rename it `Deck`.
2. **Insert**, **Catalogue**, **SG90 micro servo**. Click the top of the deck towards the left.
3. In Properties set the servo's **Position** to X `-35.2`, Y `0`, Z `4`.

The shaft is 5.2 mm off the servo's centre, so it's now at X = -30.

## Step 2: the pocket

1. Right-click the servo in the Browser and choose **Cut pocket**.

![The servo in its pocket](/learn/bettercad_robot_parts/assets/pocket.jpg){:class="img-fluid w-100"}

## Step 3: the Pico

1. **Insert**, **Mount**. Click the deck in the open space at the front right.
2. Set **Layout** to **Raspberry Pi Pico (47 × 11.4 mm)**.
3. Choose **Insert** or **Through** for the holes, then click **OK**.
4. From the Catalogue, place a **Raspberry Pi Pico** on the deck, move it over the standoffs and set its **Z** to `10`.

![The Pico on its standoffs](/learn/bettercad_robot_parts/assets/pico_on.jpg){:class="img-fluid w-100"}

## Step 4: the drive gear

1. Turn the view so you can see the top of the servo's shaft.
2. Click **Gear**, then click the round end of the shaft.

The gear stands on the shaft, centred on it.

3. Set **Teeth** to `24` and press <kbd>Tab</kbd>.
4. Set **Bore** to **Servo horn**.

![The drive gear on the servo, with a servo horn bore](/learn/bettercad_robot_parts/assets/drive_gear.jpg){:class="img-fluid w-100"}

5. Click **OK**.

## Step 5: the output gear

1. Right-click **Gear 24T** in the Browser and choose **Add mating gear**.
2. Set **Teeth** to `48`.
3. Leave **Angle** at `0` so it sits along X.
4. It copied the horn bore, which is wrong for this gear. Set **Bore** to **Round** and type `3.4`, a clearance hole for an M3 screw.

![The output gear, meshed, with the ratio shown](/learn/bettercad_robot_parts/assets/mate.jpg){:class="img-fluid w-100"}

The panel confirms it: "Meshes with Gear 24T, module 1. 2 : 1, the small gear turns twice as fast."

5. Click **OK**.

## Step 6: the gear post

The big gear is floating 14 mm above the deck. It needs something to stand on.

1. Open the **Library** and click **Cylinder**.
2. Set its size to `8`, `8`, `14`.
3. Set its **Position** to X `6`, Y `0`, Z `4`.
4. Rename it `Gear post`.

Hide the big gear (<kbd>Shift</kbd>+<kbd>H</kbd>) to check the post is where the gear's bore was.

The post is a separate body. Join it to the deck so they print as one piece:

5. Click **Deck** in the Browser, then <kbd>Ctrl</kbd>-click **Gear post**.
6. Press <kbd>Ctrl</kbd>+<kbd>G</kbd> to group them.

Now the axle hole:

7. Press <kbd>H</kbd> for the Hole tool and click the top of the post.
8. Leave **Size** at **M3**, set **Hole X** and **Hole Y** to `0`, and keep **Through all** ticked.

![An M3 hole through the gear post](/learn/bettercad_robot_parts/assets/post_hole.jpg){:class="img-fluid w-100"}

9. Click **OK**.

The hole runs down through the post and out of the bottom of the deck. An M3 screw goes up from underneath, through the post and the gear, and a locking nut on top holds the gear on while leaving it free to turn.

10. Right-click the hidden gear in the Browser and choose **Unhide all**.

## Step 7: check it

Look down from the top with the view cube.

![The deck from above](/learn/bettercad_robot_parts/assets/project_top.jpg){:class="img-fluid w-100"}

- The two gears' teeth interlock at one point, and touch nowhere else.
- The big gear clears the Pico and the edge of the deck.
- The servo's tabs sit over the two small screw holes.

With nothing selected, click **Check clashes** in Properties. It lists any bodies that overlap. Reference bodies are left out unless you tick **Reference bodies**.

Now hide the two references to see exactly what you'll print.

![The printed parts only](/learn/bettercad_robot_parts/assets/printed_only.jpg){:class="img-fluid w-100"}

Properties puts the three printed parts at about 73 g of PLA at 15% infill.

## Step 8: export

1. Press <kbd>Ctrl</kbd>+<kbd>E</kbd>.
2. Choose **3MF** and **Each on its own**.

![Exporting each printed part to its own file](/learn/bettercad_robot_parts/assets/export.jpg){:class="img-fluid w-100"}

You'll see the warning from lesson 1 about two hole shapes. Those are the servo and the Pico being left out, which is what you want. Don't click **Group them now**.

3. Click **Export**.

You get the deck and the two gears as separate files, ready for your slicer. Print the gears flat. Print the deck the way it's drawn, standoffs up.

## Put it together

1. Push a cross-shaped horn onto the servo and bolt the 24-tooth gear to it with M2 screws.
2. Drop the servo into its pocket and fix it with two M2 screws.
3. Feed an M3 screw up through the deck and the post, drop the big gear on, and add a locking nut. Leave it just loose enough to spin.
4. Screw the Pico to its standoffs.

Compare yours with the finished file: [servo-deck.bcad](/learn/bettercad_robot_parts/assets/servo-deck.bcad).

---

> **Teachers**: the three printed parts weigh about 73 g of PLA at 15% infill according to the lesson, so budget filament and printer time per group, and print the deck and gears across a later session. Building the deck needs a servo, a Pico and M2 and M3 screws for every group. Learners without kit can finish the design, checks and export on screen, and compare with the downloadable servo-deck.bcad.

## Try it Yourself

1. Edit **Gear 24T** and change it to 16 teeth. Does the 48-tooth gear follow? What's the new ratio, and is the post still in the right place?
2. Change the pattern to a **Raspberry Pi Zero 2 W**. Does it still fit on the deck?
3. Add a horn mount or four screw holes to the top of the big gear so a sensor bracket can bolt on.
4. Make the deck's width and depth parameters, as you did for the phone stand in Learn BetterCad.

## Common Issues

- **Problem**: The gear landed on the servo's body, not its shaft.
- **Solution**: Undo, zoom in on the shaft and click its round end face.
- **Why**: The gear stands on whichever flat face you click. The shaft's end is a small target.

- **Problem**: The post isn't under the big gear.
- **Solution**: Check the servo's X is -35.2 and the mating gear's **Angle** is 0. Then the post belongs at X = 6.
- **Why**: The post's place is the shaft's X plus the 36 mm centre distance.

- **Problem**: The gears mesh on screen but bind when printed.
- **Solution**: Edit both gears and raise **Backlash** under More settings by 0.05 mm, and check the big gear's nut isn't done up tight.
- **Why**: Printed teeth come out slightly fat, and a clamped gear can't turn.

- **Problem**: Ctrl+G grouped the wrong things.
- **Solution**: Undo. Select only **Deck** and **Gear post** before grouping.
- **Why**: Group joins every selected solid into one body.

## Check your understanding

{% quiz %}
? Why does the gear post go at X = 6?
- [ ] It's the middle of the deck
- [x] The shaft is at -30 and the gears' centres are 36 mm apart
- [ ] The Hole tool only works there
> Centre distance is module times the sum of the teeth, divided by two: 1 × (24 + 48) / 2.

? The servo sweeps 180°. How far does the 48-tooth gear turn?
- [ ] 360°
- [ ] 180°
- [x] 90°
> A 2 : 1 reduction halves the angle and roughly doubles the torque.

? Which parts end up in the exported files?
- [ ] Everything in the model
- [x] The deck and the two gears
- [ ] Only the deck
> The servo and the Pico are reference bodies, so they're left out.
{% endquiz %}
