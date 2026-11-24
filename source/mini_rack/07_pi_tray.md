---
title: Create the Pi-Tray
description: Design a Raspberry Pi tray with mounting holes and cooling slots, then mirror it to make the other version
type: page
layout: lesson
cover: /learn/mini_rack/assets/banners/07_pi_tray.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to design a Raspberry Pi tray with mounting holes and ventilation slots, and create a second version by mirroring it.

Ahoy there makers! The Pi tray is the part that holds a Raspberry Pi in the rack. It is where the electronics meet the rack, so it is also where we have to think about heat and power.

Each Pi is held to its tray with four M2.5 nuts and bolts. In the sketch picture you will see four `2.5mm` holes laid out `58mm` by `49mm` apart, which is the standard mounting hole pattern for a full-size Raspberry Pi, on a board outline `85mm` by `56mm`.

Kev made a live stream for the Pi tray and the laser parts. Watch it first, then build along.

{% include youtubeplayer.html id="qnewKGO8EOc" %}

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

1. On the inside edges of the tray, use the Chamfer tool to bevel the edges:

    ![Extrude](/learn/mini_rack/assets/pi_tray_15_chamfer.png){:class="img-fluid w-100"}

    ![Extrude](/learn/mini_rack/assets/pi_tray_16_chamfer.png){:class="img-fluid w-100"}

## Creating the Pi 4 Tray

We can quickly create the Pi 4 version of the tray by mirroring the Pi 5 tray.

1. Create an offset plane 20mm from the front face - we will use this to mirror the body of the tray:

    
    ![Extrude](/learn/mini_rack/assets/pi_tray_17_plane.png){:class="img-fluid w-100"}

1. Use the Mirror tool to mirror the `tray_base` body:

    ![Extrude](/learn/mini_rack/assets/pi_tray_18_mirror.png){:class="img-fluid w-100"}

    ![Mirror](/learn/mini_rack/assets/pi_tray_19_mirror.png){:class="img-fluid w-100"}

## Heat and power

A Pi gets warm, and a rack packs several of them close together. The `cooling` sketch cuts ventilation holes in the base of the tray, so air can reach the underside and pass through. Please don't block them. Leave the slots in your design, and don't stack things on the cooling holes.

If you have a Pi on the tray, you can check how hot it is. Open a terminal on the Pi and run:

```bash
vcgencmd measure_temp
```

It prints the board temperature. If the number climbs a long way when the Pi is busy, add a fan or give the rack more space.

Power is the other risk. Always use the power supply made for your Pi model, and keep the cables untangled so a plug is never pulled sideways under strain. If you power the Pis from a 10 inch rack power strip, use one that is rated for the total load. Anything wired into mains electricity should be done by a qualified adult. A PoE switch can power Pis that have a PoE HAT, but check that the switch can supply enough power for every port in use.

> **Teachers**: This lesson has the most safety points of the course. Keep mains wiring out of lessons: use ready-made power strips only, and have an adult plug them in. Heat matters too. Check the filament's heat rating before printing trays that will sit next to a busy Pi, as plastics soften when warm. Allow one printer session per tray, and print one tray first to test the fit before printing a whole class set.

## Try it Yourself

1. After you cut the `holes` sketch, look at the picture and count how many holes there are. Which are the four for the Pi?
2. Add one more cooling slot in the `cooling` sketch. Does it still look balanced?
3. Challenge: the Pi 4 tray is made by mirroring the Pi 5 tray. Before you print, hold a real Pi next to the picture and check where the ports sit. Which tray does your Pi need?

## Common Issues

- **Problem**: The Pi does not fit the mounting holes.
- **Solution**: Check the hole spacing (`58mm` by `49mm`) and that your printer is not shrinking holes. Drill them out slightly if needed.
- **Why**: Printed holes often come out a little smaller than designed.

- **Problem**: The Mirror tool does not make a second body.
- **Solution**: Choose "Bodies" as the object type, select `tray_base`, and set the plane to the offset plane you made.
- **Why**: Mirror can work on features or on bodies, and the default is features.

- **Problem**: The Pi runs hot in the rack.
- **Solution**: Make sure the cooling slots are open and the rack is not against a wall. Consider a small fan.
- **Why**: A Pi throttles its speed when hot, and trapped warm air makes that worse.

## Check your understanding

{% quiz %}
? What holds the Raspberry Pi to the tray?
- [ ] Glue
- [x] Four M2.5 nuts and bolts
- [ ] Rack mount nuts
> The tray has four 2.5mm mounting holes for the Pi.

? How is the Pi 4 tray made from the Pi 5 tray?
- [ ] By scaling it
- [x] By mirroring the `tray_base` body about an offset plane
- [ ] By cutting new holes
> The lesson mirrors the body about an offset plane `20mm` from the front face.

? What is the `cooling` sketch for?
- [ ] Mounting the Pi
- [x] Ventilation slots in the base of the tray
- [ ] Adding chamfers
> The slots let air reach the Pi.
{% endquiz %}
