---
layout: lesson
title: Attach Tracks
author: Kevin McAleer
type: page
cover: /learn/smars/assets/how_to_build_a_smars.jpg
date: 2023-02-19
previous: smars10.html
next: smars12.html
description: The tracks provide all terrain grip for the robot
percent: 72
duration: 2
quiz_total: 8
navigation:
- name: Build a SMARS Robot
- content:
  - section: Overview
    content:
    - name: Introduction
      link: 00_intro.html
  - section: Print Parts
    content:
    - name: Print the Parts
      link: smars01.html
  - section: Assemble Robot
    content:
    - name: Solder wires to motors
      link: smars02.html
    - name: Fit the Motors
      link: smars03.html
    - name: Fit the Battery
      link: smars04.html
    - name: Fit Motors Holders
      link: smars05.html
    - name: Attach Wheels
      link: smars06.html
    - name: Test Motors
      link: smars07.html
  - section: Wiring up the Robot
    content:
    - name: Add the Arduino
      link: smars08.html
    - name: Add Motor Shield
      link: smars09.html
    - name: Insert Wires
      link: smars10.html
    - name: Attach Tracks
      link: smars11.html
  - section: Code
    content:
    - name: Load the code
      link: smars12.html
    - name: Arduino Code
      link: smars13.html
  - section: Summary
    content:
    - name: Summary and Review
      link: summary.html
---


## Attach Tracks

To give the wheels more traction on different surfaces you can either attach the 8 tracks to each wheel, or attach 16 tracks to each side, in a caterpillar arrangement:

![caterpillar tracks](assets/tracks.png){:class="img-fluid w-50"}

The tracks are attached to each other by using a piece of filament.

![Tracks](assets/caterpillar_tracks.png){:class="img-fluid w-50"}

---

## Troubleshooting

You may find that the tracks are a really tight fit when connecting up in the caterpillar track formation. There are a couple of solutions to this:

* print another track at 110% - 120% scaling on the Y Axis
* use TPU / flexible filament, as this can stretch
* use a pre-sized piece from [Thingiverse](https://www.thingiverse.com/thing:3195991)

## Check your understanding

{% quiz %}
? A motor spins backwards when it should go forwards. What's the easiest fix?
- [x] Swap that motor's two wires in the motor shield connector
- [ ] Rebuild the chassis
- [ ] Use a bigger battery
> Swapping the wires reverses the motor. You could also fix it in code.

? Your caterpillar tracks are too tight. Which of these could help?
- [ ] Print the tracks at 50% infill
- [x] Print a track scaled to 110-120% on the Y axis, or use flexible TPU filament
- [ ] Remove the unpowered wheels
> A slightly longer track or a stretchy filament gives the loop the extra length it needs.
{% endquiz %}
