---
layout: lesson
title: Test Motors
author: Kevin McAleer
type: page
cover: /learn/smars/assets/how_to_build_a_smars.jpg
date: 2023-02-19
previous: smars06.html
next: smars08.html
description: Test Motors to ensure they work ok
percent: 48
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


## Test Motors

Test the motors are working correctly by touching the positive and negative wires to the battery. They should spin enthusiastically!

---

## Troubleshooting

If the motors do not spin when connecting to the battery, check that the wires are still soldered to the motor and haven't come free when fitting (they are quite delicate). Also check the 9V battery is charged: a multimeter set to DC volts should read close to 9V. A flat battery is the most common reason a SMARS won't move.

---

![Test Motors](assets/wheels.png){:class="img-fluid w-50"}

## Check your understanding

{% quiz %}
? What's the difference between the powered and unpowered wheels?
- [x] Powered wheels are driven by the motors, unpowered wheels spin freely
- [ ] Powered wheels have batteries inside them
- [ ] There is no difference
> The unpowered wheels push onto the lugs on the chassis and just turn with the tracks.

? Your motors don't spin when you test them. What should you check first?
- [ ] The Arduino code
- [x] That the wires are still soldered on and the battery is charged
- [ ] The tracks
> At this stage there's no code involved, so it's down to the wires or the battery.
{% endquiz %}
