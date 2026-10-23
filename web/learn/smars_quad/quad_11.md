---
layout: lesson
title: Check Rotation
author: Kevin McAleer
type: page
cover: /learn/smars_quad/assets/how_to_build_a_quad.jpg
date: 2023-03-04
previous: quad_10.html
next: quad_12.html
description: Check the servo rotation
percent: 72
duration: 1
quiz_total: 5
navigation:
- name: Build a SMARS Quad Walking Robot
- content:
  - section: Overview
    content:
    - name: SMARS Quad robot Overview
      link: 00_intro.html
  - section: Print parts
    content:
    - name: Print the Parts
      link: quad_01.html
    - name: Servo Holders
      link: quad_02.html
    - name: Fit the servos
      link: quad_03.html
  - section: Assemble robot
    content:
    - name: Servo Frame
      link: quad_04.html
    - name: Servo Arms
      link: quad_05.html
    - name: Assemble arms
      link: quad_06.html
    - name: Calibrate Servos
      link: quad_07.html
    - name: Servo Horns
      link: quad_08.html
    - name: Print Feet
      link: quad_09.html
    - name: Assemble Feet
      link: quad_10.html
    - name: Check Rotation
      link: quad_11.html
  - section: Wire and code
    content:
    - name: Code
      link: quad_12.html
    - name: Wiring
      link: quad_13.html
    - name: Make it walk
      link: quad_14.html
---


## 11 Check the servo rotation

You can check the precise angle of these later using the `test.py` program. The foot should be able to rotate between 50 & 150 degrees.

![instruction image](assets/servorange.png){:class="img-fluid w-25"}
![instruction image](assets/rangeofmotion.png){:class="img-fluid  w-25"}

![Check the servo rotation](assets/instruction11.png){:class="img-fluid  w-25"}

---

## Check your understanding

{% quiz %}
? Why check the servo can move equally in both directions before screwing on the horn?
- [x] So the leg can swing the same amount forwards and backwards
- [ ] To test the battery
- [ ] Servos only work in one direction
> If the horn goes on off-centre, the leg will run out of movement on one side.

? Between which angles should each foot be able to rotate?
- [ ] 0 and 30 degrees
- [x] 50 and 150 degrees
- [ ] 0 and 360 degrees
{% endquiz %}
