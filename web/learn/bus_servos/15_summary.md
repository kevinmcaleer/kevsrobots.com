---
layout: lesson
title: Summary
author: Kevin McAleer
type: page
cover: /learn/bus_servos/assets/banners/15_summary.jpg
date: 2026-11-24
previous: 14_troubleshooting.html
description: Look back at the journey from one bus servo to a six-joint arm with a
  web control panel, keep the ideas that matter and find where to go next
percent: 100
duration: 4
quiz_total: 19
date_updated: 2026-11-24
navigation:
- name: Bus Servos and the Buddy Robot Arm
- content:
  - section: Introduction
    content:
    - name: Introduction
      link: 00_intro.html
  - section: Your first bus servo
    content:
    - name: Why bus servos
      link: 01_why_bus_servos.html
    - name: Parts and wiring
      link: 02_parts_and_wiring.html
    - name: The library and your first move
      link: 03_the_library_and_first_move.html
    - name: Reading feedback
      link: 04_reading_feedback.html
  - section: Two servos and beyond
    content:
    - name: Servo IDs and chaining
      link: 05_servo_ids_and_chaining.html
    - name: The wave demo
      link: 06_the_wave_demo.html
    - name: Teach and repeat
      link: 07_teach_and_repeat.html
    - name: Gearing and choosing servos
      link: 08_gearing_and_choosing_servos.html
  - section: The Buddy arm
    content:
    - name: Meet Buddy
      link: 09_meet_buddy.html
    - name: Calibrating offsets
      link: 10_calibrating_offsets.html
    - name: Smooth synchronised moves
      link: 11_smooth_synchronised_moves.html
    - name: Kinematics
      link: 12_kinematics.html
  - section: Control panel and wrap-up
    content:
    - name: The web control panel
      link: 13_web_control_panel.html
    - name: Troubleshooting
      link: 14_troubleshooting.html
    - name: Summary
      link: 15_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

You started with one servo on a wire and finished with a robot arm that you can pose by hand, ask to put its gripper anywhere in reach, and drive from your phone. The same idea carries all the way: a servo that talks back is a servo you can trust.

---

## What you built

| Step | Lessons | What was added |
| --- | --- | --- |
| One servo | 1 to 4 | Wiring, the library, a first move and feedback |
| Two servos | 5 and 6 | IDs, a chain and the wave demo |
| Teach mode | 7 and 8 | Record and replay by hand, gearing, limits and a temperature check |
| The Buddy arm | 9 to 12 | Six joints, calibration, synchronised moves and kinematics |
| Control panel | 13 | A web page and JSON API served from the Pico |
{:class="table table-single"}

```mermaid
flowchart LR
    A["One servo"] --> B["Two on a chain"]
    B --> C["Teach and repeat"]
    C --> D["Buddy: six joints"]
    D --> E["Web control panel"]
```

## The ideas to keep

- **A bus is one cable and many addresses.** Unique IDs, set one servo at a time, are what make chaining work
- **Feedback changes everything.** Read the position, check for `None`, and wait until a move has actually finished
- **Torque off is a teaching tool.** A limp servo still reports where it is
- **Match gearing to the joint.** Torque at the shoulder and elbow, speed at the wrist and gripper
- **Calibrate once, then think in degrees.** `offset_deg` and `sign` turn raw servo angles into your own frame
- **Make joints arrive together.** Scale each speed by its distance
- **Say where, not how.** Kinematics turns "hand there" into six angles
- **Layers pay off.** The web page only talks to the server, the server to `buddy.py`, and `buddy.py` to the driver

## What's next

Here is where to go from here.

- [Building a Robot Arm with Raspberry Pi and PCA9685](/learn/pca9685/) is the hobby-servo route to an arm, which makes a great comparison with what you built here
- [Open Duck Mini](/learn/open_duck_mini/) is a 14-servo walking robot that uses the same STS3215 servos
- [Learn ROS with me](/learn/learn_ros/) takes robot control further, and an arm like Buddy is a natural thing to bring into it
- The code lives at [github.com/kevinmcaleer/sts3215](https://github.com/kevinmcaleer/sts3215). Read it, change it and send ideas back
- [Kev's YouTube channel](https://www.youtube.com/@kevinmcaleer28) has the Buddy build and new projects as they happen

Pick one small job for your arm, such as moving a bottle top from one circle to another, and use everything: calibrate, plan the poses, and make the moves smooth.

---

## Check your understanding

{% quiz %}
? What lets many bus servos share one cable?
- [ ] Each one has its own power wire
- [x] Each has a unique ID, and commands carry that ID
- [ ] They take turns by pulse width
> The ID in each packet decides which servo acts.

? Which servo gearing suits the shoulder and elbow of an arm?
- [ ] High speed and low torque
- [x] High torque, even if it is slower
- [ ] It doesn't matter
> Those joints carry the most load, so torque matters more than speed.

? What does `move_all_sync` do that `move_all` doesn't?
- [ ] It sets the servo IDs
- [x] It scales each joint's speed so every joint arrives at the same time
- [ ] It switches the torque off
> Each speed is scaled by the joint's travel distance.
{% endquiz %}
