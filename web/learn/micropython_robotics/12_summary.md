---
layout: lesson
title: Course Summary and What's Next
author: Kevin McAleer
type: page
cover: /learn/micropython_robotics/assets/banners/12_summary.jpg
date: 2025-05-20
previous: 11_final_project.html
description: Reflect on what you've accomplished and explore ideas for taking your
  MicroPython robotics skills even further.
percent: 100
duration: 4
quiz_total: 36
date_updated: 2026-11-24
navigation:
- name: MicroPython Robotics Projects with the Raspberry Pi Pico
- content:
  - section: Introduction
    content:
    - name: Introduction to MicroPython Robotics Projects
      link: 00_intro.html
  - section: Motors and Movement
    content:
    - name: Controlling DC Motors with the Raspberry Pi Pico
      link: 02_dc_motors.html
    - name: Using H-Bridge Motor Drivers
      link: 03_hbridge_control.html
  - section: Sensors and Input
    content:
    - name: Line Following with IR Sensors
      link: 04_line_following.html
    - name: Obstacle Avoidance with Ultrasonic Sensors
      link: 05_ultrasonic_sensor.html
  - section: Robot Assembly
    content:
    - name: Building a Two-Wheel Drive Robot
      link: 06_build_robot.html
    - name: Servo Scanning with Ultrasonic Sensors
      link: 07_servo_scanning.html
  - section: Remote Control
    content:
    - name: Bluetooth Control with HC-05
      link: 08_bluetooth_control.html
    - name: Wi-Fi Control with the Raspberry Pi Pico W
      link: 09_wifi_control.html
  - section: Autonomous Behavior
    content:
    - name: Creating Simple Autonomous Behavior
      link: 10_autonomous_robot.html
  - section: Final Project
    content:
    - name: Final Project - Build Your Own Robot
      link: 11_final_project.html
  - section: Summary
    content:
    - name: Course Summary and What's Next
      link: 12_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to recap every part of the robot, say which lesson to revisit when something breaks, and choose your next project.

Congratulations on completing **MicroPython Robotics Projects with the Raspberry Pi Pico**! You've gone from single components to a robot that can sense, move, react, and even take orders from your phone. That's a real achievement in physical computing.

## What you've learned

```mermaid
flowchart TD
    M["Motors: PWM and H-bridge (lessons 2 and 3)"] --> R["robot.py"]
    S1["IR line sensor (lesson 4)"] --> B["Behaviours"]
    S2["Ultrasonic sonar.py (lesson 5)"] --> B
    SV["Servo scanning (lesson 7)"] --> B
    R --> B
    C["Bluetooth and Wi-Fi (lessons 8 and 9)"] --> B
    B --> P["Your own robot (lessons 10 and 11)"]
```

| If this goes wrong | Revisit |
|--------------------|---------|
| Motor will not turn, or always runs flat out | Lesson 2 |
| Wheels turn the wrong way | Lesson 3 |
| Robot loses the line | Lesson 4 |
| Distance readings are wrong or the Pico is damaged by ECHO | Lesson 5 |
| Resets, heat or power trouble | Lesson 6 |
| Servo angle is off or buzzing | Lesson 7 |
| Remote control not responding | Lessons 8 and 9 |
{:class="table table-single"}

The big ideas to keep:

- A Pico cannot run motors directly, so a **driver** switches the power
- **PWM** sets speed: duty of 65535 means fully on, and a servo angle is a pulse width
- **Sensors** give numbers, and you must calibrate and check their voltage levels
- Robots run a **sense, decide, act** loop
- Build in **safety stops** and test each part on its own

## What's next

Here are some ways to take your skills further:

- **Understand drivers deeply** with [Motors and motor drivers](/learn/motors_and_drivers/): motor classes, braking, servos, steppers and encoders
- **Go back to the foundations** with [Robotics 101](/learn/robotics_101/)
- **Build more MicroPython skills** with [GPIO Mastery](/learn/micropython_gpio/) and [Intermediate level MicroPython](/learn/intermediate_micropython/)
- **Try a complete robot kit** with [Build your own BurgerBot](/learn/burgerbot/) or the SMARS robot courses, such as [Learn how to program SMARS with Arduino](/learn/smars_code/)
- **Wire and test in a simulator** with [Wire It Up - Electronics in Snakie](/learn/snakie_electronics/)
- **Smarter robots**: add wheel encoders, try PID control, or add more sensors

## Share what you've built

Share your photos, code and write-ups on your favourite maker forums. Show what went wrong as well as what worked, since that's what other makers learn the most from.

## Try it Yourself

1. Write down the three biggest problems you fixed during the course and what caused each.
2. Draw your robot's wiring as a table, from memory, then compare it to the intro.
3. Pick one idea from What's next and write a plan for it using the checklist in lesson 11.

## Check your understanding

{% quiz %}
? What sets the speed of a DC motor driven through an L298N enable pin?
- [ ] The direction pins
- [x] The PWM duty cycle on the enable pin
- [ ] The motor wire colour
> PWM on ENA or ENB sets the average power the motor gets.

? Which loop describes how an autonomous robot works?
- [x] Sense, decide, act
- [ ] Wire, solder, test
- [ ] Pair, connect, drive
> The robot reads its sensors, decides, and then moves.

? Why should you build in a safety stop in remote-controlled robots?
- [ ] It makes the code shorter
- [ ] It saves memory
- [x] So the robot stops if the connection is lost
> The Bluetooth lesson stops the robot when commands stop arriving, and the Wi-Fi lesson uses short nudges.
{% endquiz %}
