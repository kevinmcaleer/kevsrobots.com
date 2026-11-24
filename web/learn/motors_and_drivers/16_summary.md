---
layout: lesson
title: Summary
author: Kevin McAleer
type: page
cover: /learn/motors_and_drivers/assets/banners/16_summary.jpg
date: 2026-11-21
previous: 15_project_drive_base.html
description: Recap what you learned about DC motors, servos and steppers, and find
  the best next courses for Scout.
percent: 100
duration: 4
quiz_total: 49
date_updated: 2026-11-24
navigation:
- name: Motors and motor drivers
- content:
  - section: Introduction
    content:
    - name: Motors and motor drivers
      link: 00_intro.html
  - section: Motor basics
    content:
    - name: How motors work
      link: 01_how_motors_work.html
    - name: Why you need a motor driver
      link: 02_why_you_need_a_driver.html
    - name: Powering motors
      link: 03_powering_motors.html
  - section: DC motors
    content:
    - name: The H-bridge
      link: 04_the_h_bridge.html
    - name: Wiring a motor driver
      link: 05_wiring_a_motor_driver.html
    - name: Speed with PWM
      link: 06_speed_with_pwm.html
    - name: A motor class
      link: 07_a_motor_class.html
    - name: Driving two motors
      link: 08_driving_two_motors.html
    - name: Ramping and braking
      link: 09_ramping_and_braking.html
  - section: Servos
    content:
    - name: Servo basics
      link: 10_servo_basics.html
    - name: A servo class and calibration
      link: 11_a_servo_class_and_calibration.html
    - name: Continuous rotation servos
      link: 12_continuous_rotation_servos.html
  - section: Steppers and sensing
    content:
    - name: Stepper motors
      link: 13_stepper_motors.html
    - name: Measuring speed with encoders
      link: 14_measuring_speed_with_encoders.html
  - section: Project
    content:
    - name: Project - the Scout drive base
      link: 15_project_drive_base.html
    - name: Summary
      link: 16_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to recall the key ideas from the course and follow a debugging order when a motor misbehaves.

Well done! You started with a wheel that would not turn and finished with Scout, a drive base that drives a square, spins on the spot and looks around. Let's recap what made that possible.

## What you learned

| Idea | The key takeaway |
|---|---|
| Three motors | DC for speed, servos for angle, steppers for exact steps |
| Drivers | GPIO pins carry signals, never motor current |
| Back-EMF | Coils kick back when switched off, so use flyback diodes |
| Power | Separate motor supply, shared ground, beware brownouts |
| H-bridge | Four switches give direction, brake and coast |
| PWM | Duty cycle sets speed, and the dead zone sets the minimum |
| Classes | `Motor`, `DriveBase`, `Servo` and `Stepper` keep code tidy |
| Servos | A 50 Hz pulse of roughly 0.5 to 2.4 ms sets the angle, so calibrate |
| Encoders | Count ticks with an IRQ to turn commands into measured distance |
{:class="table table-single"}

The most useful habit from this course is the debugging order. When a motor misbehaves, check in this sequence:

1. Is the motor battery connected and charged?
2. Are the grounds joined?
3. Are the driver's enable pins (or SLP) in the right state?
4. Is the speed above the dead zone?
5. Is anything stalled, hot or shorted?

```mermaid
flowchart TD
    A["Motor misbehaves"] --> B{"Battery connected and charged?"}
    B -->|"No"| B1["Fix the supply"]
    B -->|"Yes"| C{"Grounds joined?"}
    C -->|"No"| C1["Join them"]
    C -->|"Yes"| D{"Enable pin or SLP correct?"}
    D -->|"No"| D1["Fix the pin"]
    D -->|"Yes"| E{"Speed above the dead zone?"}
    E -->|"No"| E1["Raise the speed or deadzone"]
    E -->|"Yes"| F["Look for a stall, heat or a short"]
```

## Try it Yourself

1. Without looking back, write the five debugging checks in order, then compare with the list above.
2. Pick one lesson from the course and explain it to someone else in two minutes. Teaching is the quickest way to find what you have not fully understood.
3. Challenge: choose one idea from the list below and plan which classes you would reuse and which you would write new.

## Things to try next with Scout

- Add an ultrasonic or time-of-flight sensor to the head, and sweep it to build a picture of the room
- Fit encoders and replace the timed moves with measured ones
- Add a line sensor and turn Scout into a line follower
- Control Scout over Bluetooth or WiFi

## What's Next

- [MicroPython Robotics](/learn/micropython_robotics/) takes Scout further with line following, an ultrasonic sensor, Bluetooth and WiFi control
- [Learn how to program SMARS with Arduino](/learn/smars_code/) shows the same motor ideas on a different platform, handy if you want to compare Arduino and MicroPython
- [Build your own BurgerBot](/learn/burgerbot/) is a simple 3D printed Pico and MicroPython robot that puts these skills to work
- [Building a Robot Arm with Raspberry Pi and PCA9685](/learn/pca9685/) shows how a PCA9685 board drives many servos at once, for arms and grippers

You can also revisit [Electronics basics](/learn/electronics_basics/) and [Learn MicroPython - The basics](/learn/micropython/) whenever you want to refresh the foundations.

Thanks for building along. Now go and put a wheel on something.

## Check your understanding

{% quiz %}
? A wheel will not move at low speed commands. What is the likely cause?
- [x] The speed is below the dead zone
- [ ] The grounds are joined
- [ ] The battery is too full
> Below the dead zone only a whine is heard, so raise the speed or set the deadzone.

? Which motor would you choose to point a sensor at an exact angle?
- [x] Servo
- [ ] DC motor with no feedback
- [ ] Fan motor
> A servo has a position loop built in, and Scout's head uses one.

? In the debugging order, what comes straight after checking the battery?
- [x] Check the grounds are joined
- [ ] Rewrite the code
- [ ] Replace the Pico
> Battery, grounds, enable pins, dead zone, then stalls and shorts.
{% endquiz %}
