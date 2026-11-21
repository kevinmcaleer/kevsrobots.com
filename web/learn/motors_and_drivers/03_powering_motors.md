---
layout: lesson
title: Powering motors
author: Kevin McAleer
type: page
cover: /learn/motors_and_drivers/assets/banners/03_powering_motors.jpg
date: 2026-11-21
previous: 02_why_you_need_a_driver.html
next: 04_the_h_bridge.html
description: Choose batteries, split motor and logic power, share a common ground
  and avoid brownouts on your Pico robot.
percent: 20
duration: 4
date_updated: 2026-11-21
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

Most "my motor code is broken" problems turn out to be power problems. This lesson is short, but it will save you the most debugging time of the whole course.

## Two supplies, one ground

Scout has two kinds of load. The Pico and sensors need a clean, steady 3.3 V. The motors need lots of current and make electrical noise. We give them separate supplies and join them at ground:

```text
USB (or regulated 5V) ---> Pico ---- GPIO signals ---> Driver logic
                            |                              |
                           GND ----------------------- GND (common ground)
                                                           |
4xAA battery pack  --------------------------------> Driver motor supply
```

The **common ground** is essential. Signals are voltages measured *relative to ground*, so if the Pico and the driver do not share a ground wire, the driver has no idea what a "1" from the Pico means. Wheels twitching randomly is the classic symptom.

## Choosing batteries

| Battery | Voltage | Notes |
|---|---|---|
| 4xAA alkaline | about 6 V fresh | Easy and safe, but voltage sags under load |
| 4xAA NiMH rechargeable | about 4.8 V | Strong under load, ideal for TT motors |
| 1S LiPo | 3.7 V | Compact and powerful, needs a proper charger |
| 2S LiPo | 7.4 V | Too much for TT motors without limiting, handle with care |
{:class="table table-single"}

TT motors are happy at 3 to 6 V, so a 4xAA pack is a great starting point.

**Battery safety, briefly:** if you use LiPo cells, charge them with a LiPo charger, never leave them charging unattended, and stop using a puffy or dented cell. Do not short the leads. Use a pack with a built-in protection circuit when you can.

## How much voltage reaches the motor?

Drivers lose some voltage as heat. An L298N drops roughly 2 V, while a DRV8833 drops a few tenths of a volt. That matters on a small battery:

```python
# Voltage that actually reaches the motors (approximate)
battery_v = 6.0        # 4 fresh alkaline AA cells

l298n_drop = 2.0       # the L298N's bipolar outputs are lossy
drv8833_drop = 0.4     # modern MOSFET outputs are far more efficient

print("L298N motor volts:  ", battery_v - l298n_drop)    # 4.0
print("DRV8833 motor volts:", battery_v - drv8833_drop)  # 5.6
```

Four volts will still turn a TT motor, just more slowly and with a bigger dead zone. We will measure that dead zone in lesson 6.

## Brownouts

A **brownout** is when the supply voltage dips so low that something resets. When a motor starts, it briefly draws several times its running current. A weak or shared supply sags, the Pico notices, and your program restarts in the middle of a move.

Prevention, in order of usefulness:

1. Power the motors from their own battery, not the Pico's 3V3 or VBUS pins
2. Use a fresh or charged battery with enough current capacity
3. Add a 100 uF capacitor across the motor supply near the driver
4. Start motors gently (lesson 9)

## Powering the Pico itself

While you learn, keep the Pico on USB. When Scout goes untethered, remember that the Pico's VSYS input takes up to 5.5 V, so a fresh alkaline pack at 6 V is too high. Use NiMH cells, or a small 5 V regulator module between the pack and the Pico.

## Try it Yourself

1. Measure your battery pack with a multimeter, with and without a motor running. How much does it sag?
2. Put the L298N's logic jumper settings in your notes: is the 5V regulator jumper in place? (More on this in lesson 5.)
3. Challenge: work out the stall current of both motors together, and check your battery pack can supply it.

## Common Issues

**Problem**: The motors twitch or the driver ignores the Pico.
**Solution**: Connect a ground wire between the Pico and the driver.
**Why**: Without a shared reference, the driver cannot read the Pico's signals.

**Problem**: The Pico resets when I start both motors at once.
**Solution**: Use a separate battery for the motors, add a capacitor, and ramp the speed up.
**Why**: The starting surge makes the supply voltage dip below what the Pico needs.
