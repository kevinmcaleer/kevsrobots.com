---
layout: lesson
title: Wiring a motor driver
author: Kevin McAleer
type: page
cover: /learn/motors_and_drivers/assets/banners/05_wiring_a_motor_driver.jpg
date: 2026-11-21
previous: 04_the_h_bridge.html
next: 06_speed_with_pwm.html
description: Wire an L298N or DRV8833 motor driver to a Raspberry Pi Pico with the
  same pin layout used across KevsRobots courses.
percent: 30
duration: 5
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

Time to build. We will wire Scout's two motors to an L298N driver, test one motor with a few lines of code, then look at the smaller DRV8833 as an alternative. Do the wiring with everything powered off.

## L298N wiring

The L298N has two channels. Channel A drives Scout's left motor, and channel B drives the right. This is the same layout as the [MicroPython Robotics](/learn/micropython_robotics/) course.

| Pico pin | L298N pin | Job |
|---|---|---|
| GP0 | IN1 | Left motor direction |
| GP1 | IN2 | Left motor direction |
| GP2 | ENA | Left motor speed (PWM) |
| GP3 | IN3 | Right motor direction |
| GP4 | IN4 | Right motor direction |
| GP5 | ENB | Right motor speed (PWM) |
| GND | GND | Common ground |
{:class="table table-single"}

Then the power side:

| Connection | Goes to |
|---|---|
| Battery pack positive | L298N 12V terminal (the motor supply input) |
| Battery pack negative | L298N GND terminal |
| Left motor wires | OUT1 and OUT2 |
| Right motor wires | OUT3 and OUT4 |
{:class="table table-single"}

Three things catch people out:

1. **Remove the ENA and ENB jumpers.** Most boards ship with small jumpers over the enable pins, which tie them permanently high. We need those pins free so the Pico can send PWM.
2. **Leave the 5V-enable jumper in** if your battery is under about 12 V. That powers the driver's logic from the battery. Do not use the board's 5V pin to power the Pico on a 4xAA supply, as the regulator needs more headroom.
3. **Join grounds.** The battery negative, the L298N GND and the Pico GND must all connect.

## Your first spin

Left motor only, full speed, one second each way. Because ENA is just a pin set high, this is the plain truth table from lesson 4:

```python
from machine import Pin
from time import sleep_ms

in1 = Pin(0, Pin.OUT)
in2 = Pin(1, Pin.OUT)
ena = Pin(2, Pin.OUT)

try:
    ena.value(1)                      # enable the bridge
    in1.value(1); in2.value(0)        # forward
    sleep_ms(1000)
    in1.value(1); in2.value(1)        # brake
    sleep_ms(500)
    in1.value(0); in2.value(1)        # reverse
    sleep_ms(1000)
finally:
    ena.value(0)                      # coast, whatever happens
    in1.value(0); in2.value(0)
```

Run it. The wheel should go forward, stop, reverse. If it goes the wrong way round, swap the two motor wires on OUT1 and OUT2. That is all "forward" means: whichever direction we decide. Scout's code will later get an `invert` option so you never have to rewire.

The `try` and `finally` is a habit worth forming. If you stop the program with Ctrl+C partway through, the `finally` block still switches the motor off.

## DRV8833 wiring

The DRV8833 is smaller, cooler and wastes far less battery voltage. It has no enable pins, so it needs only four Pico pins. We keep GP0, GP1, GP3 and GP4 so the layout matches the L298N:

| Pico pin | DRV8833 pin | Job |
|---|---|---|
| GP0 | AIN1 | Left motor, PWM |
| GP1 | AIN2 | Left motor, PWM |
| GP3 | BIN1 | Right motor, PWM |
| GP4 | BIN2 | Right motor, PWM |
| 3V3 | SLP (nSLEEP) | Held high to keep the chip awake |
| GND | GND | Common ground |
{:class="table table-single"}

The battery goes to VM (motor supply) and GND, and the motors go to AOUT1/AOUT2 and BOUT1/BOUT2. Many breakout boards already pull SLP high, so check yours. GP2 and GP5 are not used.

The code is just as short, but now the speed lives on the input pins:

```python
from machine import Pin, PWM
from time import sleep_ms

in1 = PWM(Pin(0))
in2 = PWM(Pin(1))
in1.freq(10000)
in2.freq(10000)

try:
    in1.duty_u16(32768)   # roughly half speed forward
    in2.duty_u16(0)
    sleep_ms(1000)
    in1.duty_u16(0)       # reverse at half speed
    in2.duty_u16(32768)
    sleep_ms(1000)
finally:
    in1.duty_u16(0)       # both low: coast
    in2.duty_u16(0)
```

Ratings to remember: the DRV8833 takes 2.7 to 10.8 V and about 1.5 A per channel, which is perfect for TT motors. The L298N takes 5 to 35 V and about 2 A per channel, but it runs warm.

## Try it Yourself

1. Run the test for the right motor too (GP3, GP4, GP5 on the L298N).
2. Change the delays so the motor runs for 3 seconds. Touch the L298N heatsink afterwards. Is it warm?
3. Challenge: put a 100 uF capacitor across the battery pack terminals at the driver. Does the motor start-up look any smoother? Does the Pico still stay awake?

## Common Issues

**Problem**: Nothing moves on the L298N.
**Solution**: Check the ENA jumper is removed and GP2 is connected, the battery is wired to 12V, and grounds are joined.
**Why**: With EN low the bridge is disabled, and with no common ground the inputs are meaningless.

**Problem**: The DRV8833 motor jerks once, then nothing.
**Solution**: Make sure SLP/nSLEEP is high, and that the battery voltage is above about 3 V.
**Why**: The chip sleeps whenever nSLEEP is low, and under-voltage protection shuts it off.

**Problem**: The driver gets very hot.
**Solution**: Check for a stalled wheel or shorted motor terminals, and let it cool.
**Why**: Stall current turns directly into heat in the driver chip.
