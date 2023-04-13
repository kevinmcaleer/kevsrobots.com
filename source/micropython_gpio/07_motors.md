---
layout: lesson
title: Controlling Motors with PWM
type: page
description: Learn how to control motors using Pulse Width Modulation (PWM) with the Raspberry Pi Pico and MicroPython.
---

## Overview

Welcome to Lesson 7 of the `Raspberry Pi Pico with MicroPython - GPIO Mastery` course. In this lesson, we will learn how to control motors using Pulse Width Modulation (PWM) with the Raspberry Pi Pico and MicroPython.

---

## Course Content

In this lesson, you will learn:

* How PWM can be used to control the speed of a motor
* How to set up a motor circuit with the Raspberry Pi Pico
* How to control the speed of a motor using PWM and MicroPython

---

## Key Results

After you have completed this lesson, you will know how to control the speed of a motor using Pulse Width Modulation (PWM) with the Raspberry Pi Pico and MicroPython. You will be able to set up a motor circuit and write MicroPython code to control the motor's speed using PWM.

---

## Required Materials

To follow this lesson, you will need:

* A computer, tablet, or phone to read the lesson material from
* A Raspberry Pi Pico board
* A DC motor
* A motor driver (such as the L293D)
* Jumper wires
* A [breadboard](/resources/how_it_works/breadboards)
* A power supply

---

## Wiring the Circuit

To wire up the circuit, follow these steps:

1. Connect the Raspberry Pi Pico 5V pin to the breadboard power rail.
2. Connect the Raspberry Pi Pico GND pin to the breadboard GND rail.
3. Connect the motor driver VCC pin to the breadboard power rail.
4. Connect the motor driver GND pin to the breadboard GND rail.
5. Connect the motor driver input 1 pin to the Raspberry Pi Pico pin of your choice (e.g., pin 0).
6. Connect the motor driver input 2 pin to the Raspberry Pi Pico pin of your choice (e.g., pin 1).
7. Connect the motor to the motor driver output pins (e.g., OUT1 and OUT2).

---

## Writing the Code

To control the motor, you will need to write code using the `machine` and `time` modules in MicroPython. Here is some example code to get you started:

```python
from machine import Pin
import time

# Set up the pins for the motor driver inputs
input_1 = Pin(0, Pin.OUT)
input_2 = Pin(1, Pin.OUT)

# Set the motor direction to forward
input_1.value(1)
input_2.value(0)

# Set up the PWM output for the motor speed
motor_speed = Pin(2, Pin.OUT)
pwm = machine.PWM(motor_speed)
pwm.freq(1000)

# Set the motor speed to 50%
pwm.duty(512)

# Wait for 5 seconds
time.sleep(5)

# Stop the motor
input_1.value(0)
input_2.value(0)
pwm.deinit()
```

---

## Conclusion

In this lesson, you got hands-on practice working with motors using your Raspberry Pi Pico board. You learned how to wire up a circuit, and how to control the motor using MicroPython code. You can use this knowledge to create a variety of projects with your Raspberry Pi Pico board.

---
