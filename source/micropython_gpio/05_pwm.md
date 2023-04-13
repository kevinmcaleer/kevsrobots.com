---
layout: lesson
title: Pulse Width Modulation (PWM)
type: page
description: Learn about Pulse Width Modulation (PWM) and controlling devices like LEDs and motors using the Raspberry Pi Pico with MicroPython.
---

## Overview

Welcome to Lesson 5 of the `Raspberry Pi Pico with MicroPython - GPIO Mastery` course. In this lesson, we will explore Pulse Width Modulation (PWM) and how to control devices like LEDs and motors using the Raspberry Pi Pico with MicroPython.

---

## Course Content

In this lesson, you will learn:

* What Pulse Width Modulation (PWM) is and how it works
* How to configure Raspberry Pi Pico pins for PWM
* How to control devices like LEDs and motors using PWM and MicroPython

---

## Pulse Width Modulation

Pulse Width Modulation (PWM) is a technique used to control the amount of power delivered to a device, such as an LED or a motor. PWM works by rapidly turning the device on and off at a specific frequency, and varying the duty cycle (the percentage of time the device is on) to control the amount of power delivered to the device.

## Configuring Raspberry Pi Pico Pins for PWM

To use a pin on the Raspberry Pi Pico for PWM, you need to configure it for PWM. This is done using the `PWM` class in MicroPython. To configure a pin for PWM, you can use the following code:

```python
from machine import Pin, PWM

# Configure pin 0 for PWM with a frequency of 1000 Hz
pwm = PWM(Pin(0), freq=1000)
```

## Controlling Devices Using PWM

To control the amount of power delivered to a device using PWM, you can use the `duty()` method of the PWM class. The `duty()` method takes a value between 0 (device off) and 1023 (device fully on) to set the duty cycle of the PWM signal. For example, to set the LED brightness to 50%, you can use the following code:

```python
from machine import Pin, PWM

# Configure pin 0 for PWM with a frequency of 1000 Hz
pwm = PWM(Pin(0), freq=1000)

# Set the LED brightness to 50%
pwm.duty(512)
```

---

## Conclusion

In this lesson, you learned about Pulse Width Modulation (PWM), how to configure Raspberry Pi Pico pins for PWM, and how to control devices like LEDs and motors using PWM and MicroPython. You can use this knowledge to create dynamic and interactive projects with your Raspberry Pi Pico board.

---

## Key Results

After you have completed this lesson, you will have a solid understanding of Pulse Width Modulation (PWM) and how to control devices like LEDs and motors using the Raspberry Pi Pico with MicroPython. You will be able to configure pins for PWM, generate PWM signals, and control devices that require PWM input.

---
