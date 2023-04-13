---
layout: lesson
title: Controlling LEDs with PWM
type: page
description: Learn how to control LEDs using Pulse Width Modulation (PWM) with the Raspberry Pi Pico and MicroPython.
---

## Overview

Welcome to Lesson 6 of the `Raspberry Pi Pico with MicroPython - GPIO Mastery` course. In this lesson, we will learn how to control LEDs using Pulse Width Modulation (PWM) with the Raspberry Pi Pico and MicroPython.

---

## Course Content

In this lesson, you will learn:

* How PWM can be used to control the brightness of an LED
* How to set up an LED circuit with the Raspberry Pi Pico
* How to control the brightness of an LED using PWM and MicroPython

---

## Key Results

After you have completed this lesson, you will know how to control the brightness of an LED using Pulse Width Modulation (PWM) with the Raspberry Pi Pico and MicroPython. You will be able to set up an LED circuit and write MicroPython code to control the LED's brightness using PWM.

---

## What you'll need

To follow this lesson, you will need:

* A computer, tablet, or phone to read the lesson material from
* A Raspberry Pi Pico board
* An LED, a resistor, and jumper wires for hands-on practice
* A breadboard

---

## Digital Input and Output

Digital input and output are used to read or write binary values (0s and 1s) from or to the Raspberry Pi Pico pins. You can use digital input and output to control various types of devices, such as LEDs, motors, and sensors.

## Configuring Raspberry Pi Pico Pins for Digital I/O

To use a pin on the Raspberry Pi Pico for digital input or output, you need to configure it for digital I/O. This is done using the `Pin` class in MicroPython. To configure a pin for digital output, you can use the following code:

```python
from machine import Pin

# Configure pin 0 for digital output
led = Pin(0, Pin.OUT)
```

To set the value of a digital output pin, you can use the value() method of the Pin class. For example, to turn on an LED connected to pin 0, you can use the following code:

```python
from machine import Pin

# Configure pin 0 for digital output
led = Pin(0, Pin.OUT)

# Turn on the LED
led.value(1)
```

---

## Using PWM to Control the Brightness of an LED

PWM can be used to control the brightness of an LED. PWM works by rapidly turning the LED on and off at a specific frequency, and varying the duty cycle (the percentage of time the LED is on) to control the brightness of the LED.

To use PWM to control the brightness of an LED connected to a Raspberry Pi Pico pin, you can use the `PWM` class in MicroPython. To set up a PWM pin, you can use the following code:

```python
from machine import Pin, PWM

# Configure pin 0 for PWM
led_pwm = PWM(Pin(0))
```

To control the brightness of the LED, you can use the `duty()` method of the `PWM` class. The `duty()` method takes a value between 0 (LED off) and 1023 (LED fully on) to set the duty cycle of the PWM signal. For example, to set the LED brightness to 50%, you can use the following code:

```python
from machine import Pin, PWM

# Configure pin 0 for PWM
led_pwm = PWM(Pin(0))

# Set the LED brightness to 50%
led_pwm.duty(512)
```

---

## Conclusion

In this lesson, you learned about digital input and output, how to configure Raspberry Pi Pico pins for digital I/O, and how to use PWM to control the brightness of an LED. You can use this knowledge to control various types of devices in your Raspberry Pi Pico projects, and create dynamic and interactive projects with LEDs, motors, and sensors.

---

In this lesson, you will learn how to control the brightness of an LED using Pulse Width Modulation (PWM) with the Raspberry Pi Pico and MicroPython. You will learn how to set up an LED circuit and write MicroPython code to control the LED's brightness using PWM.

---
