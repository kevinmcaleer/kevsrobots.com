---
layout: lesson
title: Controlling LEDs with PWM
author: Kevin McAleer
type: page
cover: /learn/micropython_gpio/assets/raspberry_pi_pico_gpio.jpg
date: 2023-04-14
previous: 05_pwm.html
next: 07_motors.html
description: Learn how to control LEDs using Pulse Width Modulation (PWM) with the
  Raspberry Pi Pico and MicroPython.
percent: 35
duration: 2
quiz_total: 33
navigation:
- name: Raspberry Pi Pico with MicroPython - GPIO Mastery
- content:
  - section: Overview
    content:
    - name: Introduction
      link: 00_intro.html
  - section: Introduction to GPIO Pins
    content:
    - name: GPIO Pin Types
      link: 01_gpio_pin_types.html
    - name: Pin Numbering
      link: 02_pin_numbering.html
    - name: Voltage Levels
      link: 03_voltage_levels.html
  - section: Digital I/O and PWM
    content:
    - name: Digital I/O
      link: 04_digital_io.html
    - name: Pulse Width Modulation (PWM)
      link: 05_pwm.html
    - name: Controlling LEDs with PWM
      link: 06_leds.html
    - name: Controlling Motors with PWM
      link: 07_motors.html
  - section: Analog I/O (ADC)
    content:
    - name: Analog Input (ADC)
      link: 08_adc.html
    - name: Using Potentiometers
      link: 09_potentiometers.html
    - name: Using Temperature Sensors
      link: 10_temperature_sensors.html
  - section: UART, I2C, and SPI
    content:
    - name: Introduction to UART
      link: 11_uart.html
    - name: Introduction to I2C
      link: 12_i2c.html
    - name: Introduction to SPI
      link: 13_spi.html
  - section: Interfacing with Servos and Ultrasonic Range Finders
    content:
    - name: Introduction to Servos
      link: 14_servos.html
    - name: Ultrasonic Range Finders
      link: 15_ultrasonic_range_finders.html
  - section: Ground Pins
    content:
    - name: Ground Pins
      link: 16_ground_pins.html
  - section: Summary and Review
    content:
    - name: Recap and Review
      link: 17_summary.html
---


## Overview

Time to put PWM to work. In this lesson you'll wire up an LED and make it fade smoothly up and down.

---

## What you'll need

* A Raspberry Pi Pico
* An LED and a 220Ω resistor
* A breadboard and jumper wires

---

## Wiring the LED

1. Connect `GP15` (physical pin 20) to one end of the 220Ω resistor
2. Connect the other end of the resistor to the LED's **long** leg (the anode)
3. Connect the LED's **short** leg (the cathode) to a `GND` pin

The resistor limits the current so the LED and the Pico pin aren't damaged.

---

## Turning the LED on and off

Before using PWM, check the wiring with a plain digital output:

```python
from machine import Pin

led = Pin(15, Pin.OUT)
led.value(1)
```

---

## Using PWM to control the brightness

```python
from machine import Pin, PWM
from time import sleep

led = PWM(Pin(15))
led.freq(1000)

while True:
    # fade up
    for duty in range(0, 65536, 1024):
        led.duty_u16(duty)
        sleep(0.01)
    # fade down
    for duty in range(65535, -1, -1024):
        led.duty_u16(duty)
        sleep(0.01)
```

- `led.freq(1000)` switches the LED on and off 1000 times a second, far too fast to see
- The first loop steps the duty from 0 up to 65535 in steps of 1024, so the LED brightens
- The second loop steps it back down again

> ## Brightness isn't linear
>
> Your eyes notice changes at low brightness much more than at high brightness, so the fade looks quick at the start and slow at the end. Squaring the value (`led.duty_u16(duty * duty // 65535)`) gives a smoother-looking fade.

---

## Try it yourself

Add a potentiometer later in the course (lesson 9) and use its reading to set the LED brightness directly.

{% quiz %}
? Why does the LED need a resistor?
- [ ] To make PWM work
- [x] To limit the current through the LED and the GPIO pin
- [ ] To convert 5V to 3.3V
> Without a resistor the LED would draw too much current and could damage itself and the pin.

? Which leg of the LED goes towards GND?
- [ ] The long leg (anode)
- [x] The short leg (cathode)
> Current flows from the anode (long leg) to the cathode (short leg).
{% endquiz %}

---
