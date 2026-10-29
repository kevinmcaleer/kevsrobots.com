---
layout: lesson
title: Pulse Width Modulation (PWM)
author: Kevin McAleer
type: page
cover: /learn/micropython_gpio/assets/raspberry_pi_pico_gpio.jpg
date: 2023-04-14
previous: 04_digital_io.html
next: 06_leds.html
description: Learn about Pulse Width Modulation (PWM) and controlling devices like
  LEDs and motors using the Raspberry Pi Pico with MicroPython.
percent: 30
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

The Pico can't output "half" a voltage from a GPIO pin, but it can fake it. In this lesson you'll learn how Pulse Width Modulation works and how to set it up in MicroPython.

---

## Pulse Width Modulation

PWM switches a pin on and off very quickly. Two numbers describe the signal:

- **Frequency** - how many on/off cycles happen per second, in hertz (Hz)
- **Duty cycle** - the percentage of each cycle that the pin is on

At 50% duty the pin is on half the time, so an LED looks half as bright and a motor gets about half the power. The switching is so fast that you don't see the flicker.

---

## Configuring a pin for PWM

```python
from machine import Pin, PWM

pwm = PWM(Pin(15))
pwm.freq(1000)        # 1000 Hz
```

---

## Setting the duty cycle

On the Pico you set the duty cycle with `duty_u16()`, which takes a number from `0` (always off) to `65535` (always on):

```python
pwm.duty_u16(32768)   # about 50%
pwm.duty_u16(16384)   # about 25%
pwm.duty_u16(0)       # off
```

To turn a percentage into a duty value, multiply by 65535 and divide by 100:

```python
def set_percent(pwm, percent):
    pwm.duty_u16(int(percent * 65535 / 100))
```

When you're finished with a PWM pin, call `pwm.deinit()` to turn the signal off.

> ## Why not `duty()`?
>
> You may see `pwm.duty(512)` in ESP32 or older examples, with a range of 0 to 1023. The Pico's MicroPython uses `duty_u16()` with a range of 0 to 65535 instead.

---

{% quiz %}
? What does a 25% duty cycle mean?
- [ ] The pin outputs 0.825V all the time
- [x] The pin is on for a quarter of each cycle
- [ ] The signal repeats 25 times per second
> Duty cycle is the share of each cycle the pin is on. The voltage is still either 0V or 3.3V.

? Which call sets roughly 50% duty on a Pico?
- [ ] `pwm.duty(512)`
- [x] `pwm.duty_u16(32768)`
- [ ] `pwm.freq(50)`
> The Pico uses `duty_u16()` with values from 0 to 65535, so half is about 32768.
{% endquiz %}

---
