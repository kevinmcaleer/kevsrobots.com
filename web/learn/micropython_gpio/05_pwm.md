---
layout: lesson
title: Pulse Width Modulation (PWM)
author: Kevin McAleer
type: page
cover: /learn/micropython_gpio/assets/banners/05_pwm.jpg
date: 2023-04-14
previous: 04_digital_io.html
next: 06_leds.html
description: Learn about Pulse Width Modulation (PWM) and controlling devices like
  LEDs and motors using the Raspberry Pi Pico with MicroPython.
percent: 30
duration: 4
quiz_total: 54
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


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to set up a PWM pin and turn a percentage into a `duty_u16()` value.

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

---

## A complete example

Save this and run it with an LED and 220Ω resistor on GP15 (the wiring is in the next lesson). It steps through four brightness levels:

```python
from machine import Pin, PWM
from time import sleep

pwm = PWM(Pin(15))
pwm.freq(1000)

def set_percent(pwm, percent):
    pwm.duty_u16(int(percent * 65535 / 100))

for percent in (0, 25, 50, 100):
    set_percent(pwm, percent)
    print(percent, "%")
    sleep(1)

pwm.deinit()
```

The Pico has 8 PWM slices, each with 2 channels, and every GPIO pin connects to one. The two pins on a slice share a frequency, so setting `freq()` on one changes its neighbour.

---

## Try it Yourself

1. Run the example and watch the LED step through four brightness levels.
2. Change the frequency to 5 Hz. What do you see, and why?
3. Challenge: use `set_percent()` in a loop to make the LED breathe from 0% to 100% and back.

## Common Issues

- **Problem**: `AttributeError: 'PWM' object has no attribute 'duty'`.
- **Solution**: Use `duty_u16()` with a value from 0 to 65535.
- **Why**: The Pico's MicroPython port does not have the older `duty()` method used on the ESP32.

## Check your understanding

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

? What is the largest value `duty_u16()` accepts?
- [ ] 100
- [ ] 1023
- [x] 65535
> `duty_u16()` runs from 0 (always off) to 65535 (always on).
{% endquiz %}

---
