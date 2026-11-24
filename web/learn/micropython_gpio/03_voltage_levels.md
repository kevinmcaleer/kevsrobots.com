---
layout: lesson
title: Voltage Levels
author: Kevin McAleer
type: page
cover: /learn/micropython_gpio/assets/banners/03_voltage_levels.jpg
date: 2023-04-14
previous: 02_pin_numbering.html
next: 04_digital_io.html
description: Learn about the voltage levels of Raspberry Pi Pico pins and how to work
  with them safely.
percent: 20
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

By the end of this lesson you will be able to say what voltages a Pico pin accepts and how to bring a 5V signal down to 3.3V.

## Overview

The Pico is a 3.3V device. Getting this wrong is the easiest way to damage one, so this short lesson is worth reading before you wire anything up.

---

## Voltage levels

- A GPIO output is either `0V` (low, `0`) or `3.3V` (high, `1`)
- A GPIO input reads `1` for anything above about 2V and `0` for anything below about 0.8V
- The GPIO pins are **not 5V tolerant**. Putting 5V on a GPIO pin can permanently damage the RP2040
- Each pin can only supply a few milliamps (aim for no more than about 12mA). That's enough for an LED with a resistor, but not for a motor, a relay or a servo

---

## Working with voltage levels safely

* Check the voltage of every component before connecting it. Many modules are 5V
* Use a [level shifter](/resources/glossary#level-shifter) or a simple voltage divider when a 5V device sends signals *into* the Pico
* Never drive motors, servos or relays directly from a GPIO pin: use a driver board or transistor, and give them their own power
* If you're not sure, measure the signal with a multimeter first

---

## Converting voltages

A voltage divider is the simplest fix for a 5V output going into a Pico input. Two resistors, for example 1kΩ and 2kΩ, turn 5V into about 3.3V:

```
5V signal ---[ 1kΩ ]---+---[ 2kΩ ]--- GND
                       |
                     Pico GPIO
```

For signals going both ways (for example I2C), use a proper bi-directional level shifter module.

Many 5V modules will happily read a 3.3V signal from the Pico as "high", so signals going *out* of the Pico often work without conversion. Check the datasheet to be sure.

---

---

## Worked example: the 1kΩ and 2kΩ divider

The two resistors share the 5V in proportion to their values. The Pico pin sits between them, across the 2kΩ:

```
V(pin) = 5V x 2kΩ / (1kΩ + 2kΩ) = 5 x 0.667 = 3.33V
```

The current through the pair is 5V / 3kΩ, about 1.7mA, which is tiny. A higher-value pair, such as 10kΩ and 20kΩ, gives the same voltage but responds more slowly to fast signals.

---

## Try it Yourself

1. Work out the voltage at the middle of a divider made from 2kΩ on top and 1kΩ on the bottom. Is it safe for a Pico pin?
2. Using the formula above, what voltage do you get from a 4.5V battery pack with 1kΩ and 2kΩ?
3. Challenge: list three parts in your kit and write down the voltage of each signal wire. Which would need protecting?

## Common Issues

- **Problem**: A 5V sensor works for a while, then the Pico stops responding.
- **Solution**: Add a divider or level shifter on the sensor's output, and replace the Pico if it no longer works.
- **Why**: GPIO pins are not 5V tolerant, so 5V can slowly or suddenly damage the RP2040 input circuits.

> **Teachers**: Never connect 5V signals to a Pico in front of a class without checking first. Have a spare Pico in the box, and pre-build the 1kΩ and 2kΩ divider on a breadboard for groups that use an HC-SR04. A cheap multimeter lets students measure the voltage themselves before connecting anything.

## Check your understanding

{% quiz %}
? A sensor module runs on 5V and its output pin goes to 5V. What should you do before connecting it to a Pico GPIO pin?
- [ ] Nothing, the Pico pins are 5V tolerant
- [x] Use a voltage divider or level shifter to bring it down to 3.3V
- [ ] Connect it to VBUS instead
> The Pico's GPIO pins are 3.3V only. A divider or level shifter keeps the input within a safe range.

? Why shouldn't you power a motor straight from a GPIO pin?
- [ ] Motors need an analog signal
- [x] A GPIO pin can only supply a few milliamps
- [ ] GPIO pins only output 1.8V
> Motors need far more current than a pin can safely give. Use a motor driver and a separate supply.

? What voltage does a 1kΩ and 2kΩ divider give from a 5V input?
- [ ] 1.65V
- [x] About 3.3V
- [ ] 5V
> 5V x 2 / 3 is about 3.3V, which is safe for a Pico input.
{% endquiz %}

---
