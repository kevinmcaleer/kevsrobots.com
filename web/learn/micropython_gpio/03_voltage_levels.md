---
layout: lesson
title: Voltage Levels
author: Kevin McAleer
type: page
cover: /learn/micropython_gpio/assets/raspberry_pi_pico_gpio.jpg
date: 2023-04-14
previous: 02_pin_numbering.html
next: 04_digital_io.html
description: Learn about the voltage levels of Raspberry Pi Pico pins and how to work
  with them safely.
percent: 20
duration: 3
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
{% endquiz %}

---
