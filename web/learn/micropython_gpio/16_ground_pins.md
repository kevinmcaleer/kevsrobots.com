---
layout: lesson
title: Ground Pins
author: Kevin McAleer
type: page
cover: /learn/micropython_gpio/assets/raspberry_pi_pico_gpio.jpg
date: 2023-04-14
previous: 15_ultrasonic_range_finders.html
next: 17_summary.html
description: Learn about ground pins on the Raspberry Pi Pico and how to use them
  in your projects.
percent: 85
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

Ground pins don't do anything clever, but every circuit needs them and a missing ground is behind a lot of "it doesn't work" moments. This short lesson explains why.

---

## What are ground pins?

Ground (`GND`) is the 0V reference that every other voltage in your circuit is measured against. It's also the return path that lets current flow back to the power supply and complete the circuit.

Ground pins aren't GPIO pins: you can't switch them on or off from code. They're always 0V.

---

## Identifying ground pins on the Pico

The Pico has eight ground pins, at physical pins **3, 8, 13, 18, 23, 28, 33 and 38**. They're spread out so there's always one near the pins you're using. On the board they have square pads rather than round ones.

Pin 33 is labelled `AGND`. It's the ground for the analog circuitry, so use it with potentiometers and analog sensors for the cleanest ADC readings.

All the ground pins are connected together, so you can use whichever is closest.

---

## Using ground pins in your circuits

* **Every circuit needs a ground.** An LED goes from a GPIO pin, through a resistor, to GND
* **Share grounds between power supplies.** When a motor or servo has its own battery, connect the battery's negative to the Pico's GND. Without a shared ground the Pico's signals have nothing to be measured against, and things behave strangely or not at all
* **Use the breadboard rail.** Run one wire from a GND pin to the breadboard's blue (`-`) rail, then connect everything else to the rail

---

{% quiz %}
? A servo has its own battery pack and twitches randomly. What's the most likely fix?
- [ ] Use a faster PWM frequency
- [x] Connect the battery's negative to the Pico's GND
- [ ] Move the signal wire to an ADC pin
> Without a shared ground, the servo can't read the Pico's signal reliably.

? Which ground pin is best for analog sensors?
- [ ] Pin 3
- [x] Pin 33, AGND
- [ ] It must be a GPIO pin set to 0
> AGND is the reference for the ADC, so it gives the cleanest readings.
{% endquiz %}

---
