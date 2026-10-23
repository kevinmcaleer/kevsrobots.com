---
layout: lesson
title: GPIO Pin Types
author: Kevin McAleer
type: page
cover: /learn/micropython_gpio/assets/raspberry_pi_pico_gpio.jpg
date: 2023-04-14
previous: 00_intro.html
next: 02_pin_numbering.html
description: Learn about the different types of GPIO pins on the Raspberry Pi Pico
  and their basic functions.
percent: 10
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

In this lesson we'll look at the different jobs a Pico GPIO pin can do. The clever part is that most pins can do *several* jobs: you choose which one in your code.

---

## Different types of GPIO pins

The Raspberry Pi Pico has 26 usable GPIO pins (`GP0` to `GP22` and `GP26` to `GP28`). Each one can be set up as:

- `Digital I/O` - send or receive simple on/off (1 or 0) signals: LEDs, buttons, switches
- `PWM` (Pulse Width Modulation) - switch on and off very fast to dim LEDs, set motor speed or position servos. **Every** GPIO pin can do PWM
- `UART` - simple two-wire serial, used by GPS modules and other microcontrollers
- `I2C` (Inter-Integrated Circuit) - a two-wire bus that can talk to many sensors and displays at once
- `SPI` (Serial Peripheral Interface) - a faster four-wire bus, used by displays and SD cards

Three pins have an extra trick:

- `ADC` (Analog-to-Digital Converter) - `GP26`, `GP27` and `GP28` can measure a voltage between 0V and 3.3V, so you can read potentiometers and analog sensors

The rest of the 40 pins around the edge are power (`3V3`, `VBUS`, `VSYS`) and ground (`GND`) pins, plus a couple of control pins (`RUN`, `3V3_EN`).

---

## Basic functions of GPIO pins

`Digital I/O pins` send or receive digital signals, which can control LEDs, read button presses, or signal other digital devices.

`PWM` lets you control the brightness of LEDs, the speed of motors, or the position of servos by adjusting the duty cycle of the PWM signal.

`ADC pins` read analog signals from sensors, such as light sensors or potentiometers, and convert them into numbers your code can use.

`UART`, `I2C` and `SPI` let the Pico communicate with other devices, such as sensors, displays, or other microcontrollers. See the glossary for [I2C](/resources/glossary#i2c) and [SPI](/resources/glossary#spi).

---

## MicroPython code example

The code below lights the Pico's on-board LED:

```python
import machine

led = machine.Pin("LED", machine.Pin.OUT)
led.value(1)
```

- `import machine` gives us access to the Pico's hardware
- `machine.Pin("LED", machine.Pin.OUT)` sets up the on-board LED pin as an output
- `led.value(1)` sets the pin high (3.3V), so the LED turns on

> ## Pico or Pico W?
>
> On the original Pico the on-board LED is wired to `GP25`, so you'll often see `Pin(25, Pin.OUT)` in older tutorials. On the Pico W the LED is connected through the wireless chip, so `Pin(25)` won't work. Using the name `"LED"` works on both boards with current MicroPython firmware.

---

## Try it yourself

Change `led.value(1)` to `led.value(0)` and run the code again. Then try `led.toggle()` a few times from the Thonny shell.

{% quiz %}
? Which Pico pins can be used for PWM?
- [ ] Only GP18, GP19 and GP20
- [ ] Only the pins marked ADC
- [x] Every GPIO pin
> The Pico has 8 PWM slices with 2 channels each, and every GPIO pin is connected to one of them.

? Which pins can read an analog voltage with the ADC?
- [ ] GP0 to GP3
- [x] GP26, GP27 and GP28
- [ ] Any GPIO pin
> Only GP26, GP27 and GP28 (ADC0 to ADC2) are wired to the analog-to-digital converter.
{% endquiz %}

---
