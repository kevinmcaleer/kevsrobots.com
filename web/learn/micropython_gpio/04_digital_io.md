---
layout: lesson
title: Digital I/O
author: Kevin McAleer
type: page
cover: /learn/micropython_gpio/assets/raspberry_pi_pico_gpio.jpg
date: 2023-04-14
previous: 03_voltage_levels.html
next: 05_pwm.html
description: Learn about digital input and output with Raspberry Pi Pico using MicroPython.
percent: 25
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

Digital I/O is the simplest thing a GPIO pin can do: a pin is either on (`1`) or off (`0`). In this lesson you'll make outputs, read inputs and wire up a button the right way.

---

## Digital input and output

**Digital output** means the Pico sets a pin high (3.3V) or low (0V), for example to turn an LED on or off.

**Digital input** means the Pico reads whether a pin is high or low, for example to tell whether a button is pressed.

---

## Configuring pins for digital I/O

You choose what a pin does with the `Pin` class from the `machine` module.

```python
from machine import Pin

led = Pin(15, Pin.OUT)                   # GP15 as an output
button = Pin(14, Pin.IN, Pin.PULL_UP)    # GP14 as an input with the pull-up resistor on
```

---

## Writing digital values

```python
led.value(1)   # on
led.value(0)   # off
led.toggle()   # flip to the other state
```

## Reading digital values

```python
print(button.value())
```

`value()` with no argument returns `0` or `1`.

---

## Wiring a button with a pull-up

An input pin that isn't connected to anything "floats" and reads random values. A pull-up resistor holds it at `1` until something pulls it to ground. The Pico has these resistors built in, so you only need `Pin.PULL_UP`.

Wire one leg of the button to `GP14` and the other leg to `GND`. Now the pin reads `1` when the button is up and `0` when it's pressed.

```python
from machine import Pin
from time import sleep

led = Pin(15, Pin.OUT)
button = Pin(14, Pin.IN, Pin.PULL_UP)

while True:
    if button.value() == 0:   # pressed
        led.value(1)
    else:
        led.value(0)
    sleep(0.01)
```

---

## Try it yourself

Change the program so each press *toggles* the LED instead of only lighting it while you hold the button. Hint: remember the last button value and only act when it changes from `1` to `0`.

{% quiz %}
? A button is wired between GP14 and GND, with `Pin.PULL_UP`. What does `button.value()` return while it's pressed?
- [x] 0
- [ ] 1
- [ ] It changes randomly
> Pressing the button connects the pin to ground, so it reads 0. The pull-up makes it read 1 when released.

? What happens to an input pin with no pull-up or pull-down and nothing connected?
- [ ] It always reads 0
- [x] It floats and can read random values
- [ ] It reads 3.3V as an analog value
> A floating input picks up electrical noise. Pull resistors give it a known default.
{% endquiz %}

---
