---
layout: lesson
title: Controlling Motors with PWM
author: Kevin McAleer
type: page
cover: /learn/micropython_gpio/assets/raspberry_pi_pico_gpio.jpg
date: 2023-04-14
previous: 06_leds.html
next: 08_adc.html
description: Learn how to control motors using Pulse Width Modulation (PWM) with the
  Raspberry Pi Pico and MicroPython.
percent: 40
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

A motor needs far more current than a GPIO pin can supply, so we use a motor driver chip between the Pico and the motor. In this lesson you'll wire up an L293D driver and control a DC motor's direction and speed.

---

## What you'll need

* A Raspberry Pi Pico
* A small DC motor (3V to 6V)
* An L293D motor driver chip (a DRV8833 board works the same way)
* A battery pack for the motor, such as 4 x AA
* A [breadboard](/resources/how_it_works/breadboards.html) and jumper wires

---

## Wiring the circuit

The L293D has two power pins: `VCC1` (pin 16) powers the logic, and `VCC2` (pin 8) powers the motor.

1. Connect the Pico's `VBUS` (5V from USB) to L293D `VCC1` (pin 16)
2. Connect the battery pack's `+` to L293D `VCC2` (pin 8)
3. Connect the battery `-`, the Pico `GND` and the L293D ground pins (4, 5, 12, 13) together. The grounds **must** be shared
4. Connect Pico `GP0` to L293D `IN1` (pin 2)
5. Connect Pico `GP1` to L293D `IN2` (pin 7)
6. Connect Pico `GP2` to L293D `EN1` (pin 1). This is the speed control
7. Connect the motor to L293D `OUT1` (pin 3) and `OUT2` (pin 6)

---

## Writing the code

```python
from machine import Pin, PWM
from time import sleep

in1 = Pin(0, Pin.OUT)
in2 = Pin(1, Pin.OUT)
speed = PWM(Pin(2))
speed.freq(1000)

def forward(percent):
    in1.value(1)
    in2.value(0)
    speed.duty_u16(int(percent * 65535 / 100))

def backward(percent):
    in1.value(0)
    in2.value(1)
    speed.duty_u16(int(percent * 65535 / 100))

def stop():
    in1.value(0)
    in2.value(0)
    speed.duty_u16(0)

forward(50)
sleep(2)
backward(100)
sleep(2)
stop()
```

- `IN1` and `IN2` set the **direction**: one high and one low makes the motor spin, and swapping them reverses it
- The PWM signal on `EN1` sets the **speed**
- `stop()` sets both inputs low and the speed to zero

> ## Low speeds
>
> Most small motors won't start turning below about 30% duty. That's normal: there isn't enough power to overcome friction.

---

## Try it yourself

Write a `ramp_up()` function that takes the motor from 0% to 100% over two seconds.

{% quiz %}
? Which pins set the motor's direction on the L293D?
- [x] IN1 and IN2
- [ ] EN1
- [ ] VCC1 and VCC2
> IN1 and IN2 choose the direction. The PWM signal on EN1 sets the speed.

? Why must the battery ground be connected to the Pico's GND?
- [ ] It makes the motor spin faster
- [x] The signals from the Pico need a shared reference to be understood
- [ ] It charges the battery
> Voltages are always measured relative to ground. Without a common ground, the driver can't read the Pico's signals reliably.
{% endquiz %}

---
