---
layout: lesson
title: Introduction to UART
author: Kevin McAleer
type: page
cover: /learn/micropython_gpio/assets/raspberry_pi_pico_gpio.jpg
date: 2023-04-14
previous: 10_temperature_sensors.html
next: 12_i2c.html
description: Learn about UART communication protocol and how to use it with the Raspberry
  Pi Pico and MicroPython.
percent: 60
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

In this lesson, you'll learn how to use UART (Universal Asynchronous Receiver/Transmitter) to send data between your Pico and other devices. UART is used by GPS modules, Bluetooth modules like the HC-05, and to link two microcontrollers together.

---

## What is UART?

UART sends data over two wires: `TX` (transmit) and `RX` (receive). It is *asynchronous*, which means there's no shared clock wire. Instead, both devices agree on a speed, called the **baud rate** (for example 9600 or 115200 bits per second).

Wire them crossed over:

* Pico `TX` goes to the other device's `RX`
* Pico `RX` goes to the other device's `TX`
* `GND` goes to `GND`

---

## Configuring UART on the Pico

The Pico has two UARTs, `UART0` and `UART1`, and each can use several pairs of pins (see the pinout). `UART0` on `GP0` (TX) and `GP1` (RX) is the usual choice:

```python
from machine import UART, Pin

uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))
```

---

## Sending and receiving data

```python
uart.write("Hello, world!\n")
```

```python
if uart.any():               # is there anything waiting?
    data = uart.read()       # read all of it, as bytes
    print(data)
```

`uart.any()` returns how many bytes are waiting, so you only read when there's something there.

---

## Try it yourself

Connect `GP0` to `GP1` with a single jumper wire (a "loopback"). Write a message and read it straight back.

{% quiz %}
? How should TX and RX be wired between two UART devices?
- [ ] TX to TX and RX to RX
- [x] TX to RX and RX to TX
- [ ] Both to GND
> Each device's transmitter needs to talk to the other device's receiver.

? What must both devices agree on for UART to work?
- [x] The baud rate
- [ ] The I2C address
- [ ] The clock pin
> UART has no clock wire, so both sides must use the same baud rate.
{% endquiz %}

---
