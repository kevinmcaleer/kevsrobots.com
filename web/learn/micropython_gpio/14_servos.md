---
layout: lesson
title: Introduction to Servos
author: Kevin McAleer
type: page
cover: /learn/micropython_gpio/assets/raspberry_pi_pico_gpio.jpg
previous: 13_spi.html
next: 15_ultrasonic_range_finders.html
description: Learn about servos and how to control them with the Raspberry Pi Pico
  and MicroPython.
percent: 75
duration: 3
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


## Introduction

Welcome to Lesson 14 of the `Raspberry Pi Pico with MicroPython - GPIO Mastery` course. In this lesson, you will learn about servos and how to control them with the Raspberry Pi Pico and MicroPython. A servo is a small device that rotates to a specific position, making it ideal for controlling the movement of robots, drones, and other projects.

---

## What is a Servo?

A [servo](/resources/how_it_works/servos) is a small motor that can be controlled to rotate to a specific position. It has three wires: power, ground, and signal. The power wire provides power to the servo, while the ground wire provides a common ground for the servo and the Raspberry Pi Pico. The signal wire is used to send a pulse-width modulation (PWM) signal to the servo to control its position.

---

## Connecting a Servo to Raspberry Pi Pico

To connect a servo to your Raspberry Pi Pico, you will need to connect its power wire to a 5V pin on the board, its ground wire to a GND pin on the board, and its signal wire to a PWM-enabled pin on the board. The signal wire is typically connected to pin GP18 on the Raspberry Pi Pico board.

---

## Controlling a Servo with MicroPython

To control a servo with MicroPython, you will need to use the `machine.PWM` module to generate the PWM signal that controls the servo's position. Here's an example of how to control a servo with MicroPython:

```python
from machine import Pin, PWM
import utime

# Initialize PWM on pin GP18 with a frequency of 50Hz
servo = PWM(Pin(18))
servo.freq(50)

# Set the servo to its minimum position
servo.duty_u16(2500)
utime.sleep(1)

# Set the servo to its maximum position
servo.duty_u16(12500)
utime.sleep(1)

# Set the servo to its middle position
servo.duty_u16(7500)
utime.sleep(1)

# Stop the PWM signal
servo.deinit()
```

In this example, we first initialize the PWM module on pin GP18 with a frequency of 50Hz. We then use the `duty_u16` method to set the servo to its minimum, maximum, and middle positions. Finally, we stop the PWM signal using the `deinit` method.

---

## Conclusion

In this lesson, you learned about servos and how to control them with the Raspberry Pi Pico and MicroPython. You also learned how to connect a servo to your board and how to write MicroPython code to control the servo's position. You can use this knowledge to create a variety of projects that involve controlling the movement of robots, drones, and other projects using servos.

---
