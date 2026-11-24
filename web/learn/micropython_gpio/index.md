---
layout: course_overview
title: Raspberry Pi Pico with MicroPython - GPIO Mastery
author: Kevin McAleer
cover: /learn/micropython_gpio/assets/raspberry_pi_pico_gpio.jpg
date: 2023-04-14
description: Master the use of GPIO pins on the Raspberry Pi Pico with MicroPython.
  Learn about various types of pins, their functions, and how to work with them in
  real-world projects.
level: beginner
course_duration: 1h 6m
lesson_count: 18
first_lesson: 00_intro.html
quiz_total: 54
video: M-0P6ipVwNU
outcomes:
- Find any Pico pin on the pinout and use the right GP number in MicroPython
- Wire LEDs, buttons and potentiometers safely, with resistor values worked out
- Use PWM to fade an LED, set motor speed and position a servo
- Read analog voltages with the ADC and turn them into volts
- Talk to other chips and modules over UART, I2C and SPI
- Protect the Pico from 5V signals with a divider or level shifter
prerequisites:
- Learn MicroPython - The basics (/learn/micropython/)
you_need:
- A Raspberry Pi Pico or Pico W with MicroPython installed
- A USB cable and a computer running Thonny
- A breadboard and jumper wires
- LEDs, 220 ohm resistors and a push button
- A 10k potentiometer
- An L293D motor driver, a small DC motor and a battery pack (lesson 7 only)
- An SG90 servo (lesson 14 only)
- An HC-SR04 range finder with 1k and 2k resistors (lesson 15 only)
- A DS18B20 or DHT11 temperature sensor (lesson 10, optional)
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
