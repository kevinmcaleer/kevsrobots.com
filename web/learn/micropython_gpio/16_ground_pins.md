---
layout: lesson
title: Ground Pins
author: Kevin McAleer
type: page
cover: /learn/micropython_gpio/assets/raspberry_pi_pico_gpio.jpg
previous: 15_ultrasonic_range_finders.html
next: 17_summary.html
description: Learn about ground pins on the Raspberry Pi Pico and how to use them
  in your projects.
percent: 85
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


## Overview

Welcome to Lesson 16 of the `Raspberry Pi Pico with MicroPython - GPIO Mastery` course. In this lesson, we will learn about ground pins on the Raspberry Pi Pico and how to use them in your projects.

---

## Course Content

In this lesson, you will learn:

* What ground pins are and how they work
* How to identify ground pins on the Raspberry Pi Pico
* How to use ground pins in your circuits

---

## Key Results

After you have completed this lesson, you will understand the basics of ground pins and how to use them in your circuits. You will be able to identify ground pins on the Raspberry Pi Pico and use them to connect components to your board.

---

## What you'll need

To follow this lesson, you will need:

* A computer, tablet, or phone to read the lesson material from
* A Raspberry Pi Pico board
* Jumper wires and a breadboard for hands-on practice

---

## What are ground pins?

Ground pins are a type of GPIO pin that is used to complete an electrical circuit. They provide a low-resistance path for electrical current to flow back to the power source, such as a battery or power supply. Without ground pins, circuits would not work properly or at all.

---

## Identifying ground pins on the Raspberry Pi Pico

The Raspberry Pi Pico has several ground pins that are labeled `GND`. These pins are usually black or brown and are often grouped together with other ground pins. It is important to use the correct ground pins when connecting components to the Pico to ensure proper circuit operation.

---

## Using ground pins in your circuits

To use ground pins in your circuits, simply connect one end of your component (such as an LED or a resistor) to a GPIO pin and the other end to a ground pin. This will complete the circuit and allow electrical current to flow through the component.

---

## Summary

In this lesson, you will learn about ground pins on the Raspberry Pi Pico and how to use them in your projects. You will be able to identify ground pins on the board and use them to connect components to your circuits.

---
