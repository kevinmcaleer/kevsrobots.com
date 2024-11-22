---
title: Understanding the Pico’s Temperature Sensor
description: Dive into how the Raspberry Pi Pico’s built-in temperature sensor works and how to interact with it programmatically.
layout: lesson
type: page
cover: assets/temperature_sensor_intro.png
---

## Introduction

The Raspberry Pi Pico comes with a built-in temperature sensor that you can access using its ADC (Analog-to-Digital Converter). This lesson explains how the sensor works and prepares you to use it in your projects.

---

## How the Temperature Sensor Works

1. The temperature sensor is integrated into the Pico’s RP2040 chip.
2. It measures the temperature of the chip itself and converts the reading into an ADC value.
3. The ADC value can be translated into a temperature in degrees Celsius using a formula.

---

## What You Will Learn

- The basics of the Pico’s ADC.
- How to read raw data from the temperature sensor.
- How to convert ADC values into human-readable temperatures.

---

## Understanding ADC

- The Pico’s ADC converts analog signals (like temperature) into digital values.
- ADC values range from 0 to 65535 on the Pico.
- The temperature sensor’s output can be processed using a specific formula.

---

## Preparing for the Next Lesson

In the next lesson, we’ll write a script to read data from the temperature sensor and display it in the Thonny console. Make sure your Pico is set up with MicroPython and ready to go!

---

