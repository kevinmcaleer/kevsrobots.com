---
layout: lesson
title: Installing MicroPython on the Pico
author: Kevin McAleer
type: page
cover: assets/micropython_install_cover.png
date: 2024-11-22
previous: 02_setting_up_thonny.html
next: 04_overview_of_pico_temperature_sensor.html
description: Learn how to install MicroPython on your Raspberry Pi Pico to enable
  programming and communication through Thonny.
percent: 24
duration: 2
navigation:
- name: Using the Raspberry Pi Pico's Built-in Temperature Sensor
- content:
  - section: Introduction to the Raspberry Pi Pico and MicroPython
    content:
    - name: Course Introduction
      link: 01_course_introduction.html
    - name: Setting Up Thonny
      link: 02_setting_up_thonny.html
    - name: Installing MicroPython on the Pico
      link: 03_installing_micropython_on_pico.html
  - section: Understanding the Built-in Temperature Sensor
    content:
    - name: "Understanding the Pico\u2019s Temperature Sensor"
      link: 04_overview_of_pico_temperature_sensor.html
    - name: Reading Temperature with the ADC
      link: 05_how_the_temperature_sensor_works.html
  - section: Writing Your First Temperature Sensor Script
    content:
    - name: Converting ADC Values to Temperature
      link: 06_reading_temperature_with_adc.html
    - name: Formatting Temperature Readings
      link: 07_displaying_temperature_in_thonny_console.html
    - name: Enhancing Temperature Readings
      link: 08_formatting_temperature_readings.html
  - section: Enhancing Your Temperature Monitoring Project
    content:
    - name: Converting ADC Values to Temperature
      link: 09_converting_adc_to_temperature.html
    - name: Creating a Temperature Alert System
      link: 10_creating_a_temperature_alert_system.html
  - section: Project Challenges and Next Steps
    content:
    - name: Project Ideas and Extensions
      link: 11_project_ideas_and_extensions.html
    - name: Troubleshooting Temperature Readings
      link: 12_troubleshooting_temperature_readings.html
---


## Introduction

MicroPython is a lightweight version of Python specifically designed for microcontrollers. By installing MicroPython on your Raspberry Pi Pico, you can use Python to control hardware and create exciting projects.

---

## Step 1: Download MicroPython Firmware

1. Visit the [official MicroPython download page](https://micropython.org/download/rp2-pico/).
2. Download the latest `.uf2` firmware file for the Raspberry Pi Pico.

---

## Step 2: Enter Bootloader Mode

1. Hold down the **BOOTSEL** button on your Pico.
2. While holding the button, connect the Pico to your computer using a USB cable.
3. The Pico will appear as a removable storage device.

---

## Step 3: Flash the Firmware

1. Drag and drop the downloaded `.uf2` file onto the Pico’s drive.
2. The Pico will reboot and appear as a MicroPython device.

---

## Step 4: Verify the Installation

1. Open Thonny and select **MicroPython (Raspberry Pi Pico)** in the interpreter settings.
2. In the console, type:

   ```python
   print("MicroPython is ready!")
   ```

3. You should see the message displayed in the console.

Congratulations, MicroPython is now installed on your Raspberry Pi Pico!

---
