---
layout: lesson
title: Setting Up Thonny
author: Kevin McAleer
type: page
cover: /learn/pico_temp_sensor/assets/cover.jpg
date: 2024-11-22
previous: 01_course_introduction.html
next: 03_installing_micropython_on_pico.html
description: Get your environment ready by installing and configuring the Thonny IDE
  to program your Raspberry Pi Pico with MicroPython.
percent: 16
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

Before you can start programming your Raspberry Pi Pico, you need an Integrated Development Environment (IDE) that supports MicroPython. Thonny is a beginner-friendly IDE that makes it easy to write and execute Python scripts on your Pico.

---

## What You Will Do in This Lesson

- Install Thonny IDE on your computer.
- Connect your Raspberry Pi Pico.
- Configure Thonny to program with MicroPython.

---

## Step 1: Install Thonny

1. Visit [thonny.org](https://thonny.org) to download the latest version of Thonny for your operating system.
2. Follow the installation instructions for your platform:
   - **Windows:** Run the installer and follow the prompts.
   - **MacOS:** Drag the Thonny app to your Applications folder.
   - **Linux:** Use your package manager (e.g., `sudo apt install thonny` on Debian-based systems).

---

## Step 2: Connect the Raspberry Pi Pico

1. Plug your Raspberry Pi Pico into your computer using a USB cable.
2. Hold down the **BOOTSEL** button while connecting the USB cable to enter the Pico’s bootloader mode.
3. The Pico should appear as a removable drive on your computer.

---

## Step 3: Install MicroPython on the Pico

1. Open Thonny.
2. From the menu, go to **Tools > Options** and select the **Interpreter** tab.
3. Choose **MicroPython (Raspberry Pi Pico)** as the interpreter.
4. If prompted, install the MicroPython firmware on your Pico.

---

## Step 4: Verify the Setup

1. In Thonny’s editor, type:

   ```python
   print("Hello, Pico!")
   ```

2. Save the script to your Pico and run it. You should see the output in Thonny’s console.

You're now ready to start programming your Pico!

---
