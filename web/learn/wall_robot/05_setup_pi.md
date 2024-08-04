---
layout: lesson
title: Setting Up the Raspberry Pi
author: Kevin McAleer
type: page
cover: assets/setup_pi.png
date: 2024-08-02
previous: 04_wiring.html
next: 06_software.html
description: Instructions to set up and configure the Raspberry Pi.
percent: 50
duration: 1
date_updated: 2024-08-02
navigation:
- name: Wall Drawing Robot Tutorial
- content:
  - section: Introduction
    content:
    - name: Introduction to the Wall Drawing Robot
      link: 01_intro.html
  - section: Build the Robot
    content:
    - name: Components Needed
      link: 02_components.html
    - name: Assembling the Robot
      link: 03_assembly.html
    - name: Wiring the Components
      link: 04_wiring.html
  - section: Programming
    content:
    - name: Setting Up the Raspberry Pi
      link: 05_setup_pi.html
    - name: Installing the Software
      link: 06_software.html
    - name: Writing Control Code
      link: 07_control.html
    - name: Creating Your First Drawing
      link: 08_draw.html
    - name: Vectorizing Images for Drawing
      link: 09_vectorize.html
  - section: Summary
    content:
    - name: Summary and Next Steps
      link: 10_summary.html
---


![Setup Pi](assets/setup_pi.png){:class="cover"}

---

## Setting Up the Raspberry Pi

1. **Install Raspberry Pi OS**:
   - Download and install the latest version of Raspberry Pi OS on a microSD card.
   - Insert the microSD card into the Raspberry Pi and boot it up.

2. **Connect to WiFi**:
   - Follow the on-screen instructions to connect your Raspberry Pi to your WiFi network.

3. **Update the System**:
   - Open a terminal and run:
     ```sh
     sudo apt-get update
     sudo apt-get upgrade
     ```

4. **Enable I2C, SPI, and Serial Interfaces**:
   - Run `sudo raspi-config` and enable the necessary interfaces.

---

Your Raspberry Pi is now set up. Next, we'll install the software needed for the robot.

---
