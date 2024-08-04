---
layout: lesson
title: Installing the Software
author: Kevin McAleer
type: page
cover: assets/software.png
date: 2024-08-02
previous: 05_setup_pi.html
next: 07_control.html
description: Guide to installing the necessary software for the wall drawing robot.
percent: 54
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
    - name: Understanding Positioning with Trigonometry
      link: 08_positioning.html
    - name: Creating Your First Drawing
      link: 09_draw.html
    - name: Vectorizing Images for Drawing
      link: 10_vectorize.html
  - section: Summary
    content:
    - name: Summary and Next Steps
      link: 11_summary.html
---


## Installing the Software

The software for this project is avialable on GitHub - <https://www.github.com/kevinmcaleer/scrawly-wally.git>.

To install this on your Raspberry Pi, follow these steps:

1. **Install Python and Required Libraries**:
   - Open a terminal and run:

     ```sh
     sudo apt-get install python3 python3-pip
     pip3 install RPi.GPIO pigpio
     ```

     Note - `pigpio` is a library for controlling the GPIO pins on the Raspberry Pi; it needs to be run as a daemon process. You can start it by running:

     ```sh
      sudo pigpiod
      ```

1. **Clone the Software**:
   - To clone the software, Run:

     ```sh
     git clone https://www.github.com/kevinmcaleer/scrawly-wally.git
     ```

---

The software is now installed. Let's move on to writing the control code.

> **Note**: The software for this project is a work in progress and may be updated. Check the GitHub repository for the latest version and updates.
>
> To update the software, navigate to the cloned repository and run `git pull` to fetch the latest changes.

---
