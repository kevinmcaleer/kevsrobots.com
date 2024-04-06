---
layout: lesson
title: Setting Up PCA9685
author: Kevin McAleer
type: page
cover: assets/5.png
date: 2024-04-06
previous: 04_introduction_to_pca9685.html
next: 06_designing_your_robot_arm.html
description: Step-by-step guide on connecting the PCA9685 servo driver to the Raspberry
  Pi and testing it with Python to control servos.
percent: 40
duration: 3
navigation:
- name: Building a Robot Arm with Raspberry Pi and PCA9685
- content:
  - section: Getting Started with Raspberry Pi and Python
    content:
    - name: Introduction
      link: 01_course_introduction.html
    - name: Setting Up Your Raspberry Pi
      link: 02_raspberry_pi_setup.html
    - name: Python Basics for Robotics
      link: 03_python_basics_for_robotics.html
  - section: Understanding the PCA9685 Servo Driver
    content:
    - name: PCA9685 Servo Driver
      link: 04_introduction_to_pca9685.html
    - name: Setting Up PCA9685
      link: 05_setting_up_the_pca9685_with_raspberry_pi.html
  - section: Building the Robot Arm
    content:
    - name: Design
      link: 06_designing_your_robot_arm.html
    - name: Assembly
      link: 07_assembling_the_robot_arm.html
  - section: Programming the Robot Arm
    content:
    - name: Controlling Servos with Python
      link: 08_controlling_servos_with_python.html
    - name: Basic Arm Movements
      link: 09_creating_basic_arm_movements.html
    - name: Advanced Control and Automation
      link: 10_advanced_arm_control_and_automation.html
  - section: Project Challenges and Ideas
    content:
    - name: Project Ideas
      link: 11_project_ideas.html
    - name: Troubleshooting
      link: 12_troubleshooting_common_issues.html
---


![Setting Up PCA9685 with Raspberry Pi]({{ page.cover }}){:class="cover"}

## Preparing for Servo Control

In this lesson, we'll take a closer look at how to properly set up the PCA9685 servo driver with your Raspberry Pi and write a simple Python script to test servo control. This is a crucial step in bringing our robot arm to life.

---

## Equipment Needed

- Raspberry Pi (with Python and necessary libraries installed)
- PCA9685 servo driver board
- Servos (at least one for testing)
- External power supply for servos (5-6V)
- Jumper wires

---

## Connection Overview

Ensure your Raspberry Pi is turned off before making any connections to prevent damage. Connect the PCA9685 to the Raspberry Pi as follows:

1. **VCC** to Raspberry Pi 5V
2. **GND** to Raspberry Pi GND
3. **SCL** to Raspberry Pi SCL (GPIO 3)
4. **SDA** to Raspberry Pi SDA (GPIO 2)

Connect your external power supply to the PCA9685, and then connect a servo to one of the PCA9685's channels, ensuring the orientation of the wires is correct.

---

## Testing the Connection

With the hardware set up, let's write a simple Python script to test controlling a servo.

### Step 1: Import Libraries

```python
from Adafruit_PCA9685 import PCA9685
from time import sleep
```

---

### Step 2: Initialize PCA9685

```python
pwm = PCA9685()
pwm.set_pwm_freq(60)  # Set frequency to 60Hz
```

---

### Step 3: Control a Servo

We'll make the servo connected to channel 0 move back and forth.

```python
channel = 0
min_pulse = 150  # Min pulse length out of 4096
max_pulse = 600  # Max pulse length out of 4096

while True:
    pwm.set_pwm(channel, 0, min_pulse)
    sleep(1)
    pwm.set_pwm(channel, 0, max_pulse)
    sleep(1)
```

This script sets the servo to its minimum position, waits a second, moves it to its maximum position, and waits again in a continuous loop.

---

## Conclusion

You have now successfully set up the PCA9685 servo driver with your Raspberry Pi and tested it by controlling a servo. This foundational knowledge will be invaluable as we proceed to design and program the movements of our robot arm.

---

## Lesson Assignment

Experiment with different values for `min_pulse` and `max_pulse` to see how they affect the servo's movement. Try connecting and controlling multiple servos on different channels of the PCA9685.

---
