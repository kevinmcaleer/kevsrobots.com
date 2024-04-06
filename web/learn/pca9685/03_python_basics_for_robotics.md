---
layout: lesson
title: Python Basics for Robotics
author: Kevin McAleer
type: page
cover: assets/3.png
date: 2024-04-06
previous: 02_raspberry_pi_setup.html
next: 04_introduction_to_pca9685.html
description: Learn the fundamentals of Python programming necessary for controlling
  hardware like servos on a robot arm.
percent: 24
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


![Python Basics for Robotics]({{ page.cover }}){:class="cover"}

## Introduction to Python Programming

Welcome to Lesson 3, where we dive into Python programming for robotics. Python is a versatile and powerful programming language that's widely used in robotics for its simplicity and readability, making it ideal for beginners and professionals alike.

---

## Why Python for Robotics?

- **Ease of Learning:** Python's syntax is clear and intuitive, making it an excellent choice for those new to programming.
- **Strong Community:** A vast community of developers means abundant resources, libraries, and frameworks are available.
- **Versatility:** Python can be used for a wide range of applications, from web development to data analysis and, of course, robotics.

---

## Setting Up Your Python Environment

Assuming you've followed the Raspberry Pi setup from Lesson 2, Python should already be installed on your Raspberry Pi. Here, we'll ensure your Python environment is ready for robotics programming.

---

### Python Libraries for Robotics

Install the necessary Python libraries for interfacing with hardware:

```shell
pip3 install RPi.GPIO Adafruit-PCA9685
```

- `RPi.GPIO` allows you to control the Raspberry Pi GPIO pins—a fundamental requirement for many robotics projects.
- `Adafruit-PCA9685` is the library we'll use to communicate with the PCA9685 servo controller board.

---

## Python Basics

Let's cover some Python basics that you'll frequently use in robotics projects.

---

### Variables and Data Types

In Python, variables don't require explicit declaration to reserve memory space. The declaration happens automatically when you assign a value to a variable.

```python
x = 10          # Integer
y = 3.14        # Float
name = "Robot"  # String
```

---

### Control Structures

**If Statement:**

```python
if x > 5:
    print("x is greater than 5")
```

**For Loop:**

```python
for i in range(5):
    print(i)
```

**While Loop:**

```python
while x > 0:
    print(x)
    x -= 1
```

---

### Functions

Functions are blocks of code that only run when called. They can receive data, operate on it, and return the result.

```python
def greet(name):
    return "Hello, " + name + "!"

print(greet("Robot"))
```

---

### Working with Libraries

Import a library and use its functions:

```python
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
```

---

## Conclusion

You've now covered the basics of Python programming! With these fundamentals, you're well-equipped to start exploring more complex robotics projects. Practice writing some basic Python scripts to familiarize yourself with these concepts.

---

## Lesson Assignment

Create a simple Python script that turns an LED on and off using the `RPi.GPIO` library. This exercise will prepare you for controlling servos in the upcoming lessons.

---
