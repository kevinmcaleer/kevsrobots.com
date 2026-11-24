---
layout: lesson
title: Controlling DC Motors with the Raspberry Pi Pico
author: Kevin McAleer
type: page
cover: /learn/micropython_robotics/assets/banners/02_dc_motors.jpg
date: 2025-05-20
previous: 01_intro.html
next: 03_hbridge_control.html
description: Learn how to connect and control DC motors using MicroPython and the
  L298N motor driver.
percent: 16
duration: 3
date_updated: 2025-05-10
navigation:
- name: MicroPython Robotics Projects with the Raspberry Pi Pico
- content:
  - section: Introduction
    content:
    - name: Introduction to MicroPython Robotics Projects
      link: 01_intro.html
  - section: Motors and Movement
    content:
    - name: Controlling DC Motors with the Raspberry Pi Pico
      link: 02_dc_motors.html
    - name: Using H-Bridge Motor Drivers
      link: 03_hbridge_control.html
  - section: Sensors and Input
    content:
    - name: Line Following with IR Sensors
      link: 04_line_following.html
    - name: Obstacle Avoidance with Ultrasonic Sensors
      link: 05_ultrasonic_sensor.html
  - section: Robot Assembly
    content:
    - name: Building a Two-Wheel Drive Robot
      link: 06_build_robot.html
    - name: Servo Scanning with Ultrasonic Sensors
      link: 07_servo_scanning.html
  - section: Remote Control
    content:
    - name: Bluetooth Control with HC-05
      link: 08_bluetooth_control.html
    - name: Wi-Fi Control with the Raspberry Pi Pico W
      link: 09_wifi_control.html
  - section: Autonomous Behavior
    content:
    - name: Creating Simple Autonomous Behavior
      link: 10_autonomous_robot.html
  - section: Final Project
    content:
    - name: "Final Project \u2013 Build Your Own Robot"
      link: 11_final_project.html
  - section: Summary
    content:
    - name: "Course Summary and What\u2019s Next"
      link: 12_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

DC motors are the core of many robotics projects — they provide movement for wheels, arms, and even propellers. In this lesson, you'll learn how to control them using your Raspberry Pi Pico and a motor driver.

---

## 🔌 What You’ll Need

To follow along, gather the following components:

Item              | Description                                                                                                                | Quantity
------------------|----------------------------------------------------------------------------------------------------------------------------|----------
Raspberry Pi Pico | [Raspberry Pi Pico / Pico 2](https://www.raspberrypi.com/products/raspberry-pi-pico/) or Pico W / Pico 2 W Microcontroller | 1
Motor Driver      | L298N motor driver module                                                                                                  | 1
Motors            | Small DC motors, either the yellow TT or N20 style                                                                         | 2
Power             | External power supply (e.g. 4xAA batteries or 2S Li-ion)                                                                   | 1
Wires             | Breadboard and jumper wires                                                                                                | As needed
{:class="table table-striped"}

---

## 🧠 How It Works

The Raspberry Pi Pico can't drive a motor directly — it doesn't output enough current. Instead, we use an **L298N H-bridge motor driver**, which allows the Pico to control the motor’s speed and direction using:

- **IN1 and IN2**: Control direction
- **ENA**: Controls speed via PWM
- **VCC**: Connects to motor power supply
- **GND**: Common ground
- **5V**: Optional logic power (can be left unused if powering Pico separately)

---

## 🛠️ Wiring the Circuit

Here’s how to connect a single motor to the L298N and Pico:

- Pico GP0 → IN1  
- Pico GP1 → IN2  
- Pico GP2 → ENA (PWM control)  
- Motor → Out1 and Out2  
- Battery pack + → L298N VCC  
- Battery pack – → L298N GND  
- Pico GND → L298N GND  

> ⚠️ Important: Do **not** connect motor power (VCC) directly to the Pico!

---

## 🧪 Writing the Code

Here's a simple MicroPython script to control motor direction and speed:

```python
from machine import Pin, PWM
from time import sleep

# Motor control pins
in1 = Pin(0, Pin.OUT)
in2 = Pin(1, Pin.OUT)
ena = PWM(Pin(2))
ena.freq(1000)

def motor_forward(speed=65025):
    in1.high()
    in2.low()
    ena.duty_u16(speed)

def motor_backward(speed=65025):
    in1.low()
    in2.high()
    ena.duty_u16(speed)

def motor_stop():
    in1.low()
    in2.low()
    ena.duty_u16(0)

# Test
motor_forward()
sleep(2)
motor_backward()
sleep(2)
motor_stop()
```

You can adjust the speed by changing the speed parameter (range: 0–65535).

---

## 🧩 Try It Yourself

* Reverse the motor direction.
* Slow the motor down by reducing PWM.
* Try turning the motor on/off with a button or switch.

Now that you can control a motor, you're one step closer to making your robot move!

Next up: [Using H-Bridge Motor Drivers](03_hbridge_control)

---
