---
layout: lesson
title: Using H-Bridge Motor Drivers
author: Kevin McAleer
type: page
cover: assets/hbridge.jpg
date: 2025-05-20
previous: 02_dc_motors.html
next: 04_line_following.html
description: Understand how H-bridge motor drivers like the L298N control motor direction
  and speed for two-wheeled robots.
percent: 24
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


![Cover](assets/01.jpg){:class="cover"}

---

An **H-bridge** is a circuit that lets you control the direction of a DC motor by switching the current flow. It’s essential for robotics because it allows you to drive motors forward and backward with just a few control signals.

---

## ⚙️ How the H-Bridge Works

The H-bridge gets its name from the circuit diagram, which looks like an “H” made of transistors or switches.

{% include gallery.html titles="H Bridge" images="assets/hbridge.jpg" descriptions="The S1 - S4 in this diagram are Transistors that act like switches. The motors are controlled by these switches." noborder=true cols=2 %}

---

It allows current to flow in either direction through the motor:

- IN1 high, IN2 low → forward
- IN1 low, IN2 high → backward
- IN1 and IN2 same → brake/stop

When paired with **PWM on the enable pin**, you can also control the **speed** of the motor.

---

## 🔌 Wiring Two Motors with L298N

{% include gallery.html titles="L298N Motor Driver" images="assets/l298n.png" descriptions="Works with up to 12v for powerful motors." cols=2 %}

---

The L298N has two H-bridges inside — perfect for controlling two motors independently. Here’s a typical wiring setup:

- **Motor A**
  - GP0 → IN1  
  - GP1 → IN2  
  - GP2 (PWM) → ENA  

- **Motor B**
  - GP3 → IN3  
  - GP4 → IN4  
  - GP5 (PWM) → ENB  

`GP` pins are the GPIO pins on the Pico.

{% include gallery.html images="assets/pico_pinouts.jpg" titles="Raspberry Pi Pico Pinouts" descriptions="Each of the green edge connectors are referred to as GPIO or General Purpose Input Output pins. The black are the common ground pins." cols=2 %}

---

> Don't forget: Pico GND must connect to L298N GND.

---

## 🧪 Writing the Code

Here’s a MicroPython script for controlling both motors:

```python
from machine import Pin, PWM
from time import sleep

# Motor A
in1 = Pin(0, Pin.OUT)
in2 = Pin(1, Pin.OUT)
ena = PWM(Pin(2))
ena.freq(1000)

# Motor B
in3 = Pin(3, Pin.OUT)
in4 = Pin(4, Pin.OUT)
enb = PWM(Pin(5))
enb.freq(1000)

def motor_a_forward(speed=50000):
    in1.high()
    in2.low()
    ena.duty_u16(speed)

def motor_a_backward(speed=50000):
    in1.low()
    in2.high()
    ena.duty_u16(speed)

def motor_b_forward(speed=50000):
    in3.high()
    in4.low()
    enb.duty_u16(speed)

def motor_b_backward(speed=50000):
    in3.low()
    in4.high()
    enb.duty_u16(speed)

def stop_motors():
    in1.low()
    in2.low()
    in3.low()
    in4.low()
    ena.duty_u16(0)
    enb.duty_u16(0)

# Test
motor_a_forward()
motor_b_forward()
sleep(2)

motor_a_backward()
motor_b_backward()
sleep(2)

stop_motors()
```

---

## 🤖 Why This Matters

With two motors under your control, you can:

- Drive forward/backward

- Turn left/right

- Rotate on the spot (tank steering)

This forms the basic movement logic for most two-wheeled robots.

---

## 🧩 Try It Yourself

Try setting different speeds for left and right motors.

- Create a function to “spin in place.”

- Add buttons to manually control each motor.

---

You’re now ready to add some intelligence to your robot!

Next up: [Line Following with IR Sensors](04_line_following)

---