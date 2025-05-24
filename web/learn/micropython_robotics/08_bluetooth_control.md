---
layout: lesson
title: Bluetooth Control with HC-05
author: Kevin McAleer
type: page
cover: assets/bluetooth_control.jpg
date: 2025-05-20
previous: 07_servo_scanning.html
next: 09_wifi_control.html
description: Learn how to control your robot remotely using Bluetooth and a smartphone
  or computer.
percent: 64
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


![Cover](assets/02.jpg){:class="cover"}

---

With Bluetooth, you can control your robot wirelessly from a phone, tablet, or PC — no need to tether it with a USB cable. In this lesson, we’ll connect an HC-05 module to the Raspberry Pi Pico and send simple commands to drive the robot.

---

## 🔌 What You’ll Need

- HC-05 Bluetooth module
- Raspberry Pi Pico
- Jumper wires
- Smartphone or computer with a serial terminal app (e.g., Bluetooth Terminal, Serial Bluetooth Terminal on Android)

---

## ⚠️ Wiring the HC-05 to the Pico

| HC-05 Pin | Connect to                                                                              |
|-----------|-----------------------------------------------------------------------------------------|
| VCC       | 3.3V or 5V (check module label)                                                         |
| GND       | GND                                                                                     |
| TXD       | **GP1** (Pico RX) via voltage divider (1kΩ + 2kΩ)                                       |
| RXD       | **GP0** (Pico TX) – use **voltage divider** to step down 3.3V if HC-05 is 3.3V tolerant |
{:class="table table-striped"}

---

> 💡 The HC-05 RX pin **must not receive 3.3V directly** from Pico TX. Use a voltage divider (e.g., 1kΩ + 2kΩ).

---

## 🧪 Simple Bluetooth Control Script

```python
from machine import UART, Pin, PWM
from time import sleep

# Set up Bluetooth UART
uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))

# Motor pins (update to match your setup)
# ...

def forward():
    print("Forward")
    # motor code here

def backward():
    print("Backward")
    # motor code here

def left():
    print("Left")
    # motor code here

def right():
    print("Right")
    # motor code here

def stop():
    print("Stop")
    # motor stop code here

# Main loop
while True:
    if uart.any():
        cmd = uart.read(1)
        if cmd == b'f':
            forward()
        elif cmd == b'b':
            backward()
        elif cmd == b'l':
            left()
        elif cmd == b'r':
            right()
        elif cmd == b's':
            stop()
```

---

## 📱 Sending Commands from Your Phone

Use a Bluetooth terminal app and pair it with the HC-05 (usually pin 1234 or 0000). Then send single-character commands:

- f → forward
- b → backward
- l → left
- r → right
- s → stop

---

## 🧩 Try It Yourself

- Add more commands (e.g., speed control)
- Use a smartphone joystick app instead of typing letters
- Create a custom Bluetooth remote interface with MIT App Inventor

---

Now you can drive your robot like an RC car — all with MicroPython!

Next up: [Wi-Fi Control with the Raspberry Pi Pico W](09_wifi_control)

---
