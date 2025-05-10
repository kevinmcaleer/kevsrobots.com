---
layout: lesson
title: Obstacle Avoidance with Ultrasonic Sensors
author: Kevin McAleer
type: page
cover: assets/ultrasonic_sensor.jpg
date: 2025-05-20
previous: 04_line_following.html
next: 06_build_robot.html
description: Use an ultrasonic sensor to detect obstacles and prevent your robot from
  crashing into things.
percent: 40
duration: 2
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


![Cover](assets/03.jpg){:class="cover"}

---

Your robot needs to be aware of its surroundings. With an **ultrasonic sensor** like the HC-SR04, you can measure the distance to objects and make your robot avoid collisions.

---

## 📏 How the Ultrasonic Sensor Works

The **HC-SR04** sends out a burst of ultrasound and listens for the echo. By measuring the time it takes for the echo to return, it can calculate distance.

- **Trigger**: Start a measurement
- **Echo**: Pulse length corresponds to distance

> Distance is calculated as:  
> `distance (cm) = duration (µs) / 58`

---

## 🔌 Wiring the HC-SR04 to the Pico

![HC-SR04](assets/hc-sr04.jpg){:class="w-100 rounded-3"}

- **VCC** → 3.3V (The Pico needs the 3.3v versino of the HC-SR04)  
- **GND** → GND  
- **TRIG** → GP8  
- **ECHO** → GP9 (use a voltage divider or level shifter to avoid damaging the Pico!)

**Voltage divider for ECHO:**

```text
ECHO → 1kΩ → Pico GP9
           ↓
         2kΩ
           ↓
         GND
```

---

## 🧪 Reading Distance with MicroPython

```python
from machine import Pin, time_pulse_us
from time import sleep

trigger = Pin(8, Pin.OUT)
echo = Pin(9, Pin.IN)

def get_distance():
    trigger.low()
    sleep(0.002)
    trigger.high()
    sleep(0.01)
    trigger.low()

    duration = time_pulse_us(echo, 1, 30000)  # timeout after 30ms
    distance_cm = duration / 58
    return distance_cm

while True:
    dist = get_distance()
    print("Distance:", dist, "cm")
    sleep(0.5)
```

---

## 🧠 Avoiding Obstacles

You can use the measured distance to decide what your robot should do:

- Distance < 10 cm → Stop or turn
- Distance ≥ 10 cm → Keep moving

```python
distance = get_distance()

if distance < 10:
    stop()
    turn_right()
else:
    forward()
```

---

## 🧩 Try It Yourself

Adjust the threshold to make your robot more or less cautious.

- Add random turns when an obstacle is detected.
- Mount the sensor on a servo and scan side-to-side.

---

With obstacle detection working, your robot now reacts to the real world!
Next up: [Building a Two-Wheel Drive Robot](06_build_robot)

---
