---
layout: lesson
title: Wi-Fi Control with the Raspberry Pi Pico W
author: Kevin McAleer
type: page
cover: assets/wifi_control.jpg
date: 2025-05-20
previous: 08_bluetooth_control.html
next: 10_autonomous_robot.html
description: Control your robot using a smartphone or browser over Wi-Fi with the
  Raspberry Pi Pico W.
percent: 72
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

Bluetooth is great, but with the **Pico W**, you can control your robot over **Wi-Fi** — even from a browser on your phone or computer. In this lesson, we’ll host a simple web interface that sends commands to your robot in real time.

---

## 📡 What You’ll Need

- Raspberry Pi Pico W (instead of the standard Pico)
- A 2.4GHz Wi-Fi network
- Smartphone or computer with a modern browser

> ⚠️ This lesson requires the `rp2-pico-w` MicroPython firmware (with networking support).

---

## 🔌 Basic Setup

Update your Pico W with the latest MicroPython firmware, then connect it to Wi-Fi:

```python
import network
import time

ssid = 'YOUR_SSID'
password = 'YOUR_PASSWORD'

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(ssid, password)

while not wlan.isconnected():
    time.sleep(1)

print('Connected, IP:', wlan.ifconfig()[0])
```

---

## 🌐 Serve a Web Page for Control

This example creates a web server that lets you send movement commands by clicking buttons:

```python
import socket

html = """<!DOCTYPE html>
<html>
<head><title>Robot Control</title></head>
<body>
<h2>Robot Control Panel</h2>
<form>
<button name="cmd" value="f">Forward</button>
<button name="cmd" value="b">Backward</button><br><br>
<button name="cmd" value="l">Left</button>
<button name="cmd" value="r">Right</button><br><br>
<button name="cmd" value="s">Stop</button>
</form>
</body>
</html>
"""

addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]

s = socket.socket()
s.bind(addr)
s.listen(1)

print("Listening on", addr)

while True:
    cl, addr = s.accept()
    print("Client connected:", addr)
    request = cl.recv(1024).decode()
    print("Request:", request)

    if '/?cmd=f' in request:
        forward()
    elif '/?cmd=b' in request:
        backward()
    elif '/?cmd=l' in request:
        left()
    elif '/?cmd=r' in request:
        right()
    elif '/?cmd=s' in request:
        stop()

    cl.send('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
    cl.send(html)
    cl.close()
```

Make sure to define forward(), backward(), etc., from earlier lessons.

---

## 🧪 Test It Out

- Connect the Pico W to Wi-Fi
- Open the IP address shown in the REPL on your phone’s browser
- Use the buttons to control your robot!

---

## 🧩 Try It Yourself

- Add more buttons (e.g., for speed control)
- Style the interface with CSS
- Use AJAX for real-time control without reloading

---

Now your robot has a wireless dashboard, ready to drive from across the room!

Next up: [Creating Simple Autonomous Behavior](10_autonomous_robot)

---
