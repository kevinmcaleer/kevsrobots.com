---
layout: lesson
title: Bluetooth Control with HC-05
author: Kevin McAleer
type: page
cover: /learn/micropython_robotics/assets/banners/08_bluetooth_control.jpg
date: 2025-05-20
previous: 07_servo_scanning.html
next: 09_wifi_control.html
description: Learn how to control your robot remotely using Bluetooth and a smartphone
  or computer.
percent: 64
duration: 6
quiz_total: 36
date_updated: 2026-11-24
navigation:
- name: MicroPython Robotics Projects with the Raspberry Pi Pico
- content:
  - section: Introduction
    content:
    - name: Introduction to MicroPython Robotics Projects
      link: 00_intro.html
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
    - name: Final Project - Build Your Own Robot
      link: 11_final_project.html
  - section: Summary
    content:
    - name: Course Summary and What's Next
      link: 12_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to wire an HC-05 Bluetooth module to a Pico UART and drive the robot with single-letter commands from a phone, with a safety stop if the signal is lost.

With Bluetooth you can drive your robot from a phone with no cable. The HC-05 is a small module that turns Bluetooth into a plain **serial** connection. Whatever your phone sends arrives as bytes on the Pico's UART, and whatever the Pico writes goes back to the phone.

## What you'll need

- HC-05 Bluetooth module
- The robot from lesson 6, with `robot.py` on the Pico
- An Android phone or a computer with a Bluetooth serial terminal app

The HC-05 uses classic Bluetooth serial (SPP). Android phones and most computers can pair with it, but iPhones generally cannot, so use a computer or an Android phone. (The Pico W has its own Bluetooth Low Energy radio, which is a different topic from the HC-05's classic serial, and is not covered here.)

## Wiring the HC-05 to the Pico

The first thing to notice: UART0's default pins are GP0 and GP1, but our robot already uses those for the left motor. So we use another UART0 pair, **GP12 (TX) and GP13 (RX)**.

| HC-05 pin | Connect to |
|-----------|------------|
| VCC | 5 V rail (most breakout boards take 3.6 V to 6 V; check yours) |
| GND | GND |
| TXD | Pico GP13 (RX, physical pin 17) |
| RXD | Pico GP12 (TX, physical pin 16) |
{:class="table table-single"}

Remember the cross-over: the module's **TXD** goes to the Pico's **RX**, and the Pico's TX goes to the module's RXD. The HC-05's data pins work at 3.3 V logic, which matches the Pico, so no voltage divider is needed here. (A divider would be needed if you connected a 5 V board like an Arduino Uno.)

```mermaid
flowchart LR
    PH["Phone terminal app"] <-->|"Bluetooth"| HC["HC-05"]
    HC -->|"TXD to GP13 (Pico RX)"| PICO["Pico UART0"]
    PICO -->|"GP12 (Pico TX) to RXD"| HC
    PICO --> BOT["Robot motors"]
```

## The control program

The commands are single letters: `f` forward, `b` backward, `l` spin left, `r` spin right and `s` stop. The HC-05's default serial speed in normal mode is 9600 baud, though some modules differ, so check yours if you only see gibberish.

There is one more safety feature: if no command arrives for 600 ms the robot stops. If the phone goes out of range while the robot is driving, you do not want a runaway robot.

```python
from machine import UART, Pin
from time import ticks_ms, ticks_diff, sleep_ms
from robot import Robot

uart = UART(0, baudrate=9600, tx=Pin(12), rx=Pin(13))
bot = Robot()

SPEED = 60
TIMEOUT_MS = 600           # stop if no command for this long

last_command = ticks_ms()

print("Waiting for Bluetooth commands: f b l r s")

while True:
    if uart.any():
        cmd = uart.read(1)
        last_command = ticks_ms()
        if cmd == b"f":
            bot.forward(SPEED)
        elif cmd == b"b":
            bot.backward(SPEED)
        elif cmd == b"l":
            bot.spin_left(SPEED)
        elif cmd == b"r":
            bot.spin_right(SPEED)
        elif cmd == b"s":
            bot.stop()
        uart.write(b"ok\n")        # a reply, so you can see it works on the phone
    elif ticks_diff(ticks_ms(), last_command) > TIMEOUT_MS:
        bot.stop()
    sleep_ms(10)
```

`ticks_ms()` and `ticks_diff()` are the right way to time things on MicroPython, because the counter wraps around and `ticks_diff()` handles that for you. Plain subtraction would not.

## Pairing and driving

1. Power the robot. The HC-05 LED starts blinking quickly while it waits for a connection.
2. In your phone or computer's Bluetooth settings, pair with the HC-05. The PIN is usually 1234, and sometimes 0000.
3. Open a Bluetooth serial terminal app and connect to the HC-05.
4. Type `f` and the robot moves. Since it stops after 600 ms of silence, tap repeatedly, or use a joystick-style app that repeats the letter while you hold a button.

> **Teachers**: Many HC-05 modules in one room all appear with the same name and the same PIN. Rename each module with AT commands beforehand, or number the labels on the robots and switch on one at a time to pair. Phones and laptops belonging to learners may not allow Bluetooth in some schools, so keep one spare computer with a terminal app per group.

## Common Issues

- **Problem**: Nothing arrives, or the output is gibberish.
- **Solution**: Swap the TXD and RXD wires, and check the baud rate is 9600 (or what your module is set to).
- **Why**: Serial lines must cross over, and both ends must agree on the speed.

- **Problem**: The robot twitches only for a moment after each key press.
- **Solution**: That is the 600 ms safety stop. Send the letter repeatedly, or raise `TIMEOUT_MS`.
- **Why**: Without a recent command the program assumes the phone is out of range and stops.

- **Problem**: Pairing works but the terminal app cannot connect.
- **Solution**: Check you are using a serial terminal app, not just the phone's Bluetooth settings, and that the module is not already connected to another device.
- **Why**: The HC-05 only allows one connection at a time.

## Try it Yourself

1. Add a command `1` to `9` that sets the speed, for example `1` for 10 percent and `9` for 90 percent.
2. Send a message back with the distance from the sonar when you send `d`.
3. Use a joystick app that sends letters, or make a remote with MIT App Inventor.
4. Challenge: stop the robot automatically if the ultrasonic sensor sees something within 10 cm while driving forward.

## Check your understanding

{% quiz %}
? Why is the HC-05 on GP12 and GP13 rather than GP0 and GP1?
- [ ] GP0 and GP1 cannot do UART
- [x] GP0 and GP1 already drive the left motor
- [ ] The HC-05 only works on the higher pin numbers
> GP12 and GP13 are another UART0 pair, so the motor and the radio can both stay connected.

? The module's TXD wire goes to which Pico pin?
- [ ] The Pico's TX pin, GP12
- [x] The Pico's RX pin, GP13
- [ ] 3V3
> Serial lines cross over: one side's transmit is the other side's receive.

? What is the timeout in the program for?
- [x] Stopping the robot if no command has arrived recently
- [ ] Pairing with the phone
- [ ] Slowing the baud rate
> If the phone goes out of range the robot stops after 600 ms rather than running away.
{% endquiz %}
