---
title: Controlling DC Motors with the Raspberry Pi Pico
description: Learn how to connect and control DC motors using MicroPython and the L298N motor driver.
layout: lesson
type: page
cover: /learn/micropython_robotics/assets/banners/02_dc_motors.jpg
date_updated: 2025-05-10
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
