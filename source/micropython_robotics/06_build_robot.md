---
title: Building a Two-Wheel Drive Robot
description: Assemble a simple two-wheeled robot chassis using motors, sensors, and the Raspberry Pi Pico.
layout: lesson
type: page
cover: assets/robot_assembly.jpg
date_updated: 2025-05-10
---

![Cover](assets/04.jpg){:class="cover"}

---

Now that you’ve mastered motor and sensor control, it’s time to build your robot! In this lesson, we’ll assemble a two-wheel drive robot chassis that will act as the base for the remaining lessons.

---

## 🧱 Components You'll Need

- **Chassis kit** (with mounting holes for motors and wheels)
- **2x DC gear motors**
- **2x Wheels**
- **Caster wheel** (or ball bearing support)
- **L298N motor driver**
- **Raspberry Pi Pico**
- **IR line sensors (x2)**
- **Ultrasonic sensor (HC-SR04)**
- **Power supply** (e.g. 4xAA battery pack or 2S Li-ion)
- **Jumper wires / breadboard / Dupont cables**
- **Screws, nuts, double-sided tape or hot glue**

---

## 🛠️ Assembly Instructions

1. **Attach Motors to the Chassis**  
   Secure each DC motor to the motor mounts on your chassis. Make sure both motor shafts face the same direction.

2. **Connect the Wheels**  
   Push the wheels onto the motor shafts. Add a **caster wheel** or similar support at the back or front for balance.

3. **Mount the L298N and Pico**  
   Use double-sided tape or standoffs to fix the L298N and Raspberry Pi Pico to the chassis.

4. **Wire the Motors**  
   Connect both motors to the L298N outputs (OUT1–OUT4), then wire the control pins to the Pico as described in previous lessons.

5. **Add Sensors**  
   - Mount the **IR sensors** near the front underside, facing the ground.
   - Mount the **ultrasonic sensor** at the front facing forward. Optionally attach it to a servo.

6. **Power Up**  
   Connect your battery pack:
   - Battery + → L298N VCC  
   - Battery – → L298N GND  
   - Pico GND → L298N GND  
   - Power the Pico separately via USB or from L298N 5V pin (if jumper is present and you're using 6V–12V input).

> ⚠️ **Double-check polarity before powering on!**

---

## 🎛️ Testing the Setup

Run the motor control and sensor code from previous lessons to verify:

- Motors spin in the correct direction
- IR sensors detect a line
- Ultrasonic sensor reports distance

Make adjustments to wheel orientation or wiring as needed.

---

## 🧩 Try It Yourself

- Experiment with motor speed and turning radius.
- Reposition sensors for better detection.
- Add a power switch or LED indicator.

---

You now have a complete robot chassis ready for autonomous or remote-controlled behavior!

Next up: [Servo Scanning with Ultrasonic Sensors](07_servo_scanning)

---
