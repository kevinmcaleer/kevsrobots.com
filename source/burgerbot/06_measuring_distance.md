---
title: Measuring Distance
description: Learn how to measure distance using the range finder on BurgerBot.
layout: lesson
type: page
---

## Ultrasonic Range Finder

Burgerbot includes a simple range finder sensor at the front of the robot.

![Line folllow animation](assets/line_follow_animation.gif)

Ultrasonic range finders are versatile and widely used devices that measure distance by utilizing the principles of sound wave propagation. These handy gadgets have found their way into a plethora of applications, from robotics and automation to obstacle detection and parking assistance. 

---
## How Ultrasonic Range Finders Work

Ultrasonic range finders operate using the basic principles of echolocation, a technique employed by animals such as bats and dolphins to navigate and locate objects. Here's a step-by-step explanation of the process:

**Transmitter**: The range finder consists of a transmitter that emits ultrasonic sound waves, typically at frequencies between 20 kHz and 200 kHz. These frequencies are higher than the audible range for humans, which is why they're called `"ultrasonic"`.

**Sound Wave Propagation**: The emitted sound waves travel through the air and eventually reach an object or surface in their path.

**Reflection**: Upon hitting the object, the sound waves reflect back towards the range finder.

**Receiver**: The device has a built-in receiver that detects the reflected sound waves, often using a piezoelectric element that converts the mechanical vibration of the sound waves into an electrical signal.

**Time-of-Flight Calculation**: The range finder measures the time it takes for the emitted sound waves to travel to the object and bounce back. This is known as the "time of flight."

**Distance Calculation** : Using the time of flight and the known speed of sound in air (approximately 343 meters per second), the range finder calculates the distance to the object using the following formula:

Distance = (Time of Flight x Speed of Sound) / 2

The division by 2 accounts for the fact that the sound waves travel to the object and back, covering twice the actual distance.

---

## MicroPython Example

Here is example code for using a range finder. This is simplified within the burgerbot:

<script src="https://gist.github.com/kevinmcaleer/c7e2a95adeaf6215617f9dcfe5ce880a.js"></script>

---

## Using the range finder with BurgerBot

To measure distance with BurgerBot simple use the distance property - it will take a measurement and return it:

```python
bot = Burgerbot()
distance = bot.distance
```

---
