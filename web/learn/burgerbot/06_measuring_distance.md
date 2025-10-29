---
layout: lesson
title: Measuring Distance
author: Kevin McAleer
type: page
cover: /learn/burgerbot/assets/burgerbot.jpg
date: 2023-04-27
previous: 05_basic_movement.html
next: 07_object_avoidance.html
description: Learn how to measure distance using the range finder on BurgerBot.
percent: 63
duration: 5
navigation:
- name: Build your own BurgerBot
- content:
  - section: Overview
    content:
    - name: Overview
      link: 00_intro.html
    - name: Why BurgerBot?
      link: 01_why_video.html
    - name: What is BurgerBot?
      link: 01_what_is_burger_bot.html
  - section: Assembling the parts
    content:
    - name: Assembly
      link: 02_assembly.html
    - name: BurgerBot Assembly video
      link: 02_video.html
  - section: Installing the code
    content:
    - name: Getting the code
      link: 03_getting_the_code.html
    - name: Loading the code
      link: 04_loading_the_code.html
  - section: Programming in MicroPython
    content:
    - name: Basic Movement
      link: 05_basic_movement.html
    - name: Measuring Distance
      link: 06_measuring_distance.html
    - name: Object Avoidance
      link: 07_object_avoidance.html
  - section: Upgrades
    content:
    - name: BurgerBot Upgrades video
      link: 08_video.html
    - name: Line Following
      link: 08_line_following.html
    - name: Line Following Code
      link: 08_line_following_code.html
    - name: Camera
      link: 09_camera.html
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

## Using the Range Finder with BurgerBot

The BurgerBot library makes it incredibly easy to measure distance - just use the `bot.distance` property!

**The code:**

```python
from burgerbot import Burgerbot

bot = Burgerbot()

# Take a single distance measurement
distance = bot.distance
print(f"Distance to object: {distance} cm")
```

**How it works:**

- `bot.distance` triggers the rangefinder to send ultrasonic pulses
- The sensor measures time-of-flight automatically
- Returns the distance in centimeters as a number
- If no object is detected, returns a very large value (like 400cm)

---

## Continuous Distance Monitoring

For robotics applications, you typically want to monitor distance continuously:

```python
from burgerbot import Burgerbot
import time

bot = Burgerbot()

# Monitor distance every 0.5 seconds
while True:
    distance = bot.distance
    print(f"Distance: {distance} cm")

    if distance < 10:
        print("Warning: Object very close!")
    elif distance < 30:
        print("Object detected nearby")
    else:
        print("Path is clear")

    time.sleep(0.5)  # Wait before next reading
```

**What this does:** Continuously reads distance and provides warnings based on proximity.

---

## Practical Application: Distance-Based Actions

Here's a real-world example that changes robot behavior based on distance:

```python
from burgerbot import Burgerbot

bot = Burgerbot()

distance = bot.distance

if distance > 50:
    print("Path clear - full speed ahead!")
    bot.forward(2)
elif distance > 20:
    print("Object detected - approaching carefully")
    bot.forward(1)
elif distance > 10:
    print("Getting close - slowing down")
    bot.forward(0.5)
else:
    print("Too close! Stopping")
    bot.stop()
```

**Why this is useful:** Robots can adjust their speed based on proximity to obstacles, creating smoother and safer navigation.

---

## Common Issues

- **Problem**: `bot.distance` always returns 0
- **Solution**: Check that the rangefinder Trigger pin and Echo pin are properly connected to the Pico GPIO pins. Verify wiring matches the Motor SHIM documentation
- **Why**: The rangefinder needs both pins connected to send pulses (Trigger) and receive echoes (Echo)

- **Problem**: Distance readings are erratic or jump between values
- **Solution**: Take multiple readings and average them, or add a small delay between readings
- **Why**: Ultrasonic sensors can occasionally pick up echoes from multiple surfaces, causing temporary fluctuations

- **Problem**: Can't detect objects closer than 2-3cm
- **Solution**: This is normal behavior for ultrasonic sensors - they have a minimum detection range
- **Why**: The sound waves need physical space to separate the outgoing pulse from the incoming echo

- **Problem**: Sensor doesn't detect soft or angled surfaces well
- **Solution**: Use objects with flat, hard surfaces for testing. Angled surfaces reflect sound away from the sensor
- **Why**: Soft materials absorb ultrasonic waves, and angles cause waves to bounce away rather than back to the sensor

- **Problem**: Getting very large numbers (like 400cm) when nothing is nearby
- **Solution**: This is normal - it means no object was detected within range. The sensor has a maximum detection range of about 4 meters
- **Why**: When no echo returns within the timeout period, the sensor returns its maximum value

---

## Try it Yourself

Now that you can measure distance, let's practice:

1. **Distance logger**: Create a program that measures distance every second and saves it to a list. After 10 measurements, print the average distance.

2. **Object detector**: Write code that continuously monitors distance and beeps (print "BEEP!") when an object comes within 15cm.

3. **Distance zones**: Create three zones:
   - **Far** (>40cm): Print "Clear"
   - **Medium** (15-40cm): Print "Caution"
   - **Near** (<15cm): Print "Stop!"

4. **Movement speed controller**: Combine distance measurement with movement - make the robot speed up when far from objects and slow down when close.

**Advanced Challenge**: Create a "parking assistant" - move the robot forward until it's exactly 10cm from a wall, then stop. The robot should slow down as it approaches the target distance!

---

## What's Next?

Now that you understand how to measure distance, the next lesson will teach you how to use this information for **object avoidance** - making your BurgerBot navigate autonomously!

---
