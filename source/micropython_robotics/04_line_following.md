---
title: Line Following with IR Sensors
description: Learn how to detect and follow a line using an IR sensor and MicroPython.
layout: lesson
type: page
cover: assets/line_following.jpg
date_updated: 2025-05-10
---

![Cover](assets/02.jpg){:class="cover"}

---

Line-following robots are one of the most classic and satisfying robotics projects. In this lesson, you'll learn how to use an IR sensor to detect a black line on a white surface and steer your robot to follow it.

---

## 🕵️ How IR Line Sensors Work

IR (infrared) sensors detect reflected light. They shine infrared light onto the surface and measure how much bounces back:

- **White/light surface** → reflects more → HIGH signal  
- **Black/dark line** → absorbs more → LOW signal  

> Most basic IR line sensors use a digital output, but some give analog readings.

We’ll use a common TCRT5000 or similar sensor with digital output for this project.

---

## 🔌 Wiring the IR Sensor

Wire the sensor to the Pico as follows:

- **VCC** → 3.3V  
- **GND** → GND  
- **OUT** → GP6 (or any free digital pin)  

> If you're using two sensors (left and right), wire the second sensor’s output to GP7.

---

## 🧪 Reading the Sensor in MicroPython

```python
from machine import Pin
from time import sleep

# Sensor input pin
sensor = Pin(6, Pin.IN)

while True:
    if sensor.value() == 0:
        print("Line detected")
    else:
        print("No line")
    sleep(0.1)
```

---

Try placing your sensor over black tape on a white surface to see the output change.

## 🧠 Line Following Logic

With two sensors (left and right), you can create simple decision-making logic:

- Left sensor off, right on → turn left
- Left on, right off → turn right
- Both on → move forward
- Both off → stop or reverse

---

## 🧰 Example: Two-Sensor Line Follower

```python
from machine import Pin, PWM
from time import sleep

# Motor pins (as in previous lesson)
# ...

# IR sensors
left_sensor = Pin(6, Pin.IN)
right_sensor = Pin(7, Pin.IN)

def forward():
    # your forward motor code here
    pass

def turn_left():
    # slow or stop left motor, run right motor
    pass

def turn_right():
    # slow or stop right motor, run left motor
    pass

def stop():
    # stop both motors
    pass

while True:
    left = left_sensor.value()
    right = right_sensor.value()

    if left == 0 and right == 1:
        turn_left()
    elif left == 1 and right == 0:
        turn_right()
    elif left == 0 and right == 0:
        forward()
    else:
        stop()

    sleep(0.05)
```

---

## 🧩 Try It Yourself

- Tune the logic to make turns smoother or sharper.
- Try an analog sensor and adjust thresholds.
- Experiment with different surface colors and materials.

---

This is your robot's first real autonomous behavior!

Next up: [Obstacle Avoidance with Ultrasonic Sensors](05_ultrasonic_sensor)

---