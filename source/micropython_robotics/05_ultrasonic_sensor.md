---
title: Obstacle Avoidance with Ultrasonic Sensors
description: Use an ultrasonic sensor to detect obstacles and prevent your robot from crashing into things.
layout: lesson
type: page
cover: assets/ultrasonic_sensor.jpg
date_updated: 2025-05-10
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
