---
layout: lesson
title: See with the Distance Module
author: Kevin McAleer
type: page
cover: /learn/robot_senses/assets/banners/05_see_distance.jpg
date: 2026-11-24
previous: 04_first_connection.html
next: 06_feel_movement.html
description: Read the Modulino Distance time-of-flight sensor, handle the None reading
  safely and compare it with the old HC-SR04 ultrasonic sensor
percent: 30
duration: 5
quiz_total: 10
date_updated: 2026-11-24
navigation:
- name: Give Your Robot Senses
- content:
  - section: Introduction
    content:
    - name: Introduction
      link: 00_intro.html
  - section: How robots sense
    content:
    - name: How Robots Sense
      link: 01_how_robots_sense.html
    - name: Meet Modulino SMARS
      link: 02_meet_modulino_smars.html
    - name: Qwiic and I2C
      link: 03_qwiic_and_i2c.html
    - name: Your First Connection
      link: 04_first_connection.html
  - section: One sense at a time
    content:
    - name: See with the Distance Module
      link: 05_see_distance.html
    - name: Feel Movement with the IMU
      link: 06_feel_movement.html
    - name: Speak with the Buzzer
      link: 07_speak_the_buzzer.html
    - name: Show with the LED Matrix
      link: 08_show_the_led_matrix.html
    - name: Move with the Motors
      link: 09_move_the_motors.html
  - section: From readings to behaviour
    content:
    - name: From Readings to Decisions
      link: 10_readings_to_decisions.html
    - name: Combining Senses
      link: 11_combining_senses.html
    - name: 'Project: The Reactive Robot'
      link: 12_project_reactive_robot.html
  - section: Navigation
    content:
    - name: Wall Following
      link: 13_wall_following.html
    - name: From Wall Following to Mapping and SLAM
      link: 14_mapping_and_slam.html
  - section: Wrapping up
    content:
    - name: Troubleshooting
      link: 15_troubleshooting.html
    - name: Summary
      link: 16_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

Sense one is sight. The **Modulino Distance** tells the robot how far away the nearest thing is. It's the sense that lets SMARS stop before it hits the skirting board.

## What it measures

The module carries a **VL53L4CD time-of-flight sensor**. It fires light and times how long it takes to bounce back. Light is so fast that the chip does the timing for you. Your code just asks for a number in centimetres.

The old SMARS used an HC-SR04 ultrasonic sensor. That one sends a 40 kHz ping and listens for the echo, and you time the echo yourself. We cover it in [Ultrasonic Range Finders](/learn/micropython_gpio/15_ultrasonic_range_finders.html). Here is how they compare.

| | HC-SR04 | Modulino Distance |
|---|---|---|
| Method | Sound echo | Light time-of-flight |
| Wiring | 4 wires to the board | One Qwiic cable |
| Timing | You time the pulse | The chip does it |
| Range | About 4 m | About 1.2 m |
{:class="table table-single"}

For a small tracked robot that mostly cares about close obstacles, the shorter range is a fair trade.

## Real-world robot example

This is a complete program. It builds the bus, creates the sensor and prints what it sees. Notice `i2c_bus=i2c`, the fix from the last lesson.

```python
from time import sleep_ms
from machine import I2C, Pin
from modulino import ModulinoDistance

i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400_000)
distance = ModulinoDistance(i2c_bus=i2c)

print("Put your hand in front of the sensor. Ctrl-C to stop.")
while True:
    print(distance.distance)    # centimetres, or None when out of range
    sleep_ms(200)
```
{: .no-snakie }

This program needs Arduino's `modulino` library from the last lesson, and it needs the real module, so it won't run on a bare board.

## The None trap

`distance.distance` returns `None`, not a number, when nothing is in range. Comparing `None` with a number crashes your loop:

<!-- code-check: skip -->
```python
if distance.distance < 10:    # TypeError when the reading is None
    print("close")
```

Kev's repository also found a second catch. The library's `distance` property can **raise** a `TypeError` when the sensor times out, instead of returning `None`. His fix catches it and treats it as "nothing in range". You can also see `OSError` from the I2C bus, and he handles that the same way.

Here is a safe wrapper. It's complete and runs anywhere, because it takes a function standing in for the sensor.

```python
def safe_distance(read, max_cm=200):
    """Return centimetres, or None when there is nothing usable."""
    try:
        d = read()
    except (TypeError, OSError):
        return None          # a timeout or a bus hiccup
    if d is None:
        return None          # nothing in range
    if d > max_cm:
        return None          # too far to trust
    return d

def describe(d):
    if d is None:
        return "nothing in range"
    if d < 10:
        return "very close"
    return "{:.1f} cm".format(d)

# Pretend sensors: a good reading, nothing, a far reading, and a timeout
def timeout():
    raise TypeError("sensor timed out")

for read in (lambda: 42.5, lambda: None, lambda: 350, timeout, lambda: 6.0):
    print(describe(safe_distance(read)))
```

On the real robot you'd pass `lambda: distance.distance` as `read`. The `200` cap comes from Kev's `config.py`, which calls anything wilder than 200 cm noise, not a reading.

**Why this matters:** an uncaught error here ends your whole run. A single bad reading shouldn't stop a robot that was otherwise driving fine.

**Common mistakes:**

- Comparing the reading before checking for `None`.
- Forgetting that the sensor is a narrow spot straight ahead. It won't see a chair leg off to one side.
- Printing too fast to read. Add `sleep_ms(200)`.

## Try it Yourself

1. Run the first program and move your hand slowly towards the sensor. What is the smallest number you see?
2. Point the sensor at the ceiling or out of the window. What does it print?
3. Challenge: change `describe()` so it prints "close" below 20 cm and "very close" below 10 cm.

## Common Issues

**Problem**: `RuntimeError: I2C interface couldn't be determined automatically`
**Solution**: Pass `i2c_bus=i2c` when you create `ModulinoDistance`.
**Why**: Your board isn't in Arduino's list, so the library needs the bus.

**Problem**: The loop crashes with `TypeError` when the path is clear.
**Solution**: Use `safe_distance()` or check for `None` first.
**Why**: `None` means "nothing in range", and you can't compare it with a number.

## Check your understanding

{% quiz %}
? What does `distance.distance` return when nothing is in range?
- [ ] `0`
- [x] `None`
- [ ] `-1`
> The sensor returns `None`. Comparing that with a number raises an error.

? Why does the Modulino Distance need no echo-timing code?
- [ ] It uses a camera
- [ ] It guesses
- [x] The sensor chip times the light itself
> The VL53L4CD does the timing and hands you a distance.

? Which is true of the Modulino Distance compared with the HC-SR04?
- [x] It reaches about 1.2 m, against about 4 m
- [ ] It reaches further
- [ ] It needs four wires
> It trades range for simplicity. That suits close-up obstacle stopping.
{% endquiz %}
