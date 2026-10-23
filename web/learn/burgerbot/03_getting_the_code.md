---
layout: lesson
title: Getting the code
author: Kevin McAleer
type: page
cover: /learn/burgerbot/assets/burgerbot.jpg
date: 2023-04-27
previous: 02_video.html
next: 04_loading_the_code.html
description: Learn how to use the BurgerBot MicroPython library to control the robotic
  vehicle.
percent: 36
duration: 3
quiz_total: 11
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
    - name: Drawing with a Pen
      link: 08_pen.html
    - name: Line Following
      link: 08_line_following.html
    - name: Line Following Code
      link: 08_line_following_code.html
    - name: Camera
      link: 09_camera.html
---


## BurgerBot MicroPython library

BurgerBot has a small MicroPython library that does the hard work of driving the motors and reading the sensors, so your own programs can be short and readable. It controls the motors, the range finder, the line sensor and the pen servo.

---

## Download the library

The library is a single file, `burgerbot.py`, in the [BurgerBot GitHub repository](https://github.com/kevinmcaleer/burgerbot).

1. Open [burgerbot.py on GitHub](https://github.com/kevinmcaleer/burgerbot/blob/main/burgerbot.py)
2. Click the **Download raw file** button
3. Copy it onto your Pico in the next lesson

The library works with the standard [MicroPython firmware for the Pico](/learn/how_to_install_micropython/). It only uses the built-in `machine`, `time` and `gc` modules, so you don't need any special firmware.

> ## Older versions
>
> Earlier versions of the library needed Pimoroni's MicroPython build and used method names like `turnleft()`. If you're following an old video, grab the latest `burgerbot.py` and use the names below.

---

## The Burgerbot class

Method or property | Description
---|---
`Burgerbot()` | Creates the robot object and sets the speed to 0.5
`forward(duration)` | Drives forward for `duration` seconds (default 0.5)
`backward(duration)` | Drives backward for `duration` seconds
`turn_left(duration)` | Spins left on the spot for `duration` seconds
`turn_right(duration)` | Spins right on the spot for `duration` seconds
`stop()` | Stops both motors
`left_motor(speed)` / `right_motor(speed)` | Sets one motor's speed, from -1 (full reverse) to 1 (full forward)
`speed` | The speed used by `forward()`, `backward()` and the turns, from -1 to 1. Read it with `bot.speed`, or change it with `bot.speed = 0.3`
`distance` | The distance to the nearest object from the range finder
`line_detected` | `True` when the line sensor sees a line
`pen_up()` / `pen_down()` / `pen_middle()` | Moves the pen servo
{:class="table table-code-fit"}

> ## Movement keeps going
>
> `forward(2)` starts the motors and waits for 2 seconds, but it doesn't stop them. Call `bot.stop()` when you want the robot to stop.

---

## Pins used

The library expects this wiring:

Part | Pico pins
---|---
Left motor | GP5 (speed), GP6 (direction)
Right motor | GP12 (speed), GP13 (direction)
Range finder | GP0 (trigger), GP1 (echo)
Pen servo | GP16
Line sensor | GP17
{:class="table table-code-fit"}

---

## Example demo

```python
from burgerbot import Burgerbot

bot = Burgerbot()
bot.speed = 0.5

bot.forward(1)
bot.turn_left(0.5)
bot.forward(1)
bot.stop()

print("Distance ahead:", bot.distance)
```

---

{% quiz %}
? What happens after `bot.forward(2)` finishes?
- [ ] The robot stops automatically
- [x] The motors keep running until you call `bot.stop()` or another movement
- [ ] The robot reverses
> `forward()` sets the motor speed and waits. It doesn't stop the motors afterwards.

? How do you make BurgerBot drive more slowly?
- [ ] `bot.forward(0.3)`
- [x] `bot.speed = 0.3`
- [ ] `bot.left_motor(3)`
> `speed` sets how fast the motors run. The number passed to `forward()` is how long to drive for.
{% endquiz %}

---
