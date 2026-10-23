---
layout: lesson
title: Drawing with a Pen
author: Kevin McAleer
type: page
cover: /learn/burgerbot/assets/burgerbot.jpg
date: 2023-04-27
previous: 08_video.html
next: 08_line_following.html
description: Add a pen to BurgerBot and make it draw shapes.
percent: 72
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


## Watch the video

{% include youtubeplayer.html id="CYP3oL3Vs9o" %}

---

## The pen upgrade

BurgerBot can carry a felt-tip pen in the middle of the robot. A small SG90 servo raises and lowers it, so the robot can draw as it drives, a bit like the classic Logo turtle.

Because the pen sits between the wheels, BurgerBot can turn on the spot without smudging a curve into your drawing.

---

## Parts

* An SG90 servo
* A Sharpie or similar felt-tip pen
* 3D printed [servo holder](/assets/stl/burgerbot_v2/servo_holder.stl), [pen holder](/assets/stl/burgerbot_v2/pen_holder.stl) and [cog wheel](/assets/stl/burgerbot_v2/cog_wheel.stl)
* M3 nuts and bolts

The servo turns the cog, which pushes the pen holder up and down. Fit the servo holder to the robot, slide in the pen holder, then add the cog to the servo. Test fit the pen so it slides freely but doesn't wobble.

---

## Wiring the servo

Servo wire | Pico
---|---
Brown (GND) | GND
Red (power) | VBUS / 5V
Orange (signal) | GP16
{:class="table table-code-fit"}

---

## Moving the pen

```python
from burgerbot import Burgerbot
from time import sleep

bot = Burgerbot()

bot.pen_up()
sleep(1)
bot.pen_down()
sleep(1)
bot.pen_middle()
```

If the pen doesn't reach the paper, or lifts too far, loosen the cog, turn it a few teeth and refit it.

---

## Drawing a square

```python
from burgerbot import Burgerbot
from time import sleep

bot = Burgerbot()
bot.speed = 0.3

bot.pen_down()
sleep(0.5)

for side in range(4):
    bot.forward(1.5)
    bot.stop()
    bot.turn_right(0.8)    # adjust until your corners are 90 degrees
    bot.stop()

bot.pen_up()
```

- `pen_down()` lowers the pen before we start moving, and `sleep(0.5)` gives the servo time to get there
- Each side is a forward movement followed by a turn
- `pen_up()` lifts the pen at the end so the robot doesn't leave a trail when you pick it up

---

## Try it yourself

1. Draw a triangle. How long does a 120-degree turn take?
2. Draw a dashed line by lifting and lowering the pen as you drive
3. Write a `polygon(sides)` function that draws any regular shape

{% quiz %}
? Why do we `sleep()` after `pen_down()` before driving?
- [ ] To let the ink flow
- [x] To give the servo time to move the pen onto the paper
- [ ] Because `forward()` won't work straight after a servo command
> Servos take a moment to move. Driving straight away would leave a gap at the start of the line.
{% endquiz %}

---
