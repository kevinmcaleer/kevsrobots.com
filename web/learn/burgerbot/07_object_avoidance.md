---
layout: lesson
title: Object Avoidance
author: Kevin McAleer
type: page
cover: /learn/burgerbot/assets/burgerbot.jpg
date: 2023-04-27
previous: 06_measuring_distance.html
next: 08_video.html
description: Learn how to avoid objects with BurgerBot.
percent: 70
duration: 1
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


## Avoiding an object

To prevent collisions with obstacles, use bot.distance to measure the distance between the object and your bot. If the measured distance is less than a specified threshold (e.g., 5 cm), stop the bot and move it backward to maintain a safe distance from the obstacle.

```python
bot = Burgerbot()

while true:
    if bot.distance <= 5:
        bot.stop()
        bot.backward(1) 
    else:
        bot.forward(1)
```

---

## Following an object

To maintain a consistent distance from an object, first measure the distance between the object and your position. Move forward until the object is at the desired distance, then stop. If the object is closer than the desired distance, move backward until the desired distance is achieved. This process allows you to effectively follow an object while maintaining a set distance from it.

---

## Example 

<script src="https://gist.github.com/kevinmcaleer/750ca53e653f70aee0138abaa767f9fb.js"></script>

---
