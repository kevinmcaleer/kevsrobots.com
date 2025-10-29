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
duration: 4
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

To prevent collisions with obstacles, use `bot.distance` to measure the distance between the object and your bot. If the measured distance is less than a specified threshold (e.g., 5 cm), stop the bot and move it backward to maintain a safe distance from the obstacle.

### Understanding the Distance Sensor

The ultrasonic rangefinder works by sending out sound waves and measuring how long they take to bounce back. The `bot.distance` property returns the distance in centimeters to the nearest object.

**The code:**

```python
from burgerbot import Burgerbot

bot = Burgerbot()

while True:  # Infinite loop - runs forever (note: capital T in True!)
    distance = bot.distance  # Get current distance in cm

    if distance <= 5:  # Object closer than 5cm
        print(f"Too close! Distance: {distance}cm")
        bot.stop()
        bot.backward(1)  # Back up for 1 second
    else:
        print(f"Path clear. Distance: {distance}cm")
        bot.forward(0.5)  # Move forward in short bursts
```

**How it works:**

1. `while True:` creates an infinite loop that runs continuously
2. `bot.distance` gets the current distance reading from the rangefinder
3. If distance is 5cm or less, the robot stops and reverses
4. If distance is greater than 5cm, the robot moves forward
5. Using `forward(0.5)` instead of `forward(1)` makes the robot check distance more frequently

**Why short bursts?** Moving in 0.5-second intervals allows the robot to respond more quickly to obstacles, making navigation smoother and safer.

---

## Common Issues

- **Problem**: `bot.distance` returns 0 or very large numbers (like 400)
- **Solution**: Check that the rangefinder Trigger and Echo pins are wired correctly to GPIO pins
- **Why**: Incorrect wiring prevents the sensor from sending or receiving ultrasonic signals

- **Problem**: Robot doesn't detect objects closer than 2cm
- **Solution**: This is normal - ultrasonic sensors have a minimum detection range of about 2cm
- **Why**: Sound waves need time and space to separate transmission from echo

- **Problem**: Robot doesn't detect dark or soft objects well
- **Solution**: Use objects with hard, flat surfaces for testing. Ultrasonic sensors work best with solid, reflective surfaces
- **Why**: Soft materials (like fabric) and dark surfaces absorb sound waves instead of reflecting them

- **Problem**: Robot behaves erratically, sometimes ignoring obstacles
- **Solution**: Ensure you're using `while True:` with a capital T. Python is case-sensitive!
- **Why**: `while true:` will cause a NameError because `true` is not defined in Python (it's `True`)

- **Problem**: Robot reverses but doesn't turn to find a new path
- **Solution**: This basic algorithm only reverses. You'll need to add turning logic (covered in later lessons)
- **Why**: The current code is designed to demonstrate simple avoidance, not full navigation

---

## Try it Yourself

Ready to experiment with object avoidance? Try these challenges:

1. **Adjust the detection threshold**: Change `if distance <= 5:` to `if distance <= 10:`. How does the robot's behavior change? What about `distance <= 3`?

2. **Add turning**: Modify the code to turn left or right after reversing. Hint: Add `bot.left(0.5)` after the backward command.

3. **Variable speed**: Make the robot slow down as it approaches objects. Use the distance value to calculate motor speed.

4. **Obstacle counter**: Keep track of how many objects the robot has avoided. Print this count to the console.

**Advanced Challenge**: Create a robot that maintains exactly 10cm from a wall as it drives parallel to it. You'll need to constantly adjust direction based on distance readings!

---

## Following an object

To maintain a consistent distance from an object, first measure the distance between the object and your position. Move forward until the object is at the desired distance, then stop. If the object is closer than the desired distance, move backward until the desired distance is achieved. This process allows you to effectively follow an object while maintaining a set distance from it.

---

## Example 

<script src="https://gist.github.com/kevinmcaleer/750ca53e653f70aee0138abaa767f9fb.js"></script>

---
