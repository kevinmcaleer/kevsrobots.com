---
layout: lesson
title: Basic Movement
author: Kevin McAleer
type: page
cover: /learn/burgerbot/assets/burgerbot.jpg
date: 2023-04-27
previous: 04_loading_the_code.html
next: 06_measuring_distance.html
description: Learn how to move the robot forward, backward, left and right.
percent: 48
duration: 6
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


## Understanding Differential Drive

BurgerBot uses **differential drive** - a system where two wheels are controlled independently to create movement. By varying the speed and direction of each motor, you can make the robot move forward, backward, and turn.

**How it works:**
- **Forward**: Both motors spin forward at the same speed
- **Backward**: Both motors spin backward at the same speed
- **Turning on the spot**: One motor spins forward and the other spins backward
- **Gentle curve**: One motor spins faster than the other

---

## Moving the robot forward

**What it does:** Activates both motors in the forward direction to propel the robot ahead.

**The code:**

```python
from burgerbot import Burgerbot

bot = Burgerbot()

# Move forward for 2 seconds
bot.forward(2)
bot.stop()
```

**How it works:**

- `from burgerbot import Burgerbot` imports the BurgerBot library
- `bot = Burgerbot()` creates a new robot object
- `bot.forward(2)` starts both motors and waits 2 seconds
- `bot.stop()` stops both motors. Without it, the robot would keep going!

**Why this matters:** Forward movement is the foundation of robot navigation. Understanding how to control movement duration helps you create precise navigation patterns.

---

## Moving the robot backward

**What it does:** Reverses both motors to move the robot backward.

**The code:**

```python
from burgerbot import Burgerbot

bot = Burgerbot()

# Move backward for 1.5 seconds
bot.backward(1.5)
bot.stop()
```

**How it works:**

- `bot.backward(1.5)` reverses both motors for 1.5 seconds
- The duration parameter (1.5) determines how far the robot travels

**Real-world use:** Backward movement is essential for obstacle avoidance - when your robot detects an object, it needs to reverse before choosing a new direction.

---

## Turning left and right

**What it does:** Rotates the robot in place by spinning the motors in opposite directions.

**The code:**

```python
from burgerbot import Burgerbot

bot = Burgerbot()

# Turn left for 1 second
bot.turn_left(1)
bot.stop()

# Wait a moment
import time
time.sleep(0.5)

# Turn right for 1 second
bot.turn_right(1)
bot.stop()
```

**How it works:**

- `bot.turn_left(1)` spins the robot on the spot by running one motor forward and the other backward
- `bot.turn_right(1)` does the same with the motors swapped
- The duration (1 second) controls how far the robot rotates

**Tip:** Turn time depends on speed. At the default speed of 0.5, try about half a second for a 90-degree turn, but this varies based on battery charge and surface friction. Experiment to find the right timing for your robot!

---

## Changing speed

`bot.speed` sets how fast the motors run, from 0 (stopped) to 1 (full speed):

```python
from burgerbot import Burgerbot

bot = Burgerbot()

bot.speed = 0.3   # slow
bot.forward(2)
bot.speed = 1     # fast
bot.forward(1)
bot.stop()
```

You can also drive each wheel separately with `bot.left_motor()` and `bot.right_motor()`. Different speeds make the robot curve:

```python
bot.left_motor(0.6)
bot.right_motor(0.3)   # the slower right wheel makes the robot curve right
```

---

## Creating Movement Patterns

Now that you understand the basics, you can combine movements to create patterns:

```python
from burgerbot import Burgerbot
import time

bot = Burgerbot()

# Drive in a square
for i in range(4):
    bot.forward(2)    # Forward for 2 seconds
    bot.stop()
    time.sleep(0.2)   # Brief pause
    bot.turn_right(0.5)  # Turn about 90 degrees
    bot.stop()
    time.sleep(0.2)

print("Square complete!")
```

**What this does:** Creates a square pattern by repeating forward movement and 90-degree turns four times.

---

## Common Issues

- **Problem**: Robot moves backward instead of forward
- **Solution**: Swap the motor connections on the Motor SHIM. The motors may be wired in reverse
- **Why**: Motor polarity determines direction - if wired backward, "forward" becomes "backward"

- **Problem**: Robot curves to one side instead of going straight
- **Solution**: Check that both motors are the same model and wheels are the same size. You may need to adjust individual motor speeds in the library
- **Why**: Manufacturing differences between motors or uneven wheel wear causes speed imbalances

- **Problem**: Robot doesn't move at all
- **Solution**: Check battery charge, motor connections, and verify the Motor SHIM is properly seated on the Pico
- **Why**: Low voltage or loose connections prevent motors from running

- **Problem**: Turns are not 90 degrees
- **Solution**: Adjust the turn duration. More time = bigger turn. Start with `bot.turn_left(0.4)` or `bot.turn_left(0.6)` and fine-tune
- **Why**: Turn angle depends on motor speed, battery charge, and surface friction - it requires calibration

- **Problem**: Robot jerks or stutters during movement
- **Solution**: Ensure battery is fully charged. Weak batteries cause inconsistent motor performance
- **Why**: Motors need stable voltage to maintain smooth, consistent speed

---

## Try it Yourself

Congratulations! Your BurgerBot is moving! Now let's practice:

1. **Modify forward duration**: Change `bot.forward(2)` to `bot.forward(5)`. How much farther does your robot travel? Can you measure the distance?

2. **Create a triangle**: Can you make your robot drive in a triangle shape? Hint: You'll need 120-degree turns instead of 90-degree turns.

3. **Figure-8 pattern**: Create a figure-8 by combining forward movement with left and right turns. This is challenging!

4. **Obstacle course**: Set up some boxes and navigate your robot around them using only forward, backward, and turn commands.

**Challenge**: Write a function called `move_distance(centimeters)` that calculates the correct duration to move a specific distance. You'll need to measure how far your robot travels in 1 second, then use that to calculate duration.

---

{% quiz %}
? How does BurgerBot turn on the spot?
- [ ] Both motors spin forward
- [x] One motor spins forward and the other spins backward
- [ ] Both motors stop
> Spinning the wheels in opposite directions rotates the robot around its centre.

? What does `bot.speed = 0.25` do?
- [x] Sets the motors to a quarter of full speed for the next movements
- [ ] Moves the robot forward for 0.25 seconds
- [ ] Turns the robot 25 degrees
> `speed` ranges from -1 to 1. Movement methods use it until you change it.
{% endquiz %}

---

## What's Next?

Now that your robot can move in all directions, the next lesson will teach you how to measure distance using the ultrasonic rangefinder. This is the first step toward autonomous navigation!

---
