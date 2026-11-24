---
title: Wall Following
description: Learn the left-hand rule for escaping a maze, turn it into code with distance thresholds and proportional control, and see it on the Arduino Alvik
layout: lesson
type: page
cover: /learn/robot_senses/assets/banners/13_wall_following.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

So far the robot reacts to what's right in front of it. Now let's give it a goal: get through a maze. The classic first answer is **wall following**, and it needs no map and no planning, just a distance sensor and a few rules. Kev covers it in his Alvik maze video, which is a good watch before you read on.

{% include youtubeplayer.html id="vHUXDgSiWio" %}

## The left-hand rule

Put your left hand on the wall and walk. Never take it off. Follow the wall around every corner, and you'll eventually find the exit. That's the whole algorithm.

There's one catch, and it's an important one. This works for any **simply-connected** maze, meaning one without loops or islands. If the exit is on a wall that's disconnected from the wall you're touching, you'll walk around forever. Keep that in mind, because it's exactly why we move on to mapping in the next lesson.

## What the robot needs

To follow a wall on its left, the robot needs to know three things: how far the wall is on its left, how far the nearest thing is in front, and, optionally, how far the wall is on its right. Modulino SMARS has a single distance sensor pointing forward, so it can't follow a wall on its own. It could bump along a room, but it can't hold a steady distance from a side wall.

You might think "add a second Distance module". But the Modulino Distance has its address fixed in the chip, so two of them can't share one Qwiic chain. You'd need another approach, and the sources for this course don't cover one, so we won't invent it.

Kev's maze project uses the **Arduino Alvik**, a robot that has distance sensors on its left, centre and right. That is the robot we use for the rest of the course.

## The rules as code

Here are the left-hand rules as a plain MicroPython function with made-up readings. It runs anywhere. The numbers are Kev's thresholds from the Alvik project, in centimetres.

```python
WALL_DISTANCE = 15       # the distance we want to keep from the wall
TOO_CLOSE = 10           # closer than this: drift away
TOO_FAR = 20             # farther than this: the wall has gone, find it
FRONT_OBSTACLE = 20      # something is ahead

def choose_move(left, front):
    """The left-hand rule. Returns what the robot should do."""
    if front < FRONT_OBSTACLE:
        return "wall ahead: turn right"
    if left > TOO_FAR:
        return "no wall on the left: turn left to find it"
    if left < TOO_CLOSE:
        return "too close to the left wall: drift right"
    return "perfect distance: go straight"

# Made-up (left, front) readings as the robot drives round a corner
for left, front in ((15, 80), (8, 80), (14, 18), (40, 60), (16, 70)):
    print((left, front), "->", choose_move(left, front))
```

The order matters. A wall ahead beats everything else, because hitting it is the worst outcome. A missing wall on the left comes next, because without it the rule has nothing to follow.

## The Alvik version

This is the program from Kev's maze post, for the Arduino Alvik. It needs the Alvik's own library, `arduino_alvik`, which is not on stock MicroPython firmware. Arduino provides it for the Alvik, and the Alvik documentation at [docs.arduino.cc](https://docs.arduino.cc/) covers installing it.

```python
from arduino_alvik import ArduinoAlvik
import time

alvik = ArduinoAlvik()
alvik.begin()

# Distance thresholds (in cm)
WALL_DISTANCE = 15
TOO_CLOSE = 10
TOO_FAR = 20
FRONT_OBSTACLE = 20

def wall_follow_left():
    while True:
        left = alvik.get_distance_left()
        front = alvik.get_distance_center()

        if front < FRONT_OBSTACLE:
            alvik.rotate(90)            # wall ahead: turn right
        elif left > TOO_FAR:
            alvik.rotate(-90)           # no wall on the left: turn left
            alvik.drive(10, 0)          # move forward a bit
        elif left < TOO_CLOSE:
            alvik.drive(10, 30)         # too close: drift right
        else:
            alvik.drive(10, 0)          # perfect: go straight

        time.sleep(0.1)

try:
    wall_follow_left()
except KeyboardInterrupt:
    alvik.stop()
    print("Stopped by user")
```
{: .no-snakie }

The structure is the same as the decision function above: read, decide, act. The `if` chain is `choose_move()` with motor commands in place of text. In this code, `rotate(90)` turns right and `rotate(-90)` turns left, and a positive turn value in `drive()` drifts right. Check the signs on your own Alvik before you let it loose.

## Smoother: proportional control

That first version works, but it moves in jerks: straight, drift, straight, drift. A smoother way is **proportional control**. Instead of fixed turns, the robot turns by an amount proportional to how far it is from the ideal distance. A little off means a little turn. A lot off means a bigger turn.

```python
WALL_DISTANCE = 15
KP = 2.0                # proportional gain: how strongly to correct

def turn_rate(left):
    # How far are we from the distance we want?
    error = WALL_DISTANCE - left
    # The correction is the error multiplied by the gain
    rate = error * KP
    # Limit it so we never turn violently
    return max(-45, min(45, rate))

for left in (15, 12, 8, 18, 25, 60):
    print("left =", left, "cm -> turn", turn_rate(left))
```

Read it line by line. If `left` is 15, the error is 0, so there's no turn. If it's 8, the error is 7, and 7 times 2 gives a turn of 14. If it's 60, the error is huge, but `max(-45, min(45, ...))` clamps it to 45. `KP` is the dial you tune. Too low and the robot drifts into the wall. Too high and it zig-zags.

On the Alvik, you'd pass this to `alvik.drive(BASE_SPEED, turn_rate)` every 50 ms, and stop and rotate if the front distance drops below `FRONT_OBSTACLE`. That is how Kev's enhanced version works.

**Common mistakes:**

- Checking the left wall before the front one, and driving into a corner.
- Setting `WALL_DISTANCE` wider than the maze corridor.
- Turning the gain up until it wobbles. Increase `KP` in small steps.

## Try it Yourself

1. Run `choose_move()` with `(15, 80)`, then `(15, 5)`. Which rule fires, and why?
2. Change `KP` to 4. What does `turn_rate(8)` give now?
3. Challenge: write `choose_move_right()`, the right-hand rule, by mirroring each decision.
4. If you have an Alvik: build a maze from cardboard boxes and tune `WALL_DISTANCE` to your corridor width.

## Common Issues

**Problem**: The robot gets stuck in a corner.
**Solution**: Add a timeout that backs up and turns if it stays in one spot.
**Why**: The rules have no memory. A corner can trap them in a loop.

**Problem**: The robot loops forever and never finds the exit.
**Solution**: Check your maze has no islands.
**Why**: The left-hand rule only escapes simply-connected mazes.

**Problem**: It turns the wrong way.
**Solution**: Check the sign of the turn values on your robot.
**Why**: Which direction is positive depends on the robot's own convention.

## Check your understanding

{% quiz %}
? When does the left-hand rule fail to find the exit?
- [ ] When the maze has more than three turns
- [x] When the maze has loops or islands
- [ ] When the walls are cardboard
> It only works for simply-connected mazes, where all the walls join up.

? What does proportional control do?
- [ ] Turns by a fixed amount every time
- [x] Turns by an amount that grows with the error
- [ ] Stops the robot
> The correction is the error multiplied by a gain, so small errors get small turns.

? Why can't Modulino SMARS use two Modulino Distance modules on one chain?
- [ ] The cable is too short
- [x] The address is fixed in the chip, so two would clash
- [ ] They use different voltages
> The Distance module's address is fixed at `0x29`.
{% endquiz %}
