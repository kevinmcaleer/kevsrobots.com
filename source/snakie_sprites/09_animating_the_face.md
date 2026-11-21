---
title: Animating the Face
description: Write an animation loop that makes the eyes blink and glance around at random so the face feels alive
layout: lesson
type: page
cover: /learn/snakie_sprites/assets/banners/09_animating_the_face.jpg
date_updated: 2026-11-21
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

## Animation is a loop

Every animation, from a blinking eye to a video game, is the same four steps on repeat:

1. Clear the buffer
2. Draw the new picture
3. Show it
4. Wait a moment

Your `face` function already does the first three. Add a wait and a list of frames and you have a blink.

---

## A blink

Start a new file, `blink.py`:

```python
import time
from simscreen import SimScreen
import eye

OPEN, HALF, CLOSED, LEFT, RIGHT = 0, 1, 2, 3, 4

screen = SimScreen()


def face(i):
    screen.fill(0)
    screen.blit(eye.frame(i), 40, 24)
    screen.blit(eye.frame(i), 72, 24)
    screen.show()


def blink():
    for i in [HALF, CLOSED, HALF, OPEN]:
        face(i)
        time.sleep(0.06)


face(OPEN)
time.sleep(1)
blink()
```

Run it. The eyes open, wait a second and blink.

The blink is a **sequence**: half, closed, half, open. Three of your five frames, in an order the Sprite editor never played them in. That's the point of treating frames as ingredients.

---

## A glance

Looking sideways is a sequence too, with a pause in the middle:

```python
def glance(direction):
    face(direction)
    time.sleep(0.5)
    face(OPEN)
```

Add it under `blink`, then try `glance(LEFT)` at the bottom of the file.

---

## Why fixed loops look robotic

Replace the last three lines with this and run it:

```python
while True:
    time.sleep(1)
    blink()
```

It blinks exactly once a second, forever. Watch it for a while. It looks like a machine, and not in a charming way. Press **Stop** when you've seen enough.

Living things aren't regular. People blink every two to ten seconds, and their eyes wander. A little randomness fixes it.

---

## Add some life

Here is the whole program with a random loop. Change `blink.py` so it reads:

```python
import time
import random
from simscreen import SimScreen
import eye

OPEN, HALF, CLOSED, LEFT, RIGHT = 0, 1, 2, 3, 4

screen = SimScreen()


def face(i):
    screen.fill(0)
    screen.blit(eye.frame(i), 40, 24)
    screen.blit(eye.frame(i), 72, 24)
    screen.show()


def blink():
    for i in [HALF, CLOSED, HALF, OPEN]:
        face(i)
        time.sleep(0.06)


def glance(direction):
    face(direction)
    time.sleep(0.5)
    face(OPEN)


face(OPEN)
while True:
    time.sleep(random.uniform(0.4, 1.2))
    choice = random.randint(0, 3)
    if choice == 0:
        glance(LEFT)
    elif choice == 1:
        glance(RIGHT)
    else:
        blink()
```

![The eyes glancing to one side](/learn/snakie_sprites/assets/blink.jpg){:class="img-fluid w-100"}

Two lines do the work:

- `random.uniform(0.4, 1.2)` picks a wait somewhere between 0.4 and 1.2 seconds, different every time
- `random.randint(0, 3)` picks 0, 1, 2 or 3. One in four is a glance left, one in four a glance right, and the other two are blinks

The waits here are short so you can see plenty happening. For a calmer robot, try `random.uniform(2, 6)`.

---

## Design the behaviour in text first

You can try out the "what happens next" part without a screen. This box runs the same decision loop and prints what the face would do:

{% capture snakie_code %}import random

for step in range(12):
    wait = random.uniform(0.4, 1.2)
    choice = random.randint(0, 3)
    if choice == 0:
        action = "glance left"
    elif choice == 1:
        action = "glance right"
    else:
        action = "blink"
    print("wait", round(wait, 1), "s, then", action)
{% endcapture %}
{% include snakie.html code=snakie_code name="behaviour.py" view="code" %}

Run it a few times. Then change the odds: make glances rare by using `random.randint(0, 9)`.

---

## Try it Yourself

1. Add a `double_blink()` that blinks twice quickly, and give it a one-in-ten chance
2. Make a `sleepy()` state: show `HALF` for two seconds, then `CLOSED` for three
3. Make the robot look left, then right, then blink, as if it's crossing the road
4. Slow the blink down to 0.2 seconds per frame. At what speed does it stop looking like a blink?

---

## Common Issues

**Problem**: The program never ends and I can't run anything else.
**Solution**: Press **Stop** in the toolbar.
**Why**: `while True:` runs until you stop it. That's what you want on a finished robot, and what Stop is for while you're building.

**Problem**: The blink is so fast I can't see it.
**Solution**: Increase the `time.sleep` inside `blink`.
**Why**: Each frame needs to be on screen long enough for your eye to register it. Below about 0.03 seconds frames start to be missed.

**Problem**: The face always does the same thing in the same order.
**Solution**: Check you're calling `random.randint` **inside** the loop, not once above it.
**Why**: A choice made before the loop is made only once.

---

## Check your understanding

{% quiz %}
? What are the four steps of an animation loop?
- [ ] Import, define, call, return
- [x] Clear, draw, show, wait
- [ ] Open, read, write, close
> Clear the buffer, draw the new picture, send it to the screen, then pause.

? Why does the blink sequence end with OPEN?
- [ ] Because OPEN is frame 0
- [x] So the eyes are left open when the blink is over
- [ ] It makes the blink faster
> A sequence should finish in the state you want to rest in.

? What makes the face feel alive rather than mechanical?
- [ ] A higher frame rate
- [ ] More pixels
- [x] Random timing and a random choice of actions
> Regular repetition looks like a machine. Small, unpredictable variations look like life.
{% endquiz %}
