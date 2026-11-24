---
layout: lesson
title: Speak with the Buzzer
author: Kevin McAleer
type: page
cover: /learn/robot_senses/assets/banners/07_speak_the_buzzer.jpg
date: 2026-11-24
previous: 06_feel_movement.html
next: 08_show_the_led_matrix.html
description: Play tones and tunes on the Modulino Buzzer and learn why blocking calls
  make a moving robot drive blind
percent: 40
duration: 4
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

Sense three is voice. The **Modulino Buzzer** lets the robot make sounds: a warning beep, a startup jingle, or a sad little "uh oh" when you pick it up. Sound also makes debugging easier. If a robot can say "I saw something", you don't need a cable attached to know it.

## What it does

The module is a piezo buzzer with a small processor onboard. You give it a frequency in hertz and a length in milliseconds, and it plays a tone. A higher number is a higher note. For reference, 440 Hz is the note A4.

## Real-world robot example

This is a complete program that plays a note and stops.

```python
from time import sleep_ms
from machine import I2C, Pin
from modulino import ModulinoBuzzer

i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400_000)
buzzer = ModulinoBuzzer(i2c_bus=i2c)

buzzer.tone(440, 500, blocking=True)    # A4 for half a second
buzzer.no_tone()                         # make sure it is silent
sleep_ms(300)
buzzer.tone(880, 150, blocking=True)    # a short, higher beep
buzzer.no_tone()
```
{: .no-snakie }

`tone(frequency, duration_ms, blocking=...)` plays the note. `no_tone()` stops it.

## Blocking or not?

The `blocking` argument decides what your program does while the note plays.

- `blocking=True` waits for the note to finish before moving to the next line.
- Leave it off, and your code carries on while the note plays.

On a bench that's a small detail. On a moving robot it's huge. A half-second blocking beep means half a second of driving blind. At cruising speed, that's far enough to hit the thing the robot was about to see.

Kev's helper code makes quick beeps non-blocking by default, and only waits when the robot is standing still, such as during the startup tune.

## Tunes with note names

The library has a table of named notes, `ModulinoBuzzer.NOTES`, and Kev's repository uses names such as `"C4"` and `"G5"` with it. Here is the startup jingle from his robot, as a complete program.

```python
from time import sleep_ms
from machine import I2C, Pin
from modulino import ModulinoBuzzer

i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400_000)
buzzer = ModulinoBuzzer(i2c_bus=i2c)

NOTES = ModulinoBuzzer.NOTES

STARTUP = (("C4", 120), ("E4", 120), ("G4", 120), ("C5", 260))
UH_OH = (("G4", 160), ("E4", 160), ("C4", 320))

def play(melody, gap_ms=20):
    """Play (note_name, duration_ms) pairs. Use None for a rest."""
    for name, ms in melody:
        if name is None:
            sleep_ms(ms)
            continue
        buzzer.tone(NOTES[name], ms, blocking=True)
        sleep_ms(gap_ms)
    buzzer.no_tone()

play(STARTUP)
sleep_ms(500)
play(UH_OH)
```
{: .no-snakie }

A melody is just a tuple of pairs, so a new tune is a new tuple, not new code. Each pair is a note name and how long to hold it.

**Why this matters:** a short rising tune says "ready" and a falling one says "uh oh". Robots that communicate this way are far easier to live with.

**Common mistakes:**

- Skipping `no_tone()`. Kev's code always calls it after a tune, to be sure the buzzer is quiet.
- Using a long blocking tune inside a driving loop.
- Asking for a note name that isn't in the table, which fails with an error.

## Try it Yourself

1. Write your own four-note jingle and play it.
2. Put a `None` rest between two notes. How does it change the tune?
3. Challenge: write a melody that rises when the robot is happy and falls when it is worried. Lesson 12 uses both.

## Common Issues

**Problem**: The buzzer stays on after your program ends.
**Solution**: Call `buzzer.no_tone()` at the end, and in a `finally` block if your program has a loop.
**Why**: Kev's code always ends a tune with `no_tone()` so the buzzer is certain to be silent. Do the same.

**Problem**: The robot hesitates every time it beeps.
**Solution**: Use `blocking=False` inside the control loop.
**Why**: Blocking calls hold up the loop, so the robot isn't sensing while the note plays.

**Problem**: An error about a missing note name when playing a tune.
**Solution**: Print `NOTES` and check the spelling of the note name.
**Why**: The table only holds the names the library defines.
