---
layout: lesson
title: PWM, Servos and Buzzers
author: Kevin McAleer
type: page
cover: /learn/snakie_blocks/assets/banners/06_pwm_servos_and_buzzers.jpg
date: 2026-11-20
previous: 05_leds_and_buttons.html
next: 07_functions.html
description: Fade an LED, play a tune on a buzzer and sweep a servo using Snakie hardware
  blocks and the for each loop
percent: 49
duration: 5
quiz_total: 36
date_updated: 2026-11-24
navigation:
- name: Snakie Blocks - From Blocks to Python
- content:
  - section: Overview
    content:
    - name: Snakie Blocks - From Blocks to Python
      link: 00_intro.html
    - name: The Blocks Workspace
      link: 01_the_blocks_workspace.html
  - section: Blocks Basics
    content:
    - name: Turtle Blocks - Draw a Square
      link: 02_turtle_draw_a_square.html
    - name: Variables, Maths and Loops
      link: 03_variables_maths_and_loops.html
    - name: Making Decisions
      link: 04_making_decisions.html
  - section: Blocks and Hardware
    content:
    - name: Hardware Blocks - LEDs and Buttons
      link: 05_leds_and_buttons.html
    - name: PWM, Servos and Buzzers
      link: 06_pwm_servos_and_buzzers.html
    - name: Functions
      link: 07_functions.html
    - name: My Parts
      link: 08_my_parts.html
  - section: From Blocks to Python
    content:
    - name: Reading the Python
      link: 09_reading_the_python.html
    - name: Simple and Advanced Blocks
      link: 10_simple_and_advanced_blocks.html
  - section: Project
    content:
    - name: Project - Reaction Timer
      link: 11_project_reaction_timer.html
    - name: Summary and Next Steps
      link: 12_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to fade an LED, play a tune and sweep a servo using PWM-based blocks.

On and off is only the start. This lesson is about everything in between: brightness, pitch and position.

---

## PWM: faking "half on"

A pin can only be fully on or fully off. But if you switch it on and off very fast, an LED looks dimmer. Spend more of the time on and it looks brighter. That trick is called **PWM**, pulse width modulation.

Two blocks control it:

- **set frequency of GP15 to 1000 Hz** sets how many times a second the pin switches
- **set power of GP15 to 50 %** sets how much of the time it's on

## Fade an LED

![The fade program as blocks](/learn/snakie_blocks/assets/blocks_fade.jpg){:class="img-fluid w-100"}

This uses a new loop, **for each item in list**. It runs the blocks inside once for every value, and puts each value in the variable `level`.

{% capture snakie_code %}
import time
from machine import PWM, Pin
pwm_15 = PWM(Pin(15))
pwm_15.freq(1000)
while True:
    for level in range(0, 101, 5):
        pwm_15.duty_u16(int(level * 65535 / 100))
        time.sleep_ms(50)
{% endcapture %}
{% include snakie.html code=snakie_code name="fade.py" view="split" %}

### Reading the Python

- `range(0, 101, 5)` counts 0, 5, 10 and so on up to 100. It stops *before* 101, which is why it isn't 100
- `duty_u16()` wants a number from 0 to 65535, not a percentage. So the block does the sum for you: `level * 65535 / 100`

That second line is a good example of why blocks are a kind way to start. The block says **50 %**. The Python shows you what the Pico really needs.

## Play a tune

A buzzer makes a note when you switch it on and off at the note's frequency. 440 Hz is the A that orchestras tune to.

![The tune program as blocks](/learn/snakie_blocks/assets/blocks_tune.jpg){:class="img-fluid w-100"}

{% capture snakie_code %}
from machine import PWM, Pin
from snakie import Buzzer
buzzer_16 = Buzzer(PWM(Pin(16)))
for _ in range(3):
    buzzer_16.tone(440, 200)
    buzzer_16.tone(660, 200)
buzzer_16.stop()
{% endcapture %}
{% include snakie.html code=snakie_code name="tune.py" view="split" %}

- `buzzer_16.tone(440, 200)` plays 440 Hz for 200 milliseconds
- `buzzer_16.stop()` is the **silence buzzer** block

The simulator has no speaker, but you can watch GP16 working in the board view. On a real Pico, connect a piezo buzzer between GP16 and GND.

Some notes to play with:

| Note | Frequency (Hz) |
|---|---|
| C | 262 |
| D | 294 |
| E | 330 |
| F | 349 |
| G | 392 |
| A | 440 |
{:class="table table-single"}

## Sweep a servo

A servo is a motor that turns to an angle you choose and holds it. Robot arms, grippers and steering all use them.

![The servo program as blocks](/learn/snakie_blocks/assets/blocks_servo.jpg){:class="img-fluid w-100"}

{% capture snakie_code %}
import time
from machine import PWM, Pin
from snakie import Servo
servo_0 = Servo(PWM(Pin(0)), pin=0)
while True:
    servo_0.angle(0)
    time.sleep(1)
    servo_0.angle(180)
    time.sleep(1)
{% endcapture %}
{% include snakie.html code=snakie_code name="servo.py" view="split" %}

In the box, the console lists each move as a line starting `SNK SERVO`. That is the simulator reporting the angle the servo has been told to go to.

Servos are driven by PWM too. The `Servo` helper turns your angle into the right pulse, so you only think in degrees.

This program uses the other wait block, **wait 1 seconds**, which writes `time.sleep(1)`.

> **Powering servos**: a small servo such as an SG90 can run from the Pico's VBUS pin for testing. Anything bigger needs its own power supply, with its ground joined to the Pico's ground.

---

> **Teachers**: The simulator has no speaker, so learners watch GP16 in the board view and read the console instead of hearing the tune. A real piezo buzzer on a Pico is needed to hear it.

## Try it Yourself

1. Make the LED fade down again after fading up. You'll need a second **for each** loop that counts backwards.
2. Play the first three notes of a scale: C, D, E.
3. Make the servo stop at 90 degrees on its way across.
4. Challenge: use a **for each** loop to sweep the servo smoothly from 0 to 180 in steps of 10.

## Common Issues

- **Problem**: The LED flickers instead of fading.
- **Solution**: Set the frequency to 1000 Hz before setting the power.
- **Why**: At a low frequency your eye can see each flash.

- **Problem**: The servo twitches or the Pico resets.
- **Solution**: Power the servo from a separate supply, and join the grounds.
- **Why**: Servos draw more current than the Pico's 3.3V pin can give.

- **Problem**: A warning badge appears on my buzzer blocks.
- **Solution**: Nothing to fix if they are meant to share a pin.
- **Why**: Snakie flags any two blocks on one pin, in case it was a mistake.

## Check your understanding

{% quiz %}
? What does PWM change to make an LED dimmer?
- [ ] The voltage
- [x] How much of the time the pin is on
- [ ] The colour
> The pin is still fully on or off. It just spends less of each cycle on.

? What does `buzzer_16.tone(440, 200)` do?
- [ ] Plays 200 Hz for 440 ms
- [x] Plays 440 Hz for 200 ms
- [ ] Plays 440 notes
> Frequency first, then how long in milliseconds.

? Why does `range(0, 101, 5)` end at 101?
- [ ] It is a typo
- [x] range stops before its end value, so 101 lets it reach 100
- [ ] The LED needs 101 per cent
> `range()` never includes its stop value.
{% endquiz %}
