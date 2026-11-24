---
title: Project - Reaction Timer
description: Build a reaction timer game from blocks in Snakie with an LED, a button and a buzzer, then read every line of its Python
layout: lesson
type: page
cover: /learn/snakie_blocks/assets/banners/11_project_reaction_timer.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to build a reaction timer game from blocks and read every line of the Python it writes.

Time to put it all together. You're going to build a game: an LED lights up after a random wait, and you hit the button as fast as you can. The Pico times you in milliseconds and beeps when you beat your best.

---

## What you'll use

Everything in this project has appeared earlier in the course:

| Idea | Lesson |
|---|---|
| Variables and maths | Variables, Maths and Loops |
| if and comparisons | Making Decisions |
| LED and button blocks | LEDs and Buttons |
| Buzzer blocks | PWM, Servos and Buzzers |
{:class="table table-single"}

There are two new blocks, both to do with time. You'll meet them in step 3.

## The circuit

You can build the program and check it runs on the simulator, but to play you need a real button to press. Wire up a Pico like this:

| Part | Connect to |
|---|---|
| LED long leg, through a 330Ω resistor | GP15 |
| LED short leg | GND |
| Button, one side | GP14 |
| Button, other side | GND |
| Buzzer + | GP16 |
| Buzzer - | GND |
{:class="table table-single"}

## Step 1: get ready

Start a new file in the Blocks workspace and open the split view.

1. Create three variables: `best`, `start` and `reaction`.
2. Drag out **set best to** and type `9999`. That's a time anyone can beat.
3. Add a **forever** block underneath. Everything else goes inside it.
4. Inside, add a **print** block that says `Get ready...`
5. Add **turn LED on GP15 off**.

## Step 2: the random wait

If the wait were always the same, you'd learn the rhythm and cheat.

1. Add a **wait 1 seconds** block.
2. From **Maths**, drop **random integer from 2 to 5** into its socket.
3. Add **turn LED on GP15 on**. Go!

## Step 3: time the press

1. Add **set start to**, and drop in **millisecond ticks** from the **Wait** drawer. This is a stopwatch reading: the number of milliseconds since the Pico switched on.
2. Add a **repeat while** block from **Control**.
3. For its question, build **read pin GP14 with pull-up = 1**. Remember, 1 means *not* pressed. So this loop spins for as long as you haven't pressed the button.
4. Inside the loop, add **wait 1 milliseconds**.
5. After the loop, add **set reaction to**, and drop in **ticks from start to millisecond ticks**. That's the stopwatch now, minus the stopwatch then.

## Step 4: show the score

1. Add a **print** block with three parts: `Your time:`, the `reaction` variable, and `ms`.
2. Add an **if** block with the question `reaction < best`.
3. Inside the if: **set best to reaction**, print `New best!`, and **buzzer on GP16 play 880 Hz for 150 ms**.
4. Last, outside the if but inside forever, add **wait 2 seconds** before the next round.

Your finished stack should look like this:

![The finished reaction timer as blocks](/learn/snakie_blocks/assets/blocks_reaction.jpg){:class="img-fluid w-100"}

## Play it

Connect to your Pico, press **Run** and watch the console. When the LED lights, hit the button.

> **On the simulator**: the program runs, but there's no button to press. A simulated input pin always reads 0, so every round scores `Your time: 0 ms`. That's a handy way to check your blocks are in the right order before you wire anything up.

Here's the complete program, with its blocks.

{% capture snakie_code %}
import random
import time
from machine import PWM, Pin
from snakie import Buzzer, Led
led_15 = Led(pin=Pin(15, Pin.OUT))
pin_14 = Pin(14, Pin.IN, Pin.PULL_UP)
buzzer_16 = Buzzer(PWM(Pin(16)))
best = 9999
while True:
    print('Get ready...')
    led_15.set(False)
    time.sleep(random.randint(2, 5))
    led_15.set(True)
    start = time.ticks_ms()
    while pin_14.value() == 1:
        time.sleep_ms(1)
    reaction = time.ticks_diff(time.ticks_ms(), start)
    print('Your time:', reaction, 'ms')
    if reaction < best:
        best = reaction
        print('New best!')
        buzzer_16.tone(880, 150)
    time.sleep(2)
{% endcapture %}
{% include snakie.html code=snakie_code name="reaction.py" view="split" height="tall" %}

## Read every line

You built this from blocks. Now read it as Python, top to bottom.

**Imports and setup**

- The `import` lines at the top bring in the tools: random numbers, time, pins and Snakie's helpers
- The next three are the setup lines, one for each part, in the order your blocks first used them
- `best = 9999` is your first block

**The round**

- `while True:` is forever
- `time.sleep(random.randint(2, 5))` is one block inside another, and in Python it's one call inside another. The inside one runs first
- `start = time.ticks_ms()` takes the first stopwatch reading

**Waiting for you**

```python
    while pin_14.value() == 1:
        time.sleep_ms(1)
```

This is a loop that does almost nothing, on purpose. It keeps checking the button, a thousand times a second, and falls through the moment the pin reads 0.

**The score**

- `time.ticks_diff(time.ticks_ms(), start)` is the time now minus the time at the start. MicroPython has a special function for this because the tick counter eventually wraps round to zero, and `ticks_diff` copes with that
- `if reaction < best:` only runs its three lines when you've beaten your record

If you can follow that, you can read Python. You didn't sit through a syntax lesson to get here. You built things.

## Put it on your Pico

To make the game start whenever the Pico is powered, save it to the board as `main.py`. [Files, Local and Device](/learn/snakie_getting_started/09_files_local_and_device) in the Getting Started course shows you how.

---

> **Teachers**: Plan a full session. The program runs on the simulator, but with no button to press every round scores 0 ms, so each learner or group needs a Pico, a breadboard, an LED with a 330 ohm resistor, a push button, a piezo buzzer and jumper wires to actually play. Learners can check their block order on the simulator first.

## Try it Yourself

1. Change the random wait to between 1 and 8 seconds.
2. Play a low, sad tone when you *don't* beat your best. You'll need an **else**.
3. Count the rounds in a new variable and print `Round 1`, `Round 2` and so on.
4. Challenge: catch cheats. If the button is already held down when the LED comes on, print `Too soon!` and don't count the round.
5. Challenge: do one of these by typing in the Python pane instead of dragging blocks.

## Common Issues

- **Problem**: It says `Your time: 0 ms` straight away.
- **Solution**: On the simulator that's expected. On a real Pico, check the while loop's question compares with `1`, and that the pin is set **with pull-up**.
- **Why**: Without the pull-up the pin can read 0 with nothing pressed, so the loop ends at once.

- **Problem**: The LED never goes off between rounds.
- **Solution**: Make sure **turn LED off** is inside forever, near the top.
- **Why**: It has to run at the start of every round, not once.

- **Problem**: `New best!` prints every single round.
- **Solution**: Check **set best to reaction** is inside the if block.
- **Why**: If `best` never changes from 9999, every time beats it.

- **Problem**: On a real board the times look too good to be true.
- **Solution**: Check the button is wired between GP14 and GND, not 3.3V.
- **Why**: A button wired to 3.3V never pulls the pin to 0, so a loose wire does the "pressing".

## Check your understanding

{% quiz %}
? Why is the wait before the LED lights random?
- [ ] To save power
- [x] So you can't learn the timing and press early
- [ ] Because the Pico's clock is unreliable
> A fixed wait would let you predict the light instead of reacting to it.

? What does the loop `while pin_14.value() == 1:` do?
- [ ] Waits while the button is pressed
- [x] Waits until the button is pressed
- [ ] Flashes the LED
> With a pull-up, 1 means not pressed. The loop keeps going until that changes.

? What does `time.ticks_diff(time.ticks_ms(), start)` work out?
- [ ] The time of day
- [x] How many milliseconds have passed since `start`
- [ ] The sum of two times
> It subtracts the earlier stopwatch reading from the current one.
{% endquiz %}
