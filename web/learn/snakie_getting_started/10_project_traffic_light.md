---
layout: lesson
title: Project - Traffic Light
author: Kevin McAleer
type: page
cover: /learn/snakie_getting_started/assets/banners/10_project_traffic_light.jpg
date: 2026-11-20
previous: 09_files_local_and_device.html
next: 11_summary.html
description: Build a MicroPython traffic light in Snakie, first on the simulator and
  then on a real Raspberry Pi Pico with three LEDs
percent: 88
duration: 6
quiz_total: 20
date_updated: 2026-11-20
navigation:
- name: Getting Started with Snakie
- content:
  - section: Overview
    content:
    - name: Getting Started with Snakie
      link: 00_intro.html
    - name: Web App or Desktop App
      link: 01_web_or_desktop.html
    - name: A Tour of the Workspaces
      link: 02_tour_of_the_workspaces.html
  - section: Your First Programs
    content:
    - name: Your First Program on the Simulator
      link: 03_first_program_on_the_simulator.html
    - name: The REPL and the Console
      link: 04_the_repl_and_console.html
    - name: Seeing Your Pins
      link: 05_seeing_your_pins.html
  - section: Real Hardware
    content:
    - name: Finding Your Board
      link: 06_finding_your_board.html
    - name: Flashing MicroPython
      link: 07_flashing_micropython.html
    - name: Connect, Run and Stop
      link: 08_connect_run_stop.html
    - name: Files on Your Computer and on the Board
      link: 09_files_local_and_device.html
  - section: Project
    content:
    - name: Project - Traffic Light
      link: 10_project_traffic_light.html
    - name: Summary
      link: 11_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

Let's put everything together and build something: a traffic light that runs the proper UK sequence. You'll write it on the simulator, watch the pins in the Board view, then wire three LEDs to a real Pico and make it run by itself.

---

## What you'll use

- New file, Run and Stop
- The Board view, to check your pins
- The REPL, to test each LED
- The Files panel, to save the program as `main.py`

## Step 1: Plan the pins

We'll use three pins that sit together at the bottom left corner of the Pico, with a ground pin right between them.

| LED | GP number | Pad |
|---|---|---|
| Green | GP13 | 17 |
| Ground | GND | 18 |
| Amber | GP14 | 19 |
| Red | GP15 | 20 |
{:class="table table-single"}

## Step 2: Write it on the simulator

Make sure you're connected to the **Simulated device (offline)**. Start a new file and type this first version:

{% capture snakie_code %}
from machine import Pin
import time

red = Pin(15, Pin.OUT)
amber = Pin(14, Pin.OUT)
green = Pin(13, Pin.OUT)

while True:
    red.on()
    print("Stop")
    time.sleep(2)
    red.off()

    green.on()
    print("Go")
    time.sleep(2)
    green.off()

    amber.on()
    print("Get ready to stop")
    time.sleep(1)
    amber.off()
{% endcapture %}
{% include snakie.html code=snakie_code name="traffic_light.py" %}

Press **Run**. The console prints `Stop`, `Go`, `Get ready to stop`, round and round. Turn on **Show pin labels** in the Board view and check that `red`, `amber` and `green` sit on GP15, GP14 and GP13.

![The traffic light running in Snakie](/learn/snakie_getting_started/assets/snakie_code_workspace.jpg){:class="img-fluid w-100"}

Press **Stop** when you've seen enough.

## Step 3: Make it a real sequence

A UK traffic light has four stages, and one of them shows two lights at once: red, then red and amber together, then green, then amber alone.

The first version repeats itself a lot. Let's write a small function that sets all three lights in one go, and a list that describes the sequence.

{% capture snakie_code %}
from machine import Pin
import time

red = Pin(15, Pin.OUT)
amber = Pin(14, Pin.OUT)
green = Pin(13, Pin.OUT)


def show(r, a, g):
    """Set all three lights at once. 1 is on, 0 is off."""
    red.value(r)
    amber.value(a)
    green.value(g)


# Each stage: a name, the three lights, and how long it lasts
SEQUENCE = [
    ("Stop",          1, 0, 0, 3),
    ("Get ready",     1, 1, 0, 1),
    ("Go",            0, 0, 1, 3),
    ("Prepare to stop", 0, 1, 0, 1),
]

while True:
    for name, r, a, g, seconds in SEQUENCE:
        show(r, a, g)
        print(name)
        time.sleep(seconds)
{% endcapture %}
{% include snakie.html code=snakie_code name="traffic_light.py" %}

Here's what changed:

- `show(r, a, g)` takes three numbers and sets the three pins. `red.value(1)` is the same as `red.on()`
- `SEQUENCE` is a list of stages. Each stage is a group of five things: a name, three light values and a time
- The `for` loop unpacks each stage into five variables and acts on them

Want to change the timing, or add a stage? Edit the list. The code that does the work stays the same. That's a pattern you'll use in every robot you build.

## Step 4: Wire it up

Unplug the Pico while you wire.

1. Push the Pico into the breadboard so it straddles the middle gap.
2. For each LED, put the **long leg** in the row of its GP pin and the **short leg** in a spare row.
3. Connect a 330 ohm resistor from each LED's short leg to the ground rail.
4. Connect the ground rail to pad 18, the **GND** pin.

The long leg is the positive side. If an LED doesn't light later, it's probably in backwards; turn it round.

The resistor limits the current so the LED and the Pico pin are both protected. Never connect an LED straight across a pin and ground.

## Step 5: Test each LED from the REPL

Plug the Pico in, **Disconnect** from the simulator, and connect to your board. Before running the whole program, test one light at a time in the console:

```python
>>> from machine import Pin
>>> red = Pin(15, Pin.OUT)
>>> red.on()
>>> red.off()
```

Do the same for GP14 and GP13. If one doesn't light, you've found the problem while it's still easy to fix.

## Step 6: Run it, then make it stay

Press **Run**. Your three LEDs run the sequence.

Now make it permanent:

1. Click the down arrow in the Files panel.
2. Enter `/main.py` as the device path and click **OK**.
3. Press **Reset**.

The traffic light starts by itself. Unplug the Pico from your computer, plug it into a USB power bank, and it carries on. You've built a standalone MicroPython device.

---

## Try it Yourself

1. Change the timings so the green light lasts five seconds.
2. Add a stage where all three lights flash on together.
3. Challenge: add a push button on GP16 for a pedestrian crossing. Read it with `Pin(16, Pin.IN, Pin.PULL_UP)`; its `value()` is `0` while the button is pressed. When it's pressed during the green stage, cut the green short.

## Common Issues

**Problem**: One LED never lights.
**Solution**: Turn the LED round, and check its resistor reaches the ground rail.
**Why**: LEDs only pass current one way.

**Problem**: The lights come on in the wrong order.
**Solution**: Compare your wiring with the Board view. Red belongs on GP15, amber on GP14, green on GP13.
**Why**: Two wires one row out is all it takes.

**Problem**: It works from Run but not from a power bank.
**Solution**: Check `main.py` is on the board with **Refresh** in Device files.
**Why**: Run doesn't save the program to the board.

## Check your understanding

{% quiz %}
? What is the resistor for?
- [ ] To make the LED brighter
- [x] To limit the current through the LED and the pin
- [ ] To store the program
> Without a resistor, too much current flows and can damage the LED or the Pico.

? In the second version, how would you add a new stage to the sequence?
- [ ] Write another `while` loop
- [x] Add another entry to the `SEQUENCE` list
- [ ] Add another pin
> The list describes the stages. The loop works through whatever is in it.
{% endquiz %}
