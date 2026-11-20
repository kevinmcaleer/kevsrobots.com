---
layout: lesson
title: Drivers, Help and Code
author: Kevin McAleer
type: page
cover: /learn/snakie_electronics/assets/banners/07_drivers_help_and_code.jpg
date: 2026-11-20
previous: 06_checking_your_wiring.html
next: 08_connections_and_measurements.html
description: Install a part's MicroPython driver from Snakie's Electronics workspace,
  read its built-in help page, and write code that matches your wiring
percent: 64
duration: 6
quiz_total: 33
date_updated: 2026-11-20
navigation:
- name: Wire It Up - Electronics in Snakie
- content:
  - section: Overview
    content:
    - name: Wire It Up - Electronics in Snakie
      link: 00_intro.html
    - name: The Electronics Workspace
      link: 01_the_electronics_workspace.html
  - section: Parts
    content:
    - name: The Parts Library
      link: 02_the_parts_library.html
    - name: Placing Parts
      link: 03_placing_parts.html
  - section: Wiring
    content:
    - name: Wiring with Noodles
      link: 04_wiring_with_noodles.html
    - name: The Schematic View
      link: 05_the_schematic_view.html
    - name: Checking Your Wiring
      link: 06_checking_your_wiring.html
  - section: From Wiring to Code
    content:
    - name: Drivers, Help and Code
      link: 07_drivers_help_and_code.html
    - name: Connections and Measurements
      link: 08_connections_and_measurements.html
    - name: Exporting and Sharing
      link: 09_exporting_and_sharing.html
  - section: Project
    content:
    - name: Project - An Obstacle-Avoiding Robot
      link: 10_project_obstacle_robot.html
    - name: Summary and Next Steps
      link: 11_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

A wired circuit does nothing until code runs on it. This lesson is about the bridge between the two: drivers, the help that ships with each part, and code that uses the pins you actually wired.

---

## What a driver is

To measure a distance, the HC SR04 needs a 10 microsecond pulse on Trigger, then the length of the answering pulse on Echo has to be timed and turned into millimetres. You could write that yourself. Or you could use a **driver**: a small Python file, written once, that does it for you.

With a driver the whole job becomes two lines:

```python
sensor = HCSR04(trigger=14, echo=15)
print(sensor.distance_mm())
```

The driver has to be **on the board** before your program can import it.

## The driver banner

Since you added the sensor, a banner has been sitting above the canvas: **1 part needs a driver**. Click the small arrow beside **Install drivers** to see what it means.

![The driver banner opened up](/learn/snakie_electronics/assets/driver_banner.jpg){:class="img-fluid w-100"}

It lists the part, the driver, and where the file will go: `lib/hcsr04.py`.

Click **Install drivers**. Snakie copies the file to the board and the banner disappears. Look in **Device Files** in the Code workspace and you'll find a new `lib` folder with the driver inside.

Three things worth knowing:

- Nothing is copied until you click. Snakie asks first
- The banner checks the board, so it only lists drivers that are missing or out of date
- It works on the simulated device too, so you can try it with no board plugged in

The motors, the battery and the MX1508 have no driver. They don't need one, because the Pico controls the MX1508 with ordinary pin signals.

## Help for every part

Select the MX1508 and click the **?** on its toolbar.

![The MX1508 help page beside the wiring](/learn/snakie_electronics/assets/part_help.jpg){:class="img-fluid w-100"}

A help page opens beside your wiring. Each one follows the same pattern:

- What the part is and what it's good for
- A **wiring table**: which pin goes where
- Warnings about the mistakes people really make
- A **Quick start** program
- An **Open example in editor** button that drops that program into a new file

Read the warning on this one: *VCC is the motor supply, not logic power. Don't feed it from the Pico's 3V3 pin; motors will brown the board out.* That's exactly why our battery goes to the driver and nowhere else.

The **Help** button at the top right of the workspace counts how many of your parts have a page like this.

## Code that matches the wiring

Here's a first program for the motor driver. It ramps the left motor up and down without touching the sensor.

{% capture snakie_code %}
from machine import Pin, PWM
import time

# Left motor: IN1 on GP10, IN2 on GP11
forward = PWM(Pin(10), freq=1000)
backward = PWM(Pin(11), freq=1000)

backward.duty_u16(0)
for percent in range(0, 101, 20):
    forward.duty_u16(int(percent * 65535 / 100))
    print("Left motor at", percent, "%")
    time.sleep(0.5)

forward.duty_u16(0)
print("Stopped")
{% endcapture %}
{% include snakie.html code=snakie_code name="motor_test.py" view="code" %}

Press **Run**. In the simulator you'll see the percentages count up. On a real robot, the left wheel speeds up in five steps and stops.

Look at the comment on line 4. `GP10` and `GP11` aren't numbers picked at random. They're the pins you wired to IN1 and IN2 in lesson 4. This is the whole idea of the Electronics workspace: **the wiring diagram is the source of your pin numbers.**

### How the motor driver works

Each motor has two inputs.

| IN1 | IN2 | The motor |
|---|---|---|
| On | Off | Turns forwards |
| Off | On | Turns backwards |
| Off | Off | Coasts to a stop |
| On | On | Brakes |
{:class="table table-single"}

Instead of fully on, we send **PWM**: the pin flicks on and off a thousand times a second, and `duty_u16` sets how much of the time it's on. More on-time means more speed. `65535` is flat out and `0` is stopped.

## Blocks know your wiring too

If you prefer blocks, switch to the **Blocks** workspace and open the **My parts** drawer. Parts you've wired get a shelf of their own there, with the pins already filled in from this diagram. The [My Parts lesson](/learn/snakie_blocks/08_my_parts.html) in the Snakie Blocks course shows how it works.

---

## Try it Yourself

1. Change the motor test to drive the **right** motor. Which two pin numbers do you need?
2. Make the left motor run backwards at half speed for two seconds.
3. Open the HC SR04's help page and read its Tips. What does a reading of -1 mean?

## Common Issues

**Problem**: `ImportError: no module named 'hcsr04'`
**Solution**: Go back to Electronics and click **Install drivers**.
**Why**: The driver file isn't on the board yet.

**Problem**: Install drivers is greyed out.
**Solution**: Connect a board, or choose the simulated device.
**Why**: Installing copies a file to a device, so there has to be one.

**Problem**: The wrong wheel turns.
**Solution**: Check the pin numbers in your code against the Connections table.
**Why**: The code does exactly what the numbers say, even when the numbers are for the other motor.

## Check your understanding

{% quiz %}
? What is a driver?
- [ ] A cable between the Pico and the motors
- [x] A Python file that knows how to talk to a part
- [ ] A kind of battery
> Drivers turn a part's raw signals into simple calls such as `distance_mm()`.

? Why does the MX1508's VCC go to the battery and not to the Pico's 3V3 pin?
- [ ] The wire is shorter
- [ ] 3V3 is only for LEDs
- [x] Motors draw far more current than the Pico's 3V3 pin can supply
> Motors would drag the Pico's supply down and reset it. They get their own power.

? Where should the pin numbers in your code come from?
- [x] The wiring diagram
- [ ] A random number generator
- [ ] The part's price
> Pin numbers in code must match the pins the part is really wired to.
{% endquiz %}
