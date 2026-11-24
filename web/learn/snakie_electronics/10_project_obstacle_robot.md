---
layout: lesson
title: Project - An Obstacle-Avoiding Robot
author: Kevin McAleer
type: page
cover: /learn/snakie_electronics/assets/banners/10_project_obstacle_robot.jpg
date: 2026-11-20
previous: 09_exporting_and_sharing.html
next: 11_summary.html
description: Wire a two-wheeled robot in Snakie, check it, print its build guide,
  and program it in MicroPython to drive forward and turn away from walls
percent: 88
duration: 9
quiz_total: 33
date_updated: 2026-11-24
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

By the end of this lesson you will be able to plan, wire, check, document and program a two-wheeled obstacle-avoiding robot, using pin numbers taken from your own wiring diagram.

You've met every tool. Now let's use them in order, the way you would on a real project: plan, wire, check, document, code, build.

The robot is the classic first one. Two wheels, one distance sensor. It drives forward until something is closer than 30 cm, then spins on the spot until the way is clear.

![The finished wiring](/learn/snakie_electronics/assets/robot_wired.jpg){:class="img-fluid w-100"}

If you've been following along, most of this is already on your canvas. Treat this lesson as the checklist.

---

## Step 1: Gather the parts

In a new Electronics project, name it **Obstacle Robot** and add:

| Qty | Part | Job |
|---|---|---|
| 1 | Raspberry Pi Pico 2 W | The brain. Already on the canvas |
| 1 | HC SR04 | Measures the distance ahead |
| 1 | MX1508 | Switches battery power to the motors |
| 2 | Yellow TT Motor | One for each wheel |
| 1 | 4×AA Battery Pack | Power for the motors |
{:class="table table-single"}

Arrange them with the Pico on the left, the sensor above, the driver in the middle and the motors and battery to the right.

## Step 2: Wire it

Fifteen wires. Tick them off as you go.

| # | From | To | Kind |
|---|---|---|---|
| 1 | HC SR04 VCC | Pico 3V3 | Power |
| 2 | HC SR04 GND | Pico GND | Ground |
| 3 | HC SR04 Trigger | Pico GP14 | Signal |
| 4 | HC SR04 Echo | Pico GP15 | Signal |
| 5 | MX1508 IN1 | Pico GP10 | Signal |
| 6 | MX1508 IN2 | Pico GP11 | Signal |
| 7 | MX1508 IN3 | Pico GP12 | Signal |
| 8 | MX1508 IN4 | Pico GP13 | Signal |
| 9 | MX1508 OUT1 | Left motor, first terminal | Motor |
| 10 | MX1508 OUT2 | Left motor, second terminal | Motor |
| 11 | MX1508 OUT3 | Right motor, first terminal | Motor |
| 12 | MX1508 OUT4 | Right motor, second terminal | Motor |
| 13 | Battery V+ | MX1508 VCC | Power |
| 14 | Battery GND | MX1508 GND | Ground |
| 15 | Pico GND | MX1508 GND | Shared ground |
{:class="table table-single"}

## Step 3: Check it

1. The **ERC** badge should show a green tick.
2. The count beside **Connections** should be **15**.
3. Open the Connections table and read it against the table above.
4. Switch on **node voltages** and check the battery's supply reaches the MX1508.

## Step 4: Install the driver

Click **Install drivers** on the banner so `hcsr04.py` is on the board.

## Step 5: Document it

Click the **printer** button and save the PDF build guide. Print the Connections page. You'll want it beside you when you build.

## Step 6: The code

Here's the complete program. Run it right here first.

{% capture snakie_code %}
from machine import Pin, PWM
import time

# The pins come straight from the Connections table
TRIGGER = 14
ECHO = 15
left_fwd = PWM(Pin(10), freq=1000)
left_back = PWM(Pin(11), freq=1000)
right_fwd = PWM(Pin(12), freq=1000)
right_back = PWM(Pin(13), freq=1000)

try:
    from machine import time_pulse_us
    from hcsr04 import HCSR04
    sensor = HCSR04(trigger=TRIGGER, echo=ECHO)
    simulated = False
except ImportError:
    # The simulator has no echo to time, so play back some pretend readings
    pretend = [800, 600, 400, 250, 150, 500, 900, 700]
    simulated = True

def distance_mm(step):
    if simulated:
        return pretend[step % len(pretend)]
    return sensor.distance_mm()

def motor(fwd, back, speed):
    # speed runs from -100 (full reverse) to 100 (full ahead)
    duty = int(abs(speed) * 65535 / 100)
    fwd.duty_u16(duty if speed > 0 else 0)
    back.duty_u16(duty if speed < 0 else 0)

def drive(left, right):
    motor(left_fwd, left_back, left)
    motor(right_fwd, right_back, right)

for step in range(16):
    d = distance_mm(step)
    if 0 < d < 300:
        print(d, "mm - too close, turning")
        drive(-60, 60)
    else:
        print(d, "mm - clear, driving")
        drive(60, 60)
    time.sleep(0.25)

drive(0, 0)
print("Stopped")
{% endcapture %}
{% include snakie.html code=snakie_code name="robot.py" view="code" %}

In the simulator the console shows the robot making its decisions:

![The robot program running in Snakie](/learn/snakie_electronics/assets/robot_code.jpg){:class="img-fluid w-100"}

```
800 mm - clear, driving
600 mm - clear, driving
400 mm - clear, driving
250 mm - too close, turning
150 mm - too close, turning
500 mm - clear, driving
```

### How it works

**The pins.** The first block sets up the six pins. Every number comes from the Connections table. If you move a wire, this is the only place the code has to change.

**Real or pretend.** The simulator can run motors' signals, but it can't time a real echo. So the program checks whether `time_pulse_us` exists. On a real Pico it does, and the driver is used. In the simulator it doesn't, so the program plays back a list of pretend distances. That lets you test the decision-making with no hardware at all.

**`motor()`** turns a speed from -100 to 100 into PWM on the right pin. Positive speeds drive the forward input, negative speeds drive the backward input, and the other input is always switched off.

**`drive()`** sets both motors at once. `drive(60, 60)` goes straight. `drive(-60, 60)` runs the left wheel backwards and the right wheel forwards, which spins the robot on the spot.

**The loop** reads the distance and chooses. The test is `0 < d < 300`, not just `d < 300`, because the sensor returns `-1` when it hears no echo at all. No echo means nothing is near, so the robot should keep driving.

The loop runs 16 times and stops, which is polite for a first test on a desk. For a robot that keeps going, change `for step in range(16):` to `while True:` and count the steps yourself.

## Step 7: Build it for real

1. Follow the Connections page of your PDF, one wire at a time, with everything unpowered.
2. Prop the robot up so its wheels are off the desk.
3. Plug the Pico in by USB and run the program from Snakie.
4. Put your hand in front of the sensor. The wheels should change direction.
5. Only then put it on the floor and let it go.

If a wheel turns the wrong way, swap that motor's two wires at the driver. If the robot spins when it should go straight, one motor is reversed.

## Make it yours

- **Slow down near walls.** Drive at a speed that depends on the distance
- **Back up first.** Reverse for a moment before turning
- **Turn a random way.** Pick left or right with `random.choice`
- **Watch it think.** The next course, [Snakie Instruments](/learn/snakie_instruments/), shows how to plot the distance live while the robot runs

> **Teachers**: The simulator version needs no hardware and is safe. For the real build, wire with everything unpowered and run the first test with the wheels off the desk. The battery pack goes to the motor driver only, never to the Pico, and motors draw a lot of current, so a short on the battery side can get hot. Groups of two to four sharing one robot kit keeps costs down.

---

## Try it Yourself

1. Change the turning distance from 300 mm to 150 mm. How does the console output change?
2. Add your own numbers to the `pretend` list to test a robot stuck in a corner.
3. Challenge: make the robot reverse for half a second before it turns.

## Common Issues

**Problem**: `ImportError: no module named 'hcsr04'` on a real Pico.
**Solution**: Click **Install drivers** in the Electronics workspace.
**Why**: The driver file has to be in the board's `lib` folder.

**Problem**: The Pico resets when the motors start.
**Solution**: Check the battery's V+ goes to the MX1508's VCC, and that the Pico's ground is joined to the driver's ground.
**Why**: Motors pull a lot of current when they start. They need their own supply and a shared ground.

**Problem**: The distance is always -1.
**Solution**: Check Trigger and Echo aren't swapped, and that the sensor has power.
**Why**: -1 means the sensor never heard an echo.

**Problem**: The motors hum but don't turn.
**Solution**: Fit fresh batteries, or raise the speed from 60.
**Why**: A TT motor needs a certain amount of power just to get going.

## Check your understanding

{% quiz %}
? Why does the program test `0 < d < 300` and not just `d < 300`?
- [ ] To make it run faster
- [x] The sensor returns -1 when there's no echo, which isn't an obstacle
- [ ] Python requires two comparisons
> Without the first test, "nothing in range" would be treated as "very close".

? What does `drive(-60, 60)` do?
- [ ] Stops both motors
- [ ] Drives straight backwards
- [x] Spins the robot on the spot
> One wheel backwards and one forwards turns the robot without moving it along.

? Why test with the wheels off the desk first?
- [x] So a wiring or code mistake can't send the robot off the edge
- [ ] The motors need air
- [ ] The sensor only works when lifted
> A first run is a test. Keep the robot where it can't hurt itself.
{% endquiz %}
