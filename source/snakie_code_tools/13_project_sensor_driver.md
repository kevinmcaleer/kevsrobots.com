---
title: Project - A Tidy Sensor Driver
description: Write an HC-SR04 ultrasonic sensor driver in Snakie and take it through the full toolchain, from autocomplete and linting to refactoring, compiling and Git
layout: lesson
type: page
cover: /learn/snakie_code_tools/assets/banners/13_project_sensor_driver.jpg
date_updated: 2026-11-21
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Time to use the whole toolbox on one job. You'll write a small **driver**, a module that hides the fiddly details of a piece of hardware behind a friendly function, and take it from first draft to something you'd be happy to share.

The hardware is the HC-SR04 ultrasonic range finder, the sensor on the front of the robot from [Wire It Up](/learn/snakie_electronics/). You don't need one to follow along.

---

## How the sensor works

The HC-SR04 measures distance with sound, like a bat.

1. Your code sends a 10 microsecond pulse on the **Trigger** pin
2. The sensor sends out a burst of ultrasound
3. It holds its **Echo** pin high for as long as the sound takes to come back
4. Your code times that pulse

Sound travels about 343 metres a second, and it goes there *and back*, so:

```python
# duration is in microseconds
# 343 m/s is 0.343 mm per microsecond, halved for the round trip
distance_mm = duration * 343 // 2000
```

## Step 1: set up the project

1. Make a folder called `obstacle-robot` and open it in Snakie with **Open Folder**
2. In **Source**, click **Initialise Repository**
3. Create a file called `hcsr04.py`

Starting Git first means every later step leaves a trail.

## Step 2: write it with autocomplete

Type this in. Lean on autocomplete: `from machine import ` then <kbd>Ctrl</kbd>+<kbd>Space</kbd> will find `time_pulse_us` for you, and typing `time_pulse_us(` shows what it wants.

```python
from machine import Pin, time_pulse_us
import time


class HCSR04:
    """Driver for the HC-SR04 ultrasonic range finder."""

    def __init__(self, trigger, echo, timeout_us=30000):
        self.trigger = Pin(trigger, Pin.OUT)
        self.echo = Pin(echo, Pin.IN)
        self.timeout_us = timeout_us
        self.trigger.value(0)

    def _pulse(self):
        self.trigger.value(1)
        time.sleep_us(10)
        self.trigger.value(0)
        return time_pulse_us(self.echo, 1, self.timeout_us)

    def distance_mm(self):
        """Return the distance in millimetres, or -1 if nothing was heard."""
        duration = self._pulse()
        if duration < 0:
            return -1
        return duration * 343 // 2000
```

A few things to notice:

- `_pulse` starts with an underscore. That's Python's way of saying *this is for the class's own use*
- `time_pulse_us` returns a negative number if no echo arrives in time, so `distance_mm` returns `-1` to mean *nothing heard*
- `// 2000` is integer division. No floats, so no memory allocated on every reading

Hover over `time_pulse_us` to read its documentation, and use **Help for symbol** on `Pin` if you want a reminder of `Pin.OUT` and `Pin.IN`.

## Step 3: check it

Click **Problems**. With ruff installed you may see an import-order note. Fix anything marked ⚠ first.

Now deliberately break it: change one `self.echo` to `self.ecko`. Notice that nothing complains, because an attribute on `self` can't be checked without running the code. Change `duration` to `duraton` in the `if` instead, and an *undefined name* finding appears at once. Linters are powerful, and they have limits. Put both back.

Commit: `Add HC-SR04 driver`.

## Step 4: the program that uses it

Create `main.py`. This first version is written the way most of us would write it on day one.

```python
from machine import Pin, PWM
import time
from hcsr04 import HCSR04

LEFT_MOTOR_PIN = 10
RIGHT_MOTOR_PIN = 12
TOO_CLOSE_MM = 200

sensor = HCSR04(trigger=14, echo=15)
left = PWM(Pin(LEFT_MOTOR_PIN))
right = PWM(Pin(RIGHT_MOTOR_PIN))


def drive(speed):
    duty = int(speed * 65535 / 100)
    left.duty_u16(duty)
    right.duty_u16(duty)


while True:
    distance = sensor.distance_mm()
    if distance < TOO_CLOSE_MM:
        drive(0)
    else:
        drive(60)
    time.sleep(0.1)
```

Commit it as it is: `First version of main`. A working-but-rough commit is exactly what you want before tidying.

## Step 5: refactor

Open **Problems**. This is the file from lesson 4, and it has six hints.

1. Right-click and choose **Tidy this file (Snakie)**. Read the preview, then **Apply**. Run it again until the `const()` and `sleep_ms` hints have gone. Then make the `//` change on the `duty` line, from the lightbulb if one is offered or by hand
2. Open **Source** and click `main.py`. Read the diff. Every change Tidy made is there in red and green
3. Commit: `Tidy main with const and integer maths`

One hint is left: *This loop blocks for 100 ms every pass*. There's no lightbulb, so this one is yours.

## Step 6: stop blocking

`time.sleep_ms(100)` makes the robot deaf for a tenth of a second, every loop. Nothing else can happen in that time: no button check, no second sensor, no LED blink.

The fix is to ask *"has 100 ms passed yet?"* instead of waiting for it. Here's the finished program. So that it runs on this page, a small `FakeSensor` class stands in for the real driver and the loop stops after six readings.

{% capture snakie_code %}
from machine import Pin, PWM
from micropython import const
import time

_LEFT_MOTOR_PIN = const(10)
_RIGHT_MOTOR_PIN = const(12)
_TOO_CLOSE_MM = const(200)
_CHECK_EVERY_MS = const(100)


class FakeSensor:
    """Stands in for the HC-SR04 so this runs on the simulator."""

    def __init__(self):
        self._readings = [600, 400, 250, 150, 500, 900]
        self._index = 0

    def distance_mm(self):
        value = self._readings[self._index % len(self._readings)]
        self._index += 1
        return value


sensor = FakeSensor()
left = PWM(Pin(_LEFT_MOTOR_PIN))
right = PWM(Pin(_RIGHT_MOTOR_PIN))


def drive(speed):
    duty = speed * 65535 // 100
    left.duty_u16(duty)
    right.duty_u16(duty)


last_check = time.ticks_ms()
checks = 0

while checks < 6:
    now = time.ticks_ms()
    if time.ticks_diff(now, last_check) >= _CHECK_EVERY_MS:
        last_check = now
        checks += 1
        distance = sensor.distance_mm()
        if distance < _TOO_CLOSE_MM:
            drive(0)
            print(distance, "mm - too close, stopping")
        else:
            drive(60)
            print(distance, "mm - clear, driving")
    # The loop is free to do other jobs here

drive(0)
print("Done")
{% endcapture %}
{% include snakie.html code=snakie_code name="robot.py" view="code" %}

The important lines:

```python
now = time.ticks_ms()
if time.ticks_diff(now, last_check) >= _CHECK_EVERY_MS:
    last_check = now
```

`ticks_ms()` is a millisecond counter. `ticks_diff()` subtracts two readings safely, even when the counter wraps round to zero, which it eventually does. Always use it, never a plain minus.

The constants now begin with an underscore, as in `_TOO_CLOSE_MM = const(200)`. The `const()` help article from lesson 5 explains why: with the underscore, the name doesn't even take up space in memory.

For the real robot, delete the `FakeSensor` class, bring back `from hcsr04 import HCSR04` and `sensor = HCSR04(trigger=14, echo=15)`, and change the loop to `while True:`.

Check **Problems** once more. Snakie now points at the `print` calls inside the loop: printing is slow, and on a finished robot you'd switch it off. A new hint for a new version. That's the tool doing its job.

Commit: `Make the main loop non-blocking`.

## Step 7: compile the driver

The driver is finished and stable. `main.py` will keep changing. So:

1. Right-click `hcsr04.py` and choose **Compile to .mpy**
2. Double-click `hcsr04.mpy` and check **DEFINES** lists `HCSR04` with its three methods
3. Upload `hcsr04.mpy` and `main.py` to the board, not `hcsr04.py`

Keep `hcsr04.py` in Git. It's the source, and you'll need it to recompile after a firmware update.

## What you ended up with

| File | In Git? | On the board? |
|---|---|---|
| `main.py` | Yes | Yes |
| `hcsr04.py` | Yes | No |
| `hcsr04.mpy` | Your choice | Yes |
| `.gitignore` | Yes | No |
{:class="table table-single"}

And a history of four commits that tells the story of how it got there.

---

## Try it Yourself

1. Add a `distance_cm()` method to the driver that uses `distance_mm()`. Recompile and check it appears under DEFINES.
2. Add a second timed job to the loop: blink the on-board LED every 500 ms, using its own `last_blink` variable.
3. Add a `DEBUG = False` flag and only print when it's true. Does the hint go away?

## Common Issues

**Problem**: `distance_mm()` always returns -1 on real hardware.
**Solution**: Check Trigger and Echo aren't swapped, and that the sensor has power and ground.
**Why**: -1 means no echo pulse arrived before the timeout.

**Problem**: `ImportError: no module named 'hcsr04'`.
**Solution**: Upload `hcsr04.mpy` (or `hcsr04.py`) to the board, next to `main.py` or in `/lib`.
**Why**: `main.py` imports the driver from the board's own file system.

**Problem**: `ImportError: can't import name time_pulse_us` on the simulated device.
**Solution**: Use the `FakeSensor` version on the simulator, and the real driver on a board.
**Why**: The simulator doesn't have that hardware timing function.

## Check your understanding

{% quiz %}
? Why commit the rough first version before tidying?
- [ ] Git requires it
- [x] So the diff shows exactly what the tidy-up changed, and you can go back
- [ ] It makes the file smaller
> A commit before a risky change is a free undo for the whole file.

? Why use time.ticks_diff() instead of subtracting?
- [ ] It's shorter
- [ ] It's required by const()
- [x] It still gives the right answer when the millisecond counter wraps round
> The counter eventually returns to zero, and ticks_diff() handles that for you.

? Which file do you compile to .mpy in this project?
- [x] The finished driver, hcsr04.py
- [ ] main.py
- [ ] .gitignore
> Compile stable library code. Keep the file you're still editing as .py.
{% endquiz %}
