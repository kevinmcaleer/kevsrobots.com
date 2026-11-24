---
title: The library and your first move
description: Use the STS3215 MicroPython library to ping a servo, enable torque and move it by position and by degrees
layout: lesson
type: page
cover: /learn/bus_servos/assets/banners/03_the_library_and_first_move.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

The wiring works, so let's make something move. In this lesson one servo goes to a few positions, and you learn the numbers that describe them.

---

## Meet the library

Everything in this course goes through one small file, `sts3215.py`, from Kev's repo at [github.com/kevinmcaleer/sts3215](https://github.com/kevinmcaleer/sts3215). Copy it to the Pico with `mpremote cp sts3215.py :` or with Thonny, as in the last lesson. The class is `STS3215`, and you create one object for the whole bus:

```python
STS3215(uart_id=0, tx_pin=0, rx_pin=1, baudrate=1_000_000)
```
{: .no-snakie }

You don't make one object per servo. You make one for the **bus**, then say which servo you mean with the ID in each call. The calls for this lesson are:

- `ping(servo_id)` returns `True` if the servo answered
- `set_torque(servo_id, True)` switches the motor on so the servo holds and moves. `False` lets it go limp
- `move(servo_id, position, speed=0, acc=50)` starts a move

## Positions

An STS3215 reports **12-bit** positions: 0 to 4095 covers one full turn of 360 degrees. So 0 is the start, and 2048 is halfway round.

| Degrees | Position |
| --- | --- |
| 0 | 0 |
| 90 | 1023 |
| 180 | 2047 |
| 270 | 3071 |
| 360 | 4095 |
{:class="table table-single"}

The library does the sums for you with two helper functions, `degrees_to_position(degrees)` and `position_to_degrees(position)`. Both map 0 to 360 onto 0 to 4095, so position 2048 is 180.04 degrees. That tiny rounding is nothing to worry about.

## Speed and acceleration

`move()` takes two more numbers:

- **speed**: how fast to travel. `0` means maximum speed, and the library says 1 to 4095 otherwise. Kev's demos use 2400 for lively moves. The Buddy arm later treats it as position steps per second, so 600 is about 53 degrees per second
- **acc**: the acceleration, 0 to 254. A gentle number such as 50 stops the servo lunging off and braking hard

## Your first move

Here is a complete program. It checks the servo is there, switches torque on, then visits four positions.

```python
import time
from sts3215 import STS3215, degrees_to_position   # sts3215.py from the repo

SERVO_ID = 1
SPEED = 600     # position steps per second, about 53 degrees per second
ACC = 50

servo = STS3215(uart_id=0, tx_pin=0, rx_pin=1, baudrate=1_000_000)

if not servo.ping(SERVO_ID):
    print("No reply from servo", SERVO_ID, "- check wiring and power")
else:
    print("Servo", SERVO_ID, "found")
    servo.set_torque(SERVO_ID, True)     # hold and move

    for degrees in (180, 90, 270, 180):
        position = degrees_to_position(degrees)
        print("Going to", degrees, "degrees (position", position, ")")
        servo.move(SERVO_ID, position, speed=SPEED, acc=ACC)
        time.sleep(2)                    # wait for it to arrive

    servo.set_torque(SERVO_ID, False)    # let go when we are done
    print("Done")
```

Run it, and the servo swings out to 90 degrees, across to 270, and back to the middle. `move()` returns immediately. It tells the servo where to go and doesn't wait for it, which is why the program sleeps for two seconds. Next lesson replaces that guess with real feedback.

**Why enable torque first?** A servo with torque off is limp. Switching torque on is cheap insurance that it will actually hold and move.

**Common mistakes:**

- Sending `move()` for a position outside 0 to 4095. Keep targets inside the range
- Forgetting that `move()` doesn't wait. If a second `move()` follows straight away, it replaces the first

## Try it Yourself

1. Change `SPEED` to 200, then to 2400, and watch the difference. What does `speed=0` do?
2. Change `ACC` to 0 and then to 200. Can you hear and see the change in how the servo starts and stops?
3. Challenge: sweep the servo from 90 to 270 degrees in steps of 10, with `time.sleep_ms(100)` between steps, to make a slow, smooth scan.

## Common Issues

**Problem**: The servo is found but doesn't move.
**Solution**: Check `set_torque(SERVO_ID, True)` ran, and that your target isn't the position it is already at.
**Why**: With torque off the servo is limp, and a target equal to the current position produces no motion.

**Problem**: `ImportError: no module named 'sts3215'`.
**Solution**: Copy `sts3215.py` onto the Pico, into the top-level folder.
**Why**: MicroPython can only import files that are on the board's own filesystem.

**Problem**: The servo jerks hard at the start of every move.
**Solution**: Raise `acc` from 0 to something like 50.
**Why**: Acceleration 0 asks for full speed instantly.
