---
title: Reading feedback
description: Ask a servo for its position, speed, whether it is moving and its temperature, and use the answers to know when a move has finished
layout: lesson
type: page
cover: /learn/bus_servos/assets/banners/04_reading_feedback.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Here's where bus servos pull ahead of anything you used in the motors course: you can ask a servo what it is doing. This lesson reads four things back, then uses them to replace "sleep for two seconds and hope" with "wait until it has actually arrived".

---

## Four questions to ask

Each of these is a method on the same `STS3215` object, with the servo ID as the argument:

| Method | Answers | Returns |
| --- | --- | --- |
| `read_position(id)` | Where are you? | 0 to 4095, or `None` |
| `read_pos_speed(id)` | Where are you, and how fast are you going? | `(position, speed)`, or `(None, None)` |
| `read_moving(id)` | Are you still moving? | `1` moving, `0` stopped, or `None` |
| `read_temperature(id)` | How hot are you? | Degrees Celsius, or `None` |
{:class="table table-single"}

The `None` matters. If the servo doesn't answer, because of a loose wire or a wrong ID, the library gives you `None` rather than a crash, so **always check for it** before you do sums on a reading. The speed from `read_pos_speed` is signed, so a negative number means it is turning the other way.

## A status report

This program prints everything the servo will tell us, with the position also shown in degrees. Copy `sts3215.py` to the Pico first.

```python
import time
from sts3215 import STS3215, position_to_degrees   # sts3215.py from the repo

SERVO_ID = 1
servo = STS3215(uart_id=0, tx_pin=0, rx_pin=1, baudrate=1_000_000)

for _ in range(5):
    position, speed = servo.read_pos_speed(SERVO_ID)
    moving = servo.read_moving(SERVO_ID)
    temperature = servo.read_temperature(SERVO_ID)

    if position is None:
        print("No answer from servo", SERVO_ID)
    else:
        degrees = position_to_degrees(position)
        print("pos", position, "(%.1f deg)" % degrees,
              "speed", speed, "moving", moving, "temp", temperature, "C")
    time.sleep(1)
```

With torque off, push the servo by hand while it runs. The position changes, and the speed jumps as you turn it. The servo is now a sensor.

## Waiting for a move to finish

`move()` returns straight away, so how do you know the servo has arrived? Ask it. This helper polls `read_moving()` until it reports stopped, or gives up after a timeout. It uses `ticks_ms()` and `ticks_diff()`, the safe way to time things in MicroPython.

```python
import time
from sts3215 import STS3215, position_to_degrees   # sts3215.py from the repo

SERVO_ID = 1
servo = STS3215(uart_id=0, tx_pin=0, rx_pin=1, baudrate=1_000_000)


def wait_until_stopped(servo_id, timeout_ms=5000):
    """Return True once the servo reports stopped, False if we time out."""
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
        if servo.read_moving(servo_id) == 0:
            return True
        time.sleep_ms(50)
    return False


def go_to(position):
    servo.move(SERVO_ID, position, speed=500, acc=50)
    time.sleep_ms(100)          # give the servo a moment to start moving
    arrived = wait_until_stopped(SERVO_ID)
    actual = servo.read_position(SERVO_ID)
    if arrived and actual is not None:
        print("target", position, "actual", actual, "error", actual - position)
    else:
        print("did not finish, or no reading")


servo.set_torque(SERVO_ID, True)
go_to(1500)
go_to(2600)
go_to(2048)
print("Final position: %.1f degrees" % position_to_degrees(servo.read_position(SERVO_ID) or 0))
```

Look at the **error** column. The servo is closed-loop, so it lands very close to the target, a few steps at most. A PWM servo can't tell you that, and you'd never know.

The short `sleep_ms(100)` before waiting is deliberate. Ask "are you moving?" in the instant after the command and the servo may not have started yet, so it would say "stopped" too early.

**Why this matters:** later, six joints will move at once, and the arm needs to know when every joint has arrived before it starts the next step.

**Common mistakes:**

- Doing arithmetic on a reading without checking for `None`. A missed reply then becomes a confusing `TypeError`
- Polling with no timeout. If a servo is stuck, your program waits for ever

## More registers

The servo reports more than the library exposes. `sts3215.py` already names registers for load (`REG_PRESENT_LOAD`) and voltage (`REG_PRESENT_VOLTAGE`), but has no method for them yet. Adding one is a good exercise below.

## Try it Yourself

1. Run the status report with torque off and turn the servo by hand. What does the speed do as you slow down?
2. Change the timeout in `wait_until_stopped` to 200 ms and ask for a slow move with `speed=100`. What does the program print?
3. Challenge: make a subclass `class MyServo(STS3215)` with a `read_voltage(self, servo_id)` method. Copy the pattern from `read_temperature` in `sts3215.py`: it calls `self._send(...)` with the register `self.REG_PRESENT_VOLTAGE` and reads one byte back with `self._receive(...)`. The value is the servo's raw number, so check the Feetech datasheet for its units.

## Common Issues

**Problem**: `TypeError: unsupported operand type(s)` when subtracting a reading.
**Solution**: Check `if position is None` first.
**Why**: A missed reply returns `None`, and you can't subtract from `None`.

**Problem**: `wait_until_stopped` returns immediately, before the servo moves.
**Solution**: Add a short `sleep_ms(100)` after `move()`.
**Why**: The servo takes a moment to begin moving, and in that moment it still reports stopped.

---

## Check your understanding

{% quiz %}
? What does `read_position()` return if the servo doesn't reply?
- [ ] 0
- [ ] It raises an exception
- [x] `None`
> The library returns `None` for a missed reply, so check for it before doing sums.

? Why wait a short moment between `move()` and polling `read_moving()`?
- [ ] The UART needs to warm up
- [x] The servo may not have started moving yet, so it would report stopped too early
- [ ] `read_moving()` is slow
> Polling the instant after a command can catch the servo before it starts.
{% endquiz %}
