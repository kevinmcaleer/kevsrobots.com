---
layout: lesson
title: Speed with PWM
author: Kevin McAleer
type: page
cover: /learn/motors_and_drivers/assets/banners/06_speed_with_pwm.jpg
date: 2026-11-21
previous: 05_wiring_a_motor_driver.html
next: 07_a_motor_class.html
description: Control DC motor speed with pulse width modulation, choose a sensible
  frequency and measure your motor's dead zone.
percent: 35
duration: 5
date_updated: 2026-11-21
navigation:
- name: Motors and motor drivers
- content:
  - section: Introduction
    content:
    - name: Motors and motor drivers
      link: 00_intro.html
  - section: Motor basics
    content:
    - name: How motors work
      link: 01_how_motors_work.html
    - name: Why you need a motor driver
      link: 02_why_you_need_a_driver.html
    - name: Powering motors
      link: 03_powering_motors.html
  - section: DC motors
    content:
    - name: The H-bridge
      link: 04_the_h_bridge.html
    - name: Wiring a motor driver
      link: 05_wiring_a_motor_driver.html
    - name: Speed with PWM
      link: 06_speed_with_pwm.html
    - name: A motor class
      link: 07_a_motor_class.html
    - name: Driving two motors
      link: 08_driving_two_motors.html
    - name: Ramping and braking
      link: 09_ramping_and_braking.html
  - section: Servos
    content:
    - name: Servo basics
      link: 10_servo_basics.html
    - name: A servo class and calibration
      link: 11_a_servo_class_and_calibration.html
    - name: Continuous rotation servos
      link: 12_continuous_rotation_servos.html
  - section: Steppers and sensing
    content:
    - name: Stepper motors
      link: 13_stepper_motors.html
    - name: Measuring speed with encoders
      link: 14_measuring_speed_with_encoders.html
  - section: Project
    content:
    - name: Project - the Scout drive base
      link: 15_project_drive_base.html
    - name: Summary
      link: 16_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

So far the motor has two speeds: off and flat out. To control speed, we cannot lower the battery voltage from code, but we *can* switch the driver on and off so quickly that the motor feels an average. That trick is **pulse width modulation**, or PWM, and the Pico has 16 PWM channels ready to go.

## Duty cycle

PWM repeats a cycle many times a second. In each cycle the output is high for a fraction of the time. That fraction is the **duty cycle**.

```python
# One PWM cycle at 1 kHz
freq_hz = 1000
period_ms = 1000 / freq_hz        # 1.0 ms per cycle

duty_percent = 25
on_time_ms = period_ms * duty_percent / 100   # 0.25 ms on, 0.75 ms off

# The motor's average voltage, with a 6 V battery
average_v = 6.0 * duty_percent / 100          # 1.5 V on average
print(on_time_ms, average_v)
```

The motor's inertia smooths the pulses out, so 25% duty feels like a slow, gentle push.

MicroPython on the Pico expresses duty as a 16-bit number from 0 to 65535, using `duty_u16()`:

```python
# Convert between percent and the Pico's duty_u16 range
def percent_to_duty(percent):
    return int(percent * 65535 / 100)

print(percent_to_duty(0))     # 0      always off
print(percent_to_duty(25))    # 16383
print(percent_to_duty(50))    # 32767
print(percent_to_duty(100))   # 65535  always on
```

## Choosing a frequency

The frequency is how many cycles per second. Too low and the motor visibly stutters and whines. Too high and the driver spends its time switching instead of delivering power.

| Frequency | What you notice |
|---|---|
| Below about 200 Hz | Jerky, vibrating motor |
| 500 Hz to 2 kHz | Smooth, audible whine on the L298N |
| 10 kHz to 20 kHz | Silent to most people, great for the DRV8833 |
{:class="table table-single"}

We use 1 kHz for the L298N, which switches slowly, and 10 kHz for the DRV8833, which does not mind.

## Finding the dead zone

Here is the surprise: a motor does not move at 1% duty. It needs a minimum push to overcome friction and gearbox drag. Below that, you only hear a whine. This is the **dead zone**, and every motor and battery combination has its own.

Run this on the Pico with the left motor wired to the L298N. Wheel off the floor, please. Watch carefully, and note when the wheel first begins to turn:

```python
from machine import Pin, PWM
from time import sleep_ms

in1 = Pin(0, Pin.OUT)
in2 = Pin(1, Pin.OUT)
ena = PWM(Pin(2))
ena.freq(1000)

in1.value(1)   # forward
in2.value(0)

try:
    for percent in range(0, 101, 5):
        duty = int(percent * 65535 / 100)
        ena.duty_u16(duty)
        print("duty", percent, "percent ->", duty)
        sleep_ms(700)
finally:
    ena.duty_u16(0)
```

On a typical TT motor running from 4xAA through an L298N, the wheel starts somewhere between 25% and 45%. Write your number down. We will call it `deadzone`.

## Mapping speed past the dead zone

We want a "speed" of 1 to feel like the *slowest useful* speed, and 100 to be flat out. So we map the speed range onto duty values *above* the dead zone:

```python
def speed_to_duty(speed, deadzone=30):
    """Map speed 0..100 onto duty_u16, skipping the dead zone."""
    if speed <= 0:
        return 0
    percent = deadzone + (100 - deadzone) * speed / 100
    return int(percent * 65535 / 100)

# With a dead zone of 30 percent:
print(speed_to_duty(0))     # 0      stopped
print(speed_to_duty(1))     # 20119  just above where the motor starts
print(speed_to_duty(50))    # 42597  65 percent duty
print(speed_to_duty(100))   # 65535  full power
```

Walk through the middle line: speed 50 is half way between the dead zone (30) and the maximum (100), which is 30 + 70 x 0.5 = 65 percent duty. That mapping is what stops Scout from creeping at speed 10 and then suddenly lurching at speed 40.

## Try it Yourself

1. Run the dead zone script and write down the percentage at which your motor starts. Try again with the wheel on the floor. Does the number change?
2. Change `ena.freq(1000)` to 100, then 20000, and listen. What happens to the sound?
3. Challenge: find the dead zone separately in forward and reverse. Are they the same?

## Common Issues

**Problem**: A `ValueError` about the duty value, or the motor behaves oddly at high values.
**Solution**: Keep duty between 0 and 65535. Clamp speeds before you convert them.
**Why**: Pico PWM duty is a 16-bit value, and out-of-range numbers are not accepted.

**Problem**: Two pins in the same PWM slice ignore my second frequency.
**Solution**: Set the same frequency for any pins in a slice, for example GP0 and GP1 together.
**Why**: Each Pico PWM slice has one counter shared by its two output channels, so the last frequency you set wins for both.

**Problem**: The motor whines but will not turn at low speed.
**Solution**: That is the dead zone. Raise the minimum duty with the `deadzone` value.
**Why**: Friction needs a minimum torque to overcome before the shaft turns.
