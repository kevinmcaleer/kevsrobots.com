---
layout: lesson
title: Controlling Motors with PWM
type: page
description: Learn how to control motors using Pulse Width Modulation (PWM) with the Raspberry Pi Pico and MicroPython.
---

## Overview

A motor needs far more current than a GPIO pin can supply, so we use a motor driver chip between the Pico and the motor. In this lesson you'll wire up an L293D driver and control a DC motor's direction and speed.

---

## What you'll need

* A Raspberry Pi Pico
* A small DC motor (3V to 6V)
* An L293D motor driver chip (a DRV8833 board works the same way)
* A battery pack for the motor, such as 4 x AA
* A [breadboard](/resources/how_it_works/breadboards.html) and jumper wires

---

## Wiring the circuit

The L293D has two power pins: `VCC1` (pin 16) powers the logic, and `VCC2` (pin 8) powers the motor.

1. Connect the Pico's `VBUS` (5V from USB) to L293D `VCC1` (pin 16)
2. Connect the battery pack's `+` to L293D `VCC2` (pin 8)
3. Connect the battery `-`, the Pico `GND` and the L293D ground pins (4, 5, 12, 13) together. The grounds **must** be shared
4. Connect Pico `GP0` to L293D `IN1` (pin 2)
5. Connect Pico `GP1` to L293D `IN2` (pin 7)
6. Connect Pico `GP2` to L293D `EN1` (pin 1). This is the speed control
7. Connect the motor to L293D `OUT1` (pin 3) and `OUT2` (pin 6)

---

## Writing the code

```python
from machine import Pin, PWM
from time import sleep

in1 = Pin(0, Pin.OUT)
in2 = Pin(1, Pin.OUT)
speed = PWM(Pin(2))
speed.freq(1000)

def forward(percent):
    in1.value(1)
    in2.value(0)
    speed.duty_u16(int(percent * 65535 / 100))

def backward(percent):
    in1.value(0)
    in2.value(1)
    speed.duty_u16(int(percent * 65535 / 100))

def stop():
    in1.value(0)
    in2.value(0)
    speed.duty_u16(0)

forward(50)
sleep(2)
backward(100)
sleep(2)
stop()
```

- `IN1` and `IN2` set the **direction**: one high and one low makes the motor spin, and swapping them reverses it
- The PWM signal on `EN1` sets the **speed**
- `stop()` sets both inputs low and the speed to zero

> ## Low speeds
>
> Most small motors won't start turning below about 30% duty. That's normal: there isn't enough power to overcome friction.

---

## Try it yourself

Write a `ramp_up()` function that takes the motor from 0% to 100% over two seconds.

{% quiz %}
? Which pins set the motor's direction on the L293D?
- [x] IN1 and IN2
- [ ] EN1
- [ ] VCC1 and VCC2
> IN1 and IN2 choose the direction. The PWM signal on EN1 sets the speed.

? Why must the battery ground be connected to the Pico's GND?
- [ ] It makes the motor spin faster
- [x] The signals from the Pico need a shared reference to be understood
- [ ] It charges the battery
> Voltages are always measured relative to ground. Without a common ground, the driver can't read the Pico's signals reliably.
{% endquiz %}

---
