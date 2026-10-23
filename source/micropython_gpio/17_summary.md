---
layout: lesson
title: Recap and Review
type: page
description: A summary of what you have learned in the Raspberry Pi Pico with MicroPython - GPIO Mastery course.
---

## Overview

Congratulations on finishing `Raspberry Pi Pico with MicroPython - GPIO Mastery`! Here's a quick recap.

---

## What you've learned

* The different jobs a GPIO pin can do, and the difference between GP numbers and physical pin numbers
* Why the Pico is a 3.3V device, and how to protect it from 5V signals
* Digital inputs and outputs, including pull-up resistors for buttons
* PWM with `duty_u16()` to fade LEDs, set motor speed and position servos
* Reading voltages with the ADC on GP26, GP27 and GP28
* Reading temperature from the built-in sensor, a DS18B20 and a DHT11
* Talking to other devices over UART, I2C and SPI
* Measuring distance with an HC-SR04
* Why every circuit, and every power supply, needs a shared ground

---

## Quick reference

| Task | Code |
|------|------|
| Output | `Pin(15, Pin.OUT).value(1)` |
| Button input | `Pin(14, Pin.IN, Pin.PULL_UP).value()` |
| PWM | `p = PWM(Pin(15)); p.freq(1000); p.duty_u16(32768)` |
| Analog input | `ADC(26).read_u16()` |
| I2C scan | `I2C(0, sda=Pin(0), scl=Pin(1)).scan()` |
| UART | `UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))` |
{:class="table table-striped"}

---

{% quiz %}
? Which method sets the PWM duty cycle on the Pico?
- [ ] `duty()`
- [x] `duty_u16()`
- [ ] `read_u16()`
> `duty_u16()` takes 0 to 65535. `read_u16()` is for reading the ADC.

? You need to read a light sensor that outputs 0V to 3.3V. Which pin do you use?
- [ ] GP15
- [x] GP27
- [ ] VBUS
> GP27 is ADC1, one of the three analog-capable pins.
{% endquiz %}

---

## What's next?

* Put these skills into a robot with [MicroPython Robotics Projects with the Raspberry Pi Pico](/learn/micropython_robotics/)
* Go further with the language in [Intermediate level MicroPython](/learn/intermediate_micropython/)
* Build a two-wheeled robot with [BurgerBot](/learn/burgerbot/)

If you have feedback or questions, leave a comment below. Thanks for taking the course!

---
