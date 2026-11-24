---
layout: lesson
title: GPIO Pin Types
type: page
description: Learn about the different types of GPIO pins on the Raspberry Pi Pico and their basic functions.
cover: /learn/micropython_gpio/assets/banners/01_gpio_pin_types.jpg
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to name the jobs a Pico GPIO pin can do and say which pins can read an analog voltage.

## Overview

In this lesson we'll look at the different jobs a Pico GPIO pin can do. The clever part is that most pins can do *several* jobs: you choose which one in your code.

---

## Different types of GPIO pins

The Raspberry Pi Pico has 26 usable GPIO pins (`GP0` to `GP22` and `GP26` to `GP28`). Each one can be set up as:

- `Digital I/O` - send or receive simple on/off (1 or 0) signals: LEDs, buttons, switches
- `PWM` (Pulse Width Modulation) - switch on and off very fast to dim LEDs, set motor speed or position servos. **Every** GPIO pin can do PWM
- `UART` - simple two-wire serial, used by GPS modules and other microcontrollers
- `I2C` (Inter-Integrated Circuit) - a two-wire bus that can talk to many sensors and displays at once
- `SPI` (Serial Peripheral Interface) - a faster four-wire bus, used by displays and SD cards

Three pins have an extra trick:

- `ADC` (Analog-to-Digital Converter) - `GP26`, `GP27` and `GP28` can measure a voltage between 0V and 3.3V, so you can read potentiometers and analog sensors

The rest of the 40 pins around the edge are power (`3V3`, `VBUS`, `VSYS`) and ground (`GND`) pins, plus a couple of control pins (`RUN`, `3V3_EN`).

---

## Basic functions of GPIO pins

`Digital I/O pins` send or receive digital signals, which can control LEDs, read button presses, or signal other digital devices.

`PWM` lets you control the brightness of LEDs, the speed of motors, or the position of servos by adjusting the duty cycle of the PWM signal.

`ADC pins` read analog signals from sensors, such as light sensors or potentiometers, and convert them into numbers your code can use.

`UART`, `I2C` and `SPI` let the Pico communicate with other devices, such as sensors, displays, or other microcontrollers. See the glossary for [I2C](/resources/glossary#i2c) and [SPI](/resources/glossary#spi).

---

## MicroPython code example

The code below lights the Pico's on-board LED:

```python
import machine

led = machine.Pin("LED", machine.Pin.OUT)
led.value(1)
```

- `import machine` gives us access to the Pico's hardware
- `machine.Pin("LED", machine.Pin.OUT)` sets up the on-board LED pin as an output
- `led.value(1)` sets the pin high (3.3V), so the LED turns on

> ## Pico or Pico W?
>
> On the original Pico the on-board LED is wired to `GP25`, so you'll often see `Pin(25, Pin.OUT)` in older tutorials. On the Pico W the LED is connected through the wireless chip, so `Pin(25)` won't work. Using the name `"LED"` works on both boards with current MicroPython firmware.

---

## Try it Yourself

1. Change `led.value(1)` to `led.value(0)` and run the code again.
2. From the Thonny shell, call `led.toggle()` a few times and watch the on-board LED.
3. Challenge: use `time.sleep()` and `led.toggle()` in a `while True:` loop to make the LED blink twice a second.

## Common Issues

- **Problem**: `Pin(25, Pin.OUT)` raises an error on a Pico W.
- **Solution**: Use `Pin("LED", Pin.OUT)` instead.
- **Why**: On the Pico W the LED hangs off the wireless chip, not GP25, so the name is the portable way to reach it.

## Check your understanding

{% quiz %}
? Which Pico pins can be used for PWM?
- [ ] Only GP18, GP19 and GP20
- [ ] Only the pins marked ADC
- [x] Every GPIO pin
> The Pico has 8 PWM slices with 2 channels each, and every GPIO pin is connected to one of them.

? Which pins can read an analog voltage with the ADC?
- [ ] GP0 to GP3
- [x] GP26, GP27 and GP28
- [ ] Any GPIO pin
> Only GP26, GP27 and GP28 (ADC0 to ADC2) are wired to the analog-to-digital converter.

? Which of these is a power pin rather than a GPIO pin?
- [ ] GP15
- [x] VBUS
- [ ] GP28
> VBUS, VSYS and 3V3 are power pins. GP numbers are GPIO pins.
{% endquiz %}

---
