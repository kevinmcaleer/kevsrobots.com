---
title: Your First Connection
description: Install Arduino's Modulino library on the FeatherS2 Neo, build the I2C bus, scan for all five modules and fix the auto-detect error
layout: lesson
type: page
cover: /learn/robot_senses/assets/banners/04_first_connection.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Time to plug in and say hello. In this lesson you install the library, build the I2C bus, and scan for your five modules. It's the single most useful test in the whole course, and you'll use it again whenever something goes quiet.

## Step 1: put MicroPython on the board

Flash MicroPython onto the FeatherS2 Neo. The firmware is called `UM_FEATHERS2NEO` on [micropython.org/download](https://micropython.org/download/UM_FEATHERS2NEO/). If you're using a different board, flash the firmware for that board instead. Then connect to it from Thonny or Snakie.

## Step 2: install the Modulino library

Arduino's library is one package that covers every Modulino. Run this once, with the board connected to Wi-Fi. It's a `mip` install, as shown in Kev's repository:

<!-- code-check: skip -->
```python
import mip
mip.install("github:arduino/arduino-modulino-mpy")
```

If you use Snakie, drag a Modulino onto the board and it offers to install the library for you, after you approve a prompt. It only installs once, however many modules you add.

This board needs Wi-Fi set up first. The sources for this course don't cover that step, so see the MicroPython documentation at [docs.micropython.org](https://docs.micropython.org/) if you haven't connected yet.

## Step 3: build the bus

Here is the one thing that trips up almost everyone. Arduino's library works out which pins carry I2C by looking up your board in a table of Arduino boards. The FeatherS2 Neo isn't in that table. If you write `ModulinoDistance()` with no arguments, you get:

```
RuntimeError: I2C interface couldn't be determined automatically for '<your board>'
```

Nothing is wrong with your wiring or your module. You just have to tell the library which bus to use. On the FeatherS2 Neo, `SCL` is GPIO 9 and `SDA` is GPIO 8. You build the bus once, and every module shares it:

```python
from machine import I2C, Pin

# The FeatherS2 Neo's I2C pins. Change these two numbers for another board.
SCL_PIN = 9
SDA_PIN = 8

i2c = I2C(0, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=400_000)
print("Bus ready:", i2c)
```

Every Modulino class takes an `i2c_bus` argument. Pass this `i2c` object to each one, and the auto-detect problem disappears. We'll do that in the next lessons.

Using another board? Look up its SCL and SDA pins in its documentation and change those two numbers. Everything else stays the same.

## Step 4: scan the chain

A scan asks every address on the bus "are you there?" and lists the ones that reply. This program is complete: save it and run it.

```python
from machine import I2C, Pin

SCL_PIN = 9
SDA_PIN = 8

# What Modulino SMARS should find, as a scan reports them
EXPECTED = {
    0x29: "Distance",
    0x1E: "Buzzer",
    0x24: "Motors",
    0x39: "LED Matrix",
    0x6A: "Movement",
}

i2c = I2C(0, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=400_000)
found = i2c.scan()

print("Found", len(found), "device(s)")
for addr in sorted(found):
    print("  0x{:02X}  {}".format(addr, EXPECTED.get(addr, "unknown device")))

missing = [name for addr, name in EXPECTED.items() if addr not in found]
if missing:
    print("MISSING:", ", ".join(missing))
    print("Re-seat the Qwiic cable next to the first missing module.")
else:
    print("All five senses present.")
```

You should see five lines and the message `All five senses present.` The Movement module can also appear at `0x6B`, which Kev's own scan script treats as an alternative. If you see `0x6B` instead, that's still the Movement module.

## Read the result

- **Nothing at all:** the problem is power or the first cable, not the modules.
- **Some modules missing:** a Qwiic cable has worked loose. They click when they're home.
- **An unknown address:** something else is on the bus, or you've changed an address.

Kev's repository has a longer `demos/scan.py` that names more Modulinos. The program above does the same job for our five.

## Try it Yourself

1. Unplug the Buzzer cable and run the scan again. Which message appears?
2. Unplug a cable in the middle of the chain. Which modules disappear, and why?
3. Challenge: change the program to print "Movement (alternate address)" if it finds `0x6B`.

## Common Issues

**Problem**: `RuntimeError: I2C interface couldn't be determined automatically`
**Solution**: Build the bus yourself and pass it as `i2c_bus=i2c` to every Modulino.
**Why**: The library only auto-detects Arduino's own boards. It doesn't refuse other boards, it just needs telling.

**Problem**: `mip.install` fails.
**Solution**: Check that the board is connected to Wi-Fi before you run it.
**Why**: `mip` downloads from GitHub, so it needs an internet connection.

**Problem**: The scan finds nothing.
**Solution**: Check the cable from the board to the first module, and check the board is powered.
**Why**: The modules get power and data down that one cable.
