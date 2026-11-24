---
title: Porting to a XIAO
description: Meet the Seeed XIAO family, move every pin number into config.py and check the buddy on a tiny new board
layout: lesson
type: page
cover: /learn/micropython_projects/assets/banners/13_porting_to_a_xiao.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

The status buddy works. But a Pico plus a breadboard is not exactly pocket-sized. In this lesson we **shrink it**: we look at the Seeed Studio XIAO family and set the buddy up so that moving it to a new board is a ten-minute job, not a rewrite.

Kev's video shows the XIAO boards in action, including a four-wheeled robot called Grover:

{% include youtubeplayer.html id="VBVsachP54s" %}

---

## What is a XIAO?

XIAO is pronounced "see-ow". It is a family of tiny boards from Seeed Studio, about 20mm by 17.5mm. The [official XIAO page](https://www.seeedstudio.com/XIAO-Dev-Boards-c-2477.html) lists them all. Kev's post goes through the ones he has tried:

| Board | What Kev says about it |
|---|---|
| Seeeduino XIAO (SAMD21) | Can technically run MicroPython, but has little memory left for programs. "Yes it can, but you shouldn't" |
| XIAO RP2040 | The same processor as the Pico, with 264KB of SRAM and 2MB of flash. Much more suited to MicroPython |
| XIAO RP2350 | The newest, with 512KB of SRAM and 4MB of flash. The same chip family as the Pico 2. Kev uses it in his robot Grover |
| XIAO nRF52840 | Built-in Bluetooth Low Energy and NFC. Good for low-power wireless |
| XIAO ESP32-S3 | The flagship, with Wi-Fi and Bluetooth built in |
| XIAO ESP32-S3 Sense | Adds more sensors and a camera, and the camera works with MicroPython |
{:class="table table-single"}

For this course, the **RP2040** and **RP2350** versions are the closest to a Pico. And because the buddy needs Wi-Fi for its cloud stage, the **ESP32-S3** is the XIAO that can do everything.

## The XIAO Expansion Board

Kev also tried the XIAO **Expansion Board**. It gives the XIAO easy connections to other parts, with digital and analog pins, I2C, SPI and UART. It has a built-in voltage regulator, so you can power it all from a single source such as a LiPo battery. It also has four **Grove** connectors for plug-in sensors and actuators.

Unlike Qwiic or STEMMA QT, Grove mixes digital and analog signals on its connectors, which makes it flexible for robots.

## What changes when we port?

Think about what the buddy's program actually depends on:

| Thing the program needs | Changes between boards? |
|---|---|
| The MicroPython language and the `machine` module | No |
| `ssd1306.py` and the SAM library | No, they are just files you copy over |
| **Pin numbers** | **Yes. This is the big one** |
| Wi-Fi and Arduino Cloud | Only on boards that have a radio |
{:class="table table-single"}

So the answer is to put every pin number in one place.

## Meet config.py

Create a file called `config.py` and save it on the Pico. These are the values we have used all course:

```python
# config.py - the only file you edit to move the buddy to a new board

REED_PIN = 15        # reed switch, wired to GND
I2C_ID = 0           # which I2C bus the OLED is on
SDA_PIN = 4          # OLED data
SCL_PIN = 5          # OLED clock
OLED_ADDR = 0x3C     # from the I2C scan in lesson 6
SPEAKER_PIN = 0      # SAM's speaker
LED_PIN = "LED"      # onboard LED name on a Pico W. Use None for no LED

USE_CLOUD = False    # True needs Wi-Fi hardware and the keys in secrets.py
DEBOUNCE_MS = 50
```

That is all the hardware knowledge in one tiny file. The main program, in lesson 14, only ever says `config.REED_PIN` and never a bare `15`.

## Porting step by step

1. **Install MicroPython** on the new board, using the firmware for that exact board.
2. **Copy the files across**: `ssd1306.py`, the SAM library and `config.py`. They are ordinary files, so Thonny or `mpremote` work as before.
3. **Look up the pinout** on the board's page. MicroPython's `Pin()` takes the chip's GPIO **number**, not the silkscreen label (such as D1), so find the GPIO number behind each label you want to use.
4. **Edit `config.py`** with the new numbers for the reed switch, the OLED's SDA and SCL, and the speaker.
5. **Run the port check** below to prove each part, one at a time.
6. **Set `USE_CLOUD`** to `True` only on a board with Wi-Fi, such as the ESP32-S3, with `secrets.py` and the cloud client copied across as in lesson 12.

A word of honesty: Kev's XIAO post doesn't give pin numbers or the SAM library's behaviour on every XIAO, so you will need to check the pinout yourself. His SAM post does say that on an ESP32, SAM uses a timer interrupt in place of the Pico's PIO. It works, but it uses more of the CPU.

## The port check

This complete program reads every part of the config and reports what it finds. Run it before the full gadget.

```python
from machine import Pin, I2C
import time
import config

print("Port check")

# 1. The OLED should show up on the I2C bus
i2c = I2C(config.I2C_ID, scl=Pin(config.SCL_PIN), sda=Pin(config.SDA_PIN), freq=400000)
found = i2c.scan()
print("I2C devices:", [hex(a) for a in found])
if config.OLED_ADDR in found:
    print("OLED found")
else:
    print("OLED NOT found: check SDA_PIN, SCL_PIN and I2C_ID")

# 2. The reed switch: move the magnet in the next 10 seconds
reed = Pin(config.REED_PIN, Pin.IN, Pin.PULL_UP)
last = reed.value()
print("Reed starts at", last)
for _ in range(100):
    now = reed.value()
    if now != last:
        print("Reed changed to", now)
        last = now
    time.sleep(0.1)

print("Port check finished")
```

You want to see "OLED found", and you want a "Reed changed" message each time you move the magnet. Once both pass, the same `main.py` from lesson 14 should run on the new board.

## Try it Yourself

1. Create a second file called `config_pico.py` that is a copy of your Pico's settings, so you can switch back.
2. Think about which of your *other* projects would be easier to move if their pins were in one file.
3. Challenge: Kev's Grover robot uses a XIAO RP2350 with an ultrasonic sensor and an IMU. What would its `config.py` look like?

## Common Issues

**Problem**: `ValueError` when creating the I2C bus.

**Solution**: Check the pinout. The pins you chose may not belong to the I2C bus number in `I2C_ID`. If it still fails, `SoftI2C(scl=Pin(...), sda=Pin(...))` from `machine` works on any pins.

**Why**: Hardware I2C is tied to certain pins on each chip, while `SoftI2C` uses software to do the same job on any pins.

**Problem**: `Pin("LED")` fails on the new board.

**Solution**: Set `LED_PIN = None` in `config.py`, or use the GPIO number of the board's user LED.

**Why**: The name `"LED"` exists on the Pico W, but other boards name (or wire) their LED differently.

## Check your understanding

{% quiz %}
? Why does the buddy keep every pin number in `config.py`?
- [x] So porting to another board means editing one file
- [ ] Because MicroPython requires a config file
- [ ] Because pin numbers can't be written in `main.py`
> Pin numbers are the main thing that changes between boards, so keeping them together makes a port quick.

? Which XIAO can run the Wi-Fi and Arduino Cloud stage?
- [ ] XIAO RP2040
- [ ] XIAO RP2350
- [x] XIAO ESP32-S3
> The ESP32-S3 has Wi-Fi and Bluetooth built in. The RP2040 and RP2350 versions don't have a radio.
{% endquiz %}
