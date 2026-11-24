---
layout: lesson
title: OLED wiring and first text
author: Kevin McAleer
type: page
cover: /learn/micropython_projects/assets/banners/06_oled_wiring_and_first_text.jpg
date: 2026-11-24
previous: 05_interrupts_and_debounce.html
next: 07_drawing_shapes_and_icons.html
description: Wire an SSD1306 OLED to the Pico over I2C, find its address, install
  the ssd1306.py driver and say hello
percent: 35
duration: 4
quiz_total: 19
date_updated: 2026-11-24
navigation:
- name: MicroPython Projects - Make It Show, Talk and Connect
- content:
  - section: Getting started
    content:
    - name: Introduction
      link: 00_intro.html
    - name: Meet the status buddy
      link: 01_meet_the_status_buddy.html
    - name: Parts and pin plan
      link: 02_parts_and_pin_plan.html
  - section: Sense
    content:
    - name: Reed switches and magnets
      link: 03_reed_switches_and_magnets.html
    - name: Reading the door
      link: 04_reading_the_door.html
    - name: Interrupts and debouncing
      link: 05_interrupts_and_debounce.html
  - section: Show
    content:
    - name: OLED wiring and first text
      link: 06_oled_wiring_and_first_text.html
    - name: Drawing shapes and icons
      link: 07_drawing_shapes_and_icons.html
    - name: The status screen
      link: 08_the_status_screen.html
  - section: Talk
    content:
    - name: Meet SAM
      link: 09_meet_sam.html
    - name: Giving the buddy a voice
      link: 10_giving_the_buddy_a_voice.html
  - section: Connect
    content:
    - name: Pico W and Wi-Fi
      link: 11_pico_w_and_wifi.html
    - name: Arduino Cloud
      link: 12_arduino_cloud.html
  - section: Shrink it
    content:
    - name: Porting to a XIAO
      link: 13_porting_to_a_xiao.html
  - section: Putting it together
    content:
    - name: The full gadget
      link: 14_the_full_gadget.html
    - name: Troubleshooting
      link: 15_troubleshooting.html
    - name: Summary
      link: 16_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

The buddy can sense the door, but only you and a blinking LED know. Time to give it a face. In this lesson we wire up an OLED screen and make it say hello. This begins the **Show** section.

Kev's OLED video, the one at the top of the course introduction, covers this in detail. It is worth a rewatch before you wire up.

---

## What is an OLED?

**OLED** stands for Organic Light-Emitting Diode. Unlike an LCD, every pixel makes its own light, so there is no backlight. You get crisp contrast, wide viewing angles and very low power, because pixels that are off use nothing.

The common maker version has an **SSD1306** driver chip, 128 by 64 pixels, and talks over **I2C**, which needs only two data wires.

We don't re-teach I2C here. If you want the theory, [Working with I2C and SPI](/learn/i2c_spi/) covers it, including how addresses work.

## Wiring

| OLED pin | Pico pin |
|---|---|
| VCC | 3.3V (pin 36) |
| GND | GND (pin 38) |
| SCL | GP5 (pin 7) |
| SDA | GP4 (pin 6) |
{:class="table table-single table-narrow"}

> **Important:** use the **3.3V** pin, not 5V. Double-check VCC and GND before you power up, because reversed power can damage the display.

These are different pins from the original OLED post, which used GP0 and GP1. We keep GP0 free for SAM's speaker, as the pin plan in lesson 2 explained.

## Find the display's address

Every I2C device has an address. Let's check the Pico can see the screen. Save and run this:

```python
from machine import Pin, I2C

# I2C bus 0 on GP4 (SDA) and GP5 (SCL)
i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)

devices = i2c.scan()

if devices:
    print("I2C devices found:")
    for device in devices:
        print("  Address: 0x{:02x}".format(device))
else:
    print("No I2C devices found. Check your wiring!")
```

You should see `Address: 0x3c`. That is the default for most SSD1306 screens. If yours says `0x3d`, note it down and use it in place of `0x3c` below.

## Install the ssd1306.py driver

MicroPython doesn't include an SSD1306 driver, so we copy one onto the Pico. Kev's OLED post points to this file:

- [ssd1306.py in the oled_display repository](https://github.com/kevinmcaleer/oled_display/blob/main/ssd1306.py)

The MicroPython project also publishes an [SSD1306 driver](https://github.com/micropython/micropython/blob/master/drivers/display/ssd1306.py) in its own repository, which the post lists in its useful links.

Get the file onto the Pico in either of these ways:

- **Thonny**: download `ssd1306.py`, open it in Thonny, then choose **File**, **Save as**, **Raspberry Pi Pico**, and save it with the name `ssd1306.py`
- **mpremote**: download the file, then from the same folder run `mpremote cp ssd1306.py :` which copies it to the Pico's root

To check it worked, type `import ssd1306` in the Thonny shell. No error means success.

## Hello, OLED

```python
from machine import Pin, I2C
from ssd1306 import SSD1306_I2C

i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)

# 128 x 64 pixels, default I2C address 0x3c
oled = SSD1306_I2C(128, 64, i2c)

oled.fill(0)                       # clear: 0 is off, 1 is on
oled.text("Hello World!", 0, 0)    # text at x=0, y=0
oled.text("Status buddy", 0, 20)
oled.text("is awake", 0, 40)
oled.show()                        # push the buffer to the screen
```

Three calls do the work:

1. `oled.fill(0)` clears the buffer
2. `oled.text("...", x, y)` draws text with its top-left corner at pixel (x, y)
3. `oled.show()` sends the buffer to the screen

> **Remember:** drawing only changes a buffer in memory. **Nothing appears until you call `oled.show()`.**

The built-in font is 8 pixels wide and 8 tall, so a 128-pixel screen fits 16 characters per line and 8 lines.

## Try it Yourself

1. Put your own name on the screen.
2. Make a three-line menu using y positions 0, 16 and 32.
3. Challenge: centre a word. Its width is 8 pixels times the number of letters.

## Common Issues

**Problem**: The scan finds nothing.

**Solution**: Check SDA and SCL aren't swapped, and VCC and GND are right. Make sure your code says `scl=Pin(5), sda=Pin(4)`.

**Why**: I2C needs both wires on the right pins, and a swapped pair simply looks like an empty bus.

**Problem**: `ImportError: no module named 'ssd1306'`.

**Solution**: The driver file isn't on the Pico. Copy `ssd1306.py` to its root and try again.

**Why**: MicroPython can only import files on the Pico itself, not on your computer.

**Problem**: The screen stays blank even though the scan works.

**Solution**: Add `oled.show()` after drawing.

**Why**: Drawing only changes the buffer.
