---
layout: lesson
title: A Real OLED
author: Kevin McAleer
type: page
cover: /learn/snakie_sprites/assets/banners/12_a_real_oled.jpg
date: 2026-11-21
previous: 11_sprite_files_on_the_board.html
next: 13_summary.html
description: Wire an SSD1306 OLED to a Raspberry Pi Pico and run the same robot face
  on real hardware
percent: 91
duration: 6
quiz_total: 39
date_updated: 2026-11-24
navigation:
- name: Pixel Art and Animation in Snakie
- content:
  - section: Overview
    content:
    - name: Pixel Art and Animation in Snakie
      link: 00_intro.html
    - name: Tiny Screens and Pixels
      link: 01_tiny_screens_and_pixels.html
  - section: Drawing
    content:
    - name: The Sprite Editor
      link: 02_the_sprite_editor.html
    - name: Drawing Your First Sprite
      link: 03_drawing_your_first_sprite.html
    - name: Frames and Animation
      link: 04_frames_and_animation.html
    - name: Saving and Exporting
      link: 05_saving_and_exporting.html
  - section: Sprites in Code
    content:
    - name: Pixels into Bytes
      link: 06_pixels_into_bytes.html
    - name: A Screen in the Simulator
      link: 07_a_screen_in_the_simulator.html
    - name: Putting a Sprite on Screen
      link: 08_putting_a_sprite_on_screen.html
    - name: Animating the Face
      link: 09_animating_the_face.html
    - name: Bigger and Smarter
      link: 10_bigger_and_smarter.html
  - section: Going Further
    content:
    - name: Sprite Files on the Board
      link: 11_sprite_files_on_the_board.html
    - name: A Real OLED
      link: 12_a_real_oled.html
    - name: Summary and What's Next
      link: 13_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to wire an SSD1306 OLED to a Raspberry Pi Pico and run the same robot face on real hardware.

## From pretend to real

Everything so far has run on the simulated device. This lesson moves the face to a real screen. If you don't have the parts yet, read along anyway. The change to your code is tiny, and that's the lesson.

You'll need:

- A Raspberry Pi Pico (any model) with MicroPython on it
- An **SSD1306 OLED**, 128×64, the I2C kind with four pins
- Four jumper wires and a breadboard

---

## Wiring

The display has four pins. Connect them like this:

| OLED pin | Pico pin | Why |
|---|---|---|
| GND | GND | Ground |
| VCC | 3V3(OUT) | Power, 3.3 volts |
| SDA | GP0 | Data |
| SCL | GP1 | Clock |
{:class="table table-single"}

Check the labels on your own display before you connect power. Some boards have VCC and GND the other way round from the picture you're copying.

These are the same pins Snakie's Display instrument shows under its screen: **SDA GP0**, **SCL GP1**, address **0x3C**. If you'd like to plan the wiring on screen first, [Wire It Up - Electronics in Snakie](/learn/snakie_electronics/) shows how.

---

## Connect and install the driver

1. Plug in the Pico and connect to it from the device menu above the console
2. Open the **Packages** panel in the left-hand bar, search for `ssd1306` and install it. That puts the display driver on the board
3. Upload `eye.py` to the real board with right-click, **Upload to board**. The simulated device and your Pico have separate file systems, so the files you uploaded earlier aren't on the Pico

---

## The one-line change

Here's the top of `face.py` as you wrote it:

```python
from simscreen import SimScreen
import eye

screen = SimScreen()
```

And here it is for a real OLED:

```python
from machine import Pin, I2C
from ssd1306 import SSD1306_I2C
import eye

i2c = I2C(0, sda=Pin(0), scl=Pin(1))
screen = SSD1306_I2C(128, 64, i2c)
```

Nothing else changes. `fill`, `blit`, `text`, `fill_rect` and `show` all work the same, because the real driver is a frame buffer with a `show()` too. That's why we built `SimScreen` to match it.

Run `face.py` and your eyes appear on the glass.

---

## One program for both

Editing the top of every file each time you switch gets tiresome. A small helper can choose for you. Make `myscreen.py`:

```python
def make_screen():
    """A real SSD1306 if one is connected, otherwise the simulated screen."""
    try:
        from machine import Pin, I2C
        from ssd1306 import SSD1306_I2C
        return SSD1306_I2C(128, 64, I2C(0, sda=Pin(0), scl=Pin(1)))
    except Exception as problem:
        print("No OLED found (" + str(problem) + "), using the Display instrument")
        from simscreen import SimScreen
        return SimScreen()
```

Upload it, then start your programs like this:

```python
from myscreen import make_screen
import eye

screen = make_screen()
```

`try` attempts the real display. If the driver isn't installed or no screen answers, the `except` part runs and you get the simulated one instead. The same file now works on the simulator, on a bare Pico and on a Pico with an OLED.

Try it on the simulated device first. You'll see the "No OLED found" message in the console, and the face on the Display instrument as before.

---

## Run it without a computer

To make the face start by itself when the robot is switched on:

1. Open `robot_face.py` and change its first lines to use `make_screen`
2. Save a copy named `main.py`
3. Upload `main.py`, `myscreen.py`, `simscreen.py` and `eye.py` to the Pico

MicroPython runs `main.py` automatically at power-up. Unplug the Pico from your computer, power it from a battery pack or a USB charger, and the face wakes up.

> **Teachers**: This lesson is the only one that needs hardware: a Pico with MicroPython and an SSD1306 OLED. If you have only a few sets, run it as a group activity. Learners without the kit can still finish the rest of the course on the simulated device, and the face program falls back to the simulated screen when no OLED answers.

---

## Try it Yourself

1. Run `blink.py` on the real screen
2. Add a real light sensor (a photoresistor and a 10k resistor on GP26) and cover it with your hand to send the robot to sleep
3. Try `screen.invert(1)` after making the screen. The SSD1306 driver can swap light and dark for the whole display
4. Mount the screen on a robot. [Build your own BurgerBot](/learn/burgerbot/) has a good spot for one

---

## Common Issues

**Problem**: `ImportError: no module named 'ssd1306'`.
**Solution**: Install `ssd1306` from the Packages panel while connected to the Pico.
**Why**: The driver isn't part of MicroPython. It has to be copied to each board that uses it.

**Problem**: `OSError: [Errno 5] EIO` or `ETIMEDOUT`.
**Solution**: Check the four wires, especially that SDA and SCL aren't swapped. Then run the **I²C detect** instrument to see whether anything answers at 0x3C.
**Why**: The Pico asked the display to respond and heard nothing.

**Problem**: The picture is there but shifted or wrapped.
**Solution**: Check your display's size. Some small OLEDs are 128×32, in which case use `SSD1306_I2C(128, 32, i2c)` and redo the positions.
**Why**: The driver has to know the real size to send the buffer correctly.

**Problem**: It works from Snakie but not from a battery.
**Solution**: Make sure the program is saved on the Pico as `main.py`, along with every module it imports.
**Why**: Only `main.py` runs at power-up, and it can only import files that are on the board.

---

## Check your understanding

{% quiz %}
? Why does the same drawing code work on the real OLED?
- [ ] Snakie translates it while uploading
- [x] The real driver is a frame buffer with a show() function, like SimScreen
- [ ] OLEDs understand Python
> Both objects offer the same drawing functions, so the code that uses them doesn't need to know which it has.

? What does the try and except in make_screen do?
- [ ] Makes the screen brighter
- [ ] Runs the code twice
- [x] Falls back to the simulated screen if the real one can't be set up
> If anything in the try part fails, the except part runs instead.

? What must a program be called to start by itself when the Pico powers up?
- [x] main.py
- [ ] start.py
- [ ] robot_face.py
> MicroPython looks for main.py and runs it automatically.
{% endquiz %}
