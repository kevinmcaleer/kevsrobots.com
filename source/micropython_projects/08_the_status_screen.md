---
title: The status screen
description: Join the reed switch and the OLED so the buddy shows the door state, a counter and how long it has been that way
layout: lesson
type: page
cover: /learn/micropython_projects/assets/banners/08_the_status_screen.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

We can sense, and we can draw. Now let's join them. This is **status buddy stage 2**: the screen shows what the door is doing.

---

## Designing the screen

A 128 by 64 screen is small, so plan it. Here is our layout:

```
+--------------------------+
| Status Buddy             |   y = 0     title
|--------------------------|   y = 10    divider line
|  [door icon]  DOOR       |
|               OPEN       |   y = 20..  state in words
|               for 12s    |   y = 44    how long it's been that way
|  Opened: 3               |   y = 56    the counter
+--------------------------+
```

Three things change: the icon, the words, and the timer. The title and the line stay put.

## Counting seconds the MicroPython way

To show "for 12s" we need to know when the door last changed. We store `time.ticks_ms()` at that moment and subtract it later with `time.ticks_diff()`:

```python
secs = time.ticks_diff(time.ticks_ms(), changed_at) // 1000
```
{: .no-snakie }

The Pico has no battery-backed clock, so there is no "real" time of day here. A stopwatch since the last change is exactly what we need anyway.

## Stage 2 program

This is complete. Run it on your Pico and move the magnet.

```python
from machine import Pin, I2C
from ssd1306 import SSD1306_I2C
import time

DEBOUNCE_MS = 50

# --- hardware ---
reed = Pin(15, Pin.IN, Pin.PULL_UP)                      # 0 = magnet near = closed
i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)
oled = SSD1306_I2C(128, 64, i2c)

# --- state ---
changed = False
door_open = reed.value() == 1
open_count = 0
changed_at = time.ticks_ms()


def on_edge(pin):
    global changed
    changed = True


reed.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=on_edge)


def draw_door(x, y, is_open):
    if is_open:
        oled.rect(x, y, 30, 44, 1)
        oled.line(x, y, x + 20, y + 6, 1)
        oled.line(x + 20, y + 6, x + 20, y + 50, 1)
        oled.line(x, y + 44, x + 20, y + 50, 1)
    else:
        oled.fill_rect(x, y, 30, 44, 1)
        oled.ellipse(x + 24, y + 22, 2, 2, 0, True)


def draw_screen():
    secs = time.ticks_diff(time.ticks_ms(), changed_at) // 1000
    oled.fill(0)
    oled.text("Status Buddy", 16, 0)
    oled.hline(0, 10, 128, 1)
    draw_door(4, 16, door_open)
    oled.text("DOOR", 56, 20)
    oled.text("OPEN" if door_open else "CLOSED", 56, 32)
    oled.text("for {}s".format(secs), 56, 44)
    oled.text("Opened: {}".format(open_count), 0, 56)
    oled.show()


draw_screen()
last_draw = time.ticks_ms()

while True:
    if changed:
        changed = False
        time.sleep_ms(DEBOUNCE_MS)
        now_open = reed.value() == 1
        if now_open != door_open:
            door_open = now_open
            changed_at = time.ticks_ms()
            if door_open:
                open_count += 1
            draw_screen()
            last_draw = time.ticks_ms()

    # Redraw once a second so the timer ticks
    if time.ticks_diff(time.ticks_ms(), last_draw) >= 1000:
        draw_screen()
        last_draw = time.ticks_ms()

    time.sleep_ms(10)
```

There is nothing new in the sensing half: it is lesson 5's flag, debounce and compare. The new half is `draw_screen()`, which redraws the *whole* screen every time, from the current state. That is simpler and safer than trying to patch only the part that changed.

> **Tip:** **Show everything from the state.** When the screen is always drawn from variables like `door_open`, it can never disagree with them.

## Try it Yourself

1. Move the title to the centre of the line.
2. Show "!" next to OPEN while the door has been open for more than 30 seconds.
3. Challenge: add a small progress bar under the timer that fills as the door stays open, using `fill_rect()`.

## Common Issues

**Problem**: The timer doesn't update.

**Solution**: Check the one-second redraw block is inside the `while True:` loop and not indented under `if changed:`.

**Why**: Redrawing only on changes would freeze the timer between them.

**Problem**: The screen flickers.

**Solution**: Draw everything, then call `oled.show()` once at the end, as `draw_screen()` does.

**Why**: Each `show()` sends a frame to the screen, so extra calls in the middle show half-finished drawings.

## Check your understanding

{% quiz %}
? Why does `draw_screen()` start with `oled.fill(0)`?
- [ ] To turn the screen off
- [x] To clear the old frame before drawing the new one
- [ ] To reset the I2C bus
> The buffer keeps whatever was drawn before, so each frame starts by wiping it.

? When does the OLED actually change?
- [ ] As soon as you call `oled.text()`
- [x] When you call `oled.show()`
- [ ] Every time the loop repeats
> Drawing commands only edit a buffer in memory. `show()` sends it to the display.

? Which wire pair carries the OLED data in this course?
- [ ] GP0 and GP1
- [x] GP4 (SDA) and GP5 (SCL)
- [ ] GP15 and GND
> GP0 is reserved for SAM's speaker, so the display uses GP4 and GP5.
{% endquiz %}
