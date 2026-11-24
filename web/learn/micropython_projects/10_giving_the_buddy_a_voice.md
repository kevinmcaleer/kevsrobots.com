---
layout: lesson
title: Giving the buddy a voice
author: Kevin McAleer
type: page
cover: /learn/micropython_projects/assets/banners/10_giving_the_buddy_a_voice.jpg
date: 2026-11-24
previous: 09_meet_sam.html
next: 11_pico_w_and_wifi.html
description: Change SAM's pitch, mouth and throat, then make the status buddy announce
  the door state out loud
percent: 55
duration: 5
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

SAM can speak. Now we pick a voice for the buddy and make it announce the door. This is **status buddy stage 3**.

---

## Changing the voice

SAM has a few settings that change how it sounds. Kev's post lists them:

| Setting | Range | Effect |
|---|---|---|
| `pitch` | 0 to 255 | Higher values give a higher voice |
| `speed` | 0 to 255 | Higher values give slower speech |
| `mouth` | 0 to 255 | The mouth shape, which affects the vowels |
| `throat` | 0 to 255 | The throat shape, which affects the timbre |
{:class="table table-single"}

Try two different characters back to back:

```python
from sam import Sam

sam = Sam(pin=0)

# A normal robot voice
sam.pitch = 64
sam.mouth = 128
sam.throat = 128
sam.say("I am a robot")

# A tiny robot voice
sam.pitch = 200
sam.mouth = 100
sam.throat = 100
sam.say("I am a tiny robot")
```

Every combination sounds a little different, so experiment. The defaults shown in `sam.info()` are a good place to come back to if you get lost.

## Don't talk in the interrupt

Here the golden rule from lesson 5 earns its keep. Speaking takes time: SAM must render the speech before it plays. If we called `sam.say()` from the interrupt handler, we'd be doing a long job in a place that should be instant. So the handler still only sets a flag, and the main loop does the talking after it has debounced.

## Stage 3 program

Here is the status buddy that senses, shows and talks. Run it and open and close the door.

```python
from machine import Pin, I2C
from ssd1306 import SSD1306_I2C
from sam import Sam
import time

DEBOUNCE_MS = 50

# --- hardware ---
reed = Pin(15, Pin.IN, Pin.PULL_UP)                      # 0 = magnet near = closed
i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)
oled = SSD1306_I2C(128, 64, i2c)
sam = Sam(pin=0)

# The buddy's voice
sam.pitch = 64
sam.mouth = 128
sam.throat = 128

# --- state ---
changed = False
door_open = reed.value() == 1
open_count = 0
changed_at = time.ticks_ms()


def on_edge(pin):
    global changed
    changed = True


reed.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=on_edge)


def draw_screen():
    secs = time.ticks_diff(time.ticks_ms(), changed_at) // 1000
    oled.fill(0)
    oled.text("Status Buddy", 16, 0)
    oled.hline(0, 10, 128, 1)
    oled.text("DOOR", 40, 20)
    oled.text("OPEN" if door_open else "CLOSED", 40, 32)
    oled.text("for {}s".format(secs), 40, 44)
    oled.text("Opened: {}".format(open_count), 0, 56)
    oled.show()


draw_screen()
last_draw = time.ticks_ms()
sam.say("Status buddy ready")

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
            draw_screen()                 # show it first...
            last_draw = time.ticks_ms()
            if door_open:                 # ...then say it
                sam.say("The door is open")
            else:
                sam.say("The door is closed")

    if time.ticks_diff(time.ticks_ms(), last_draw) >= 1000:
        draw_screen()
        last_draw = time.ticks_ms()

    time.sleep_ms(10)
```

We draw first and speak second, so the screen updates instantly and doesn't wait for the voice.

## Project ideas for a talking buddy

Kev's SAM post has plenty of ideas you can steal:

- A **button-triggered** phrase, so the buddy can say the status on demand
- An **alert system** that speaks warnings aloud
- A **speaking clock** that announces the time at intervals
- Announcing the Pico W's **IP address** after it connects to Wi-Fi, which is handy when there's no screen

## Try it Yourself

1. Give the buddy a different voice for "open" than for "closed", by changing `sam.pitch` before each `say()`.
2. Make it say how many times the door has opened, using `sam.say("Opened {} times".format(open_count))`.
3. Challenge: make the buddy complain if the door is open for more than a minute.

## Common Issues

**Problem**: The buddy speaks, but the screen is slow to update.

**Solution**: Make sure `draw_screen()` runs *before* `sam.say()`, as in the program above.

**Why**: SAM has to render the speech before it can play it. That is quick with the native renderer, but it is still time the screen would be waiting if you spoke first.

**Problem**: The buddy says the same thing twice on one open.

**Solution**: Check your debounce wait is still there, and that you compare `now_open != door_open` before speaking.

**Why**: A bounce that isn't filtered looks like two separate openings.

## Check your understanding

{% quiz %}
? Which setting makes SAM's voice higher?
- [ ] `speed`
- [x] `pitch`
- [ ] `throat`
> Higher `pitch` values give a higher-pitched voice. Higher `speed` values actually make speech slower.

? Why does the buddy speak from the main loop rather than from the interrupt handler?
- [ ] Interrupt handlers can't use GPIO pins
- [x] Speaking takes time, and handlers should be as quick as possible
- [ ] SAM only works in loops
> The handler just sets a flag. The slower jobs, like rendering speech, happen in the main loop.

? What is the job of the optional 1K resistor?
- [ ] It powers the speaker
- [x] It smooths out pops and clicks
- [ ] It sets the speaker's volume to zero
> Kev suggests a 1K resistor between GP0 and the speaker to reduce pops and clicks.
{% endquiz %}
