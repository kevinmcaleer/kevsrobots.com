---
title: Hardware Blocks - LEDs and Buttons
description: Blink an LED and read a button with Snakie hardware blocks, and understand pins, pull-ups and the forever loop
layout: lesson
type: page
cover: /learn/snakie_blocks/assets/banners/05_leds_and_buttons.jpg
date_updated: 2026-11-20
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Time to leave the screen and control some pins. The LED half of this lesson runs on the simulator. For the button half you'll want a real Pico, because the simulator has no button to press yet.

---

## The Hardware drawer

![The Hardware drawer](/learn/snakie_blocks/assets/toolbox_hardware.jpg){:class="img-fluid w-100"}

Hardware blocks are orange. Each one has a **pin dropdown** such as `GP15`. The dropdown only lists pins your board really has, and only the ones that can do that block's job. So you can't pick a pin that doesn't exist.

## Blink an LED

Three blocks make a blink:

1. From **Control**, drag out a **forever** block.
2. From **Hardware**, put **toggle LED on GP15** inside it.
3. From **Wait**, add **wait 400 milliseconds**.

![The blink program as blocks](/learn/snakie_blocks/assets/blocks_blink.jpg){:class="img-fluid w-100"}

Press **Run** and watch the board view on the right: GP15 flashes. Press **Stop** when you've seen enough.

{% capture snakie_code %}
import time
from machine import Pin
pin_15 = Pin(15, Pin.OUT)
while True:
    pin_15.toggle()
    time.sleep_ms(400)
{% endcapture %}
{% include snakie.html code=snakie_code name="blink.py" view="split" %}

### Reading the Python

- `pin_15 = Pin(15, Pin.OUT)` is the **setup line**. It tells the Pico that GP15 is an output. There's no block for it. Snakie writes it for you the first time a block uses that pin
- `while True:` is **forever**. `True` is always true, so the loop never ends
- `pin_15.toggle()` flips the pin: on becomes off, off becomes on
- `time.sleep_ms(400)` waits 400 milliseconds, which is 0.4 seconds

> **About the grey blocks**: in some screenshots you'll see grey `import` blocks at the top of a program. They appear when a program started life as Python. When you build from the toolbox, Snakie adds the imports to the code without putting blocks on the canvas.

## Read a button

A button is an input. The program asks the pin a question instead of telling it what to do.

Build this:

![The button program as blocks](/learn/snakie_blocks/assets/blocks_button.jpg){:class="img-fluid w-100"}

{% capture snakie_code %}
import time
from machine import Pin
from snakie import Led
pin_14 = Pin(14, Pin.IN, Pin.PULL_UP)
led_15 = Led(pin=Pin(15, Pin.OUT))
while True:
    if pin_14.value() == 0:
        led_15.set(True)
    else:
        led_15.set(False)
    time.sleep_ms(50)
{% endcapture %}
{% include snakie.html code=snakie_code name="button.py" view="split" %}

### Why does pressed mean 0?

The button block says **with pull-up**. A pull-up gently holds the pin at 1 when nothing is happening. The button connects the pin to ground, so pressing it drags the pin down to 0.

| Button | Pin reads |
|---|---|
| Not pressed | 1 |
| Pressed | 0 |
{:class="table table-single"}

> **On the simulator**: a simulated input pin always reads 0, and there's no way to press it yet. So this program behaves as if the button is held down, and the LED stays on. Wire up a real button to see it switch.

That feels backwards the first time. If you'd rather not think about it, use the **button on GP14 is pressed** block instead. It gives you True when the button is down, and writes `not pin_14.value()` in the Python for you.

### Two kinds of LED block

**turn LED on GP15 on** uses a small helper, `Led`, from Snakie's own library. **toggle LED** and **set pin** talk to the pin directly. Both are fine. The helper reads more like English, and the direct way is what you'll see in most MicroPython tutorials.

### The warning badge

Notice the little warning triangles on the two LED blocks. Snakie adds one whenever two blocks use the same pin. Here that's exactly what we want, since both blocks control the same LED. The badge only means "have a look at this", so you can ignore it when the sharing is on purpose.

## On a real Pico

To try this for real, connect an LED (with a 330Ω resistor) between GP15 and GND, and a button between GP14 and GND. Connect to your board as you did in [Getting Started with Snakie](/learn/snakie_getting_started/08_connect_run_stop), then press Run. The same program, no changes.

---

## Try it Yourself

1. Make the LED blink twice as fast.
2. Swap **toggle LED** for the **turn the onboard LED on** blocks so the Pico's own LED blinks.
3. Flip the button program so the LED is on until you press the button.
4. Challenge: make the LED blink quickly while the button is held, and stay off otherwise.

## Common Issues

**Problem**: The LED stays on, or never lights.
**Solution**: Check the **wait** block is inside the forever loop.
**Why**: With no wait, the pin flips thousands of times a second and looks permanently dim or lit.

**Problem**: The button works backwards.
**Solution**: With a pull-up, compare with `0` for pressed. Or use the **button is pressed** block.
**Why**: The pull-up holds the pin at 1 until the button pulls it to ground.

**Problem**: On the simulator the LED is always on.
**Solution**: Try it on a real Pico with a button.
**Why**: The simulator's input pins read 0, which with a pull-up means pressed.

**Problem**: On a real board, nothing happens.
**Solution**: Check the LED is the right way round (long leg towards GP15) and the pin in the block matches your wiring.
**Why**: An LED only lets current through one way.

## Check your understanding

{% quiz %}
? What Python does the "forever" block write?
- [x] `while True:`
- [ ] `for _ in range(forever):`
- [ ] `loop()`
> `while True:` repeats for as long as True is true, which is always.

? With a pull-up, what does a pressed button read?
- [ ] 1
- [x] 0
- [ ] It depends on the LED
> The button connects the pin to ground, so pressed reads 0.

? Where does the line `pin_15 = Pin(15, Pin.OUT)` come from?
- [ ] A setup block you have to add
- [x] Snakie writes it when a block first uses the pin
- [ ] It is built into the Pico
> Setup lines are written for you, in the order the pins are first used.
{% endquiz %}
