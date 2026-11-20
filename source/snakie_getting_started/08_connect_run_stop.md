---
title: Connect, Run and Stop
description: Connect a real MicroPython board to Snakie and control it with Run, Stop and Reset
layout: lesson
type: page
cover: /learn/snakie_getting_started/assets/banners/08_connect_run_stop.jpg
date_updated: 2026-11-20
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Your board has MicroPython on it. Let's get Snakie talking to it and run a program on real hardware.

---

## Leave the simulator

Snakie is still connected to the simulated device, and it only talks to one device at a time. Click **Disconnect**, the button beside the device list above the console.

## Connect your board

1. Plug the board in with a USB data cable. Don't hold BOOTSEL this time.
2. Open the device list above the console.
3. **In the desktop app**, pick your board's serial port from the list and click **Connect**.
   **In the web app**, choose **Connect a USB board…**. The browser shows its own list of devices; pick your board and click **Connect**.

The console wakes up with MicroPython's greeting and a `>>>` prompt, and the status bar at the bottom left changes from *Simulated device* to your board. Snakie also tells you which board it found and shows it in the Board view.

## Run a program

Open a new file and type the classic first hardware program:

{% capture snakie_code %}
from machine import Pin
import time

led = Pin("LED", Pin.OUT)

while True:
    led.toggle()
    print("LED is", led.value())
    time.sleep(0.5)
{% endcapture %}
{% include snakie.html code=snakie_code name="blink.py" %}

Press **Run**. The little LED on your board blinks, and the console prints its state twice a second.

`Pin("LED")` is a handy name that means *the built-in LED, wherever it is on this board*. On a plain Pico that's GP25; on a Pico W it's wired through the WiFi chip. Using the name means the same code works on both.

## Stop it

This program has a `while True:` loop, so it will run forever. Press **Stop**.

Stop does the same job as <kbd>Ctrl</kbd>+<kbd>C</kbd> in the console. The program ends with a `KeyboardInterrupt` message, which is normal, and the `>>>` prompt comes back.

## Reset

When nothing is running, the red button says **Reset**. It soft-resets the board: MicroPython starts again from clean, forgetting every variable. If there's a `main.py` on the board, it runs.

Use Reset when the board seems to be in an odd state, or to check that your saved program starts by itself.

## What Run actually does

It's worth knowing what happens when you press Run, because it explains something that confuses everyone once.

**Run sends the code in the editor to the board and runs it from memory. It doesn't save it on the board.**

So if you unplug the board and power it from a battery, the program is gone. To make a program stay, you need to save it onto the board as a file, and that's the next lesson.

| Button | What it does |
|---|---|
| Run | Runs the editor's code on the board, without saving it there |
| Stop | Interrupts the running program |
| Reset | Soft-resets the board and runs `main.py` if there is one |
{:class="table table-single"}

In the desktop app, Run is <kbd>Ctrl</kbd>+<kbd>R</kbd> and Stop is <kbd>Ctrl</kbd>+<kbd>.</kbd> (use <kbd>⌘</kbd> on a Mac).

---

## Try it Yourself

1. Make the LED blink faster.
2. Stop the program, then type `led.on()` in the console. The variable is still there.
3. Press **Reset**, then type `led.on()` again. What happens, and why?

## Common Issues

**Problem**: My board isn't in the list.
**Solution**: Try a different cable or USB port, and close anything else that may be using the board, such as Thonny or a serial monitor.
**Why**: Only one program can hold a serial port at a time.

**Problem**: The device list won't open.
**Solution**: Press **Disconnect** first.
**Why**: The list is locked while a device is connected.

**Problem**: `NameError: name 'led' isn't defined` after a reset.
**Solution**: Run the program again.
**Why**: Reset wipes the board's memory, so `led` no longer exists.

## Check your understanding

{% quiz %}
? You press Run, then unplug the board and power it from a battery. Does the program run?
- [ ] Yes, Run saves it to the board
- [x] No, Run only runs it from memory
- [ ] Only on a Pico W
> Run doesn't save anything on the board. Save the program as `main.py` to make it stay.

? Why use `Pin("LED")` instead of `Pin(25)`?
- [ ] It's faster
- [x] It works on every Pico, wherever the LED is wired
- [ ] It makes the LED brighter
> The built-in LED is on different pins on different boards. The name finds it for you.
{% endquiz %}
