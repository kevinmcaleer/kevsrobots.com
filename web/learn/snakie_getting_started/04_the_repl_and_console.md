---
layout: lesson
title: The REPL and the Console
author: Kevin McAleer
type: page
cover: /learn/snakie_getting_started/assets/banners/04_the_repl_and_console.jpg
date: 2026-11-20
previous: 03_first_program_on_the_simulator.html
next: 05_seeing_your_pins.html
description: Use the MicroPython REPL in Snakie to try out code one line at a time
percent: 40
duration: 4
quiz_total: 23
date_updated: 2026-11-24
navigation:
- name: Getting Started with Snakie
- content:
  - section: Overview
    content:
    - name: Getting Started with Snakie
      link: 00_intro.html
    - name: Web App or Desktop App
      link: 01_web_or_desktop.html
    - name: A Tour of the Workspaces
      link: 02_tour_of_the_workspaces.html
  - section: Your First Programs
    content:
    - name: Your First Program on the Simulator
      link: 03_first_program_on_the_simulator.html
    - name: The REPL and the Console
      link: 04_the_repl_and_console.html
    - name: Seeing Your Pins
      link: 05_seeing_your_pins.html
  - section: Real Hardware
    content:
    - name: Finding Your Board
      link: 06_finding_your_board.html
    - name: Flashing MicroPython
      link: 07_flashing_micropython.html
    - name: Connect, Run and Stop
      link: 08_connect_run_stop.html
    - name: Files on Your Computer and on the Board
      link: 09_files_local_and_device.html
  - section: Project
    content:
    - name: Project - Traffic Light
      link: 10_project_traffic_light.html
    - name: Summary
      link: 11_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to type MicroPython into the REPL, one line at a time, and use it to check your own programs.

The black panel at the bottom of Snakie is more than a place for output. It's a direct line to MicroPython, and it's one of the best tools you have for learning and for debugging robots.

---

## What is the REPL?

REPL stands for **Read, Evaluate, Print, Loop**. MicroPython reads a line you type, works it out, prints the answer, and waits for the next line. The `>>>` prompt means it's ready.

Nothing you type here is saved. That makes it the perfect place to ask quick questions: does this pin work? What does this function return?

## Try it

Click in the console and type each line, pressing <kbd>Enter</kbd> after each one:

```python
>>> 2 + 2
4
>>> from machine import Pin
>>> led = Pin("LED", Pin.OUT)
>>> led.on()
>>> led.value()
1
```

![Typing into the REPL](/learn/snakie_getting_started/assets/repl.jpg){:class="img-fluid w-75"}

You just switched on the board's built-in LED by hand, then asked the pin for its value. `1` means on. Type `led.off()` and ask again.

This is a great way to test wiring on a real robot. Before you write a whole program, poke each motor and sensor from the REPL and see it respond.

## Useful keys

| Keys | What they do |
|---|---|
| <kbd>Ctrl</kbd>+<kbd>C</kbd> | Interrupt a running program |
| <kbd>Ctrl</kbd>+<kbd>D</kbd> | Soft reboot the board, which runs `main.py` again |
| <kbd>Up</kbd> | Bring back the last line you typed |
| <kbd>Tab</kbd> | Complete a name, or list the options |
{:class="table table-single"}

Try the last one: type `led.` and press <kbd>Tab</kbd>. MicroPython lists everything a pin can do.

## Ask MicroPython for help

The REPL has help built in:

```python
>>> help()
>>> import machine
>>> help(machine)
```

`help(machine)` lists everything in the `machine` module, which is where `Pin`, `PWM`, `I2C` and friends live.

## Around the console

The strip above the console has a few controls worth knowing:

- **Console** and **Problems** switch between the REPL and a list of problems found in your code
- The bin clears the console
- The device list shows what you're connected to, with **Connect** or **Disconnect** beside it
- The pop-out button at the top right of the console moves it into its own window, which is handy on a second screen

## The REPL and your programs

When you press **Run**, the names your program made are still there afterwards. Run a program that contains `speed = 50`, and then type `speed` at the prompt. You'll get `50` back. You can inspect what your program left behind, which is very useful when something isn't behaving.

The **Inspect** panel in the side bar shows the same thing as a list of the variables currently on the board.

---

## Try it Yourself

1. At the prompt, work out how many seconds there are in a day.
2. Make a variable called `name` holding your name, then type `print("Hello", name)`.
3. Challenge: use `help()` on `time` after importing it. Which function waits in milliseconds?

## Common Issues

- **Problem**: Typing does nothing.
- **Solution**: Click inside the console first so it has the cursor.
- **Why**: The editor above keeps the keyboard until you click elsewhere.

- **Problem**: The prompt shows `...` instead of `>>>`.
- **Solution**: Press <kbd>Enter</kbd> on an empty line, or <kbd>Ctrl</kbd>+<kbd>C</kbd> to cancel.
- **Why**: MicroPython thinks you're in the middle of a block, such as a loop, and is waiting for more.

## Check your understanding

{% quiz %}
? What does the `>>>` prompt mean?
- [ ] The program has crashed
- [x] MicroPython is ready for you to type a line
- [ ] The board is disconnected
> The prompt shows the REPL is waiting for input.

? Which keys interrupt a program that is running?
- [x] Ctrl+C
- [ ] Ctrl+D
- [ ] Tab
> Ctrl+C interrupts. Ctrl+D does a soft reboot.
{% endquiz %}
