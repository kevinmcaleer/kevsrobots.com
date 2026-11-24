---
layout: lesson
title: Your First Program on the Simulator
author: Kevin McAleer
type: page
cover: /learn/snakie_getting_started/assets/banners/03_first_program_on_the_simulator.jpg
date: 2026-11-20
previous: 02_tour_of_the_workspaces.html
next: 04_the_repl_and_console.html
description: Write and run your first MicroPython program in Snakie using the built-in
  simulator, with no hardware
percent: 32
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

By the end of this lesson you will be able to write, run and stop a MicroPython program on the simulated device.

Time to write some code. You don't need a board for this lesson, because Snakie has one built in.

---

## The simulated device

When Snakie opens, it connects to the **Simulated device (offline)**. This is real MicroPython, running inside Snakie instead of on a chip. You can see it in the status bar at the bottom left, and in the device list above the console.

It behaves like a real board: it runs your programs, it has pins you can switch on and off, and it even has its own little filesystem. The one difference is that it forgets everything when you disconnect.

## Write the program

1. Click the **New file** button in the toolbar. A tab called `untitled-1.py` opens.
2. Type this one line:

```python
print("Hello from Snakie!")
```

3. Press the green **Run** button.

Look at the **Console** at the bottom. Your message appears after the `>>>` prompt.

![Hello from Snakie in the console](/learn/snakie_getting_started/assets/first_program.jpg){:class="img-fluid w-100"}

`print()` shows whatever you put between the brackets. Text goes inside quote marks; that's called a *string*.

## Try it right here

You can run the same program in this page. Press **Run**, then change the message and run it again.

{% capture snakie_code %}
print("Hello from Snakie!")
{% endcapture %}
{% include snakie.html code=snakie_code name="hello.py" %}

## A program with a few more lines

Robots repeat things, so let's count. Replace your code with this:

{% capture snakie_code %}
import time

for count in range(5, 0, -1):
    print("Launching in", count)
    time.sleep(1)

print("Lift off!")
{% endcapture %}
{% include snakie.html code=snakie_code name="countdown.py" %}

Here's what each line does:

- `import time` brings in MicroPython's timing tools
- `for count in range(5, 0, -1):` repeats the indented lines with `count` set to 5, 4, 3, 2, then 1
- `time.sleep(1)` waits one second
- The last line isn't indented, so it runs once, after the loop has finished

## Run, Stop and the keyboard

While a program is running, the red button says **Stop**. Press it to end the program early. On the simulator, stopping also restarts the simulated board, and the console says so.

In the desktop app you can press <kbd>Ctrl</kbd>+<kbd>R</kbd> to run (<kbd>⌘</kbd>+<kbd>R</kbd> on a Mac).

---

> **Teachers**: This lesson needs no hardware at all, so a whole class can start at once. The simulated board forgets everything when you disconnect, so on shared machines ask learners to save their file with the **Save** button before they finish.

## Try it Yourself

1. Change the countdown to start from 10.
2. Make it count twice as fast.
3. Challenge: make it count *up* from 1 to 5 instead. You'll need to change all three numbers in `range()`.

## Common Issues

- **Problem**: `SyntaxError: invalid syntax`
- **Solution**: Check that every `(` has a `)`, every `"` has a partner, and the `for` line ends with a colon.
- **Why**: Python needs these to know where things start and end.

- **Problem**: `IndentationError`
- **Solution**: The lines inside the loop must all start with the same four spaces.
- **Why**: Indentation is how Python knows which lines belong to the loop.

## Check your understanding

{% quiz %}
? Where does the output of `print()` appear in Snakie?
- [ ] In a pop-up window
- [x] In the Console
- [ ] In the status bar
> The Console at the bottom shows everything your program prints.

? What is the Simulated device?
- [ ] A cloud service that needs an account
- [x] MicroPython running inside Snakie, with no hardware
- [ ] A video of a board
> It is real MicroPython, running in the app, so you can practise with no board.
{% endquiz %}
