---
layout: lesson
title: Example 01
author: Kevin McAleer
type: page
cover: /learn/micropython/assets/banners/07_hello_world.jpg
date: 2022-12-04
previous: 06_our_first_program.html
next: 08_variables.html
description: Save and run hello_world.py, and make it start by itself
percent: 45
duration: 4
quiz_total: 57
date_updated: 2026-11-24
navigation:
- name: Learn MicroPython - The basics
- content:
  - section: Overview
    content:
    - name: Introduction
      link: 00_intro.html
    - name: Learn MicroPython Introduction Video
      link: 00_videos.html
  - section: Introduction
    content:
    - name: Why is it called MicroPython?
      link: 01_why_is_it_called_micropython.html
    - name: Where to get MicroPython
      link: 02_where_to_get_micropython.html
    - name: How to Install MicroPython
      link: 03_installing_micropython.html
    - name: Why use MicroPython?
      link: 04_why_use_micropython.html
    - name: Python Development Environments
      link: 05_ides.html
    - name: Our first program
      link: 06_our_first_program.html
    - name: Example 01
      link: 07_hello_world.html
  - section: Variables and Reserved Words
    content:
    - name: Variables and Constants
      link: 08_variables.html
    - name: Example 02
      link: 09_example02.html
    - name: Values & Variables Types
      link: 10_values_and_variable_types.html
    - name: Reserved Words
      link: 11_reserved_words.html
  - section: Controlling the Flow
    content:
    - name: If, elif, else
      link: 12_if_elif_else.html
    - name: Loops
      link: 13_loops.html
    - name: Operators
      link: 14_operators.html
  - section: Functions and Modules
    content:
    - name: Functions
      link: 15_functions.html
    - name: The REPL
      link: 16_repl.html
    - name: Modules
      link: 17_modules.html
  - section: Summary and Review
    content:
    - name: Summary and Review
      link: 18_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to save a program as a file, run it, and explain the difference between running a program and saving it on the Pico.

## Type, save, run

Last lesson we typed a single line. Real programs live in files, so let's make one called `hello_world.py`. The `.py` ending tells your computer it is a Python file.

Type the program below into your MicroPython editor.

{% capture snakie_code %}
# Hello World

print("Hello, world!")
{% endcapture %}
{% include snakie.html code=snakie_code name="hello_world.py" %}

In Thonny, save the file and then run it with the green `run` button. In Snakie, use `Run`.

![A screenshot of the hello world program](assets/hello_world_program.png){:class="img-fluid w-100 shadow-lg"}

![A screenshot of the results in the console from running the Hello World program](assets/hello_world_console.png){:class="img-fluid w-100 shadow-lg"}

## Where does the program live?

When you press `Run`, your editor sends the program to the Pico over the USB cable, and the Pico runs it. The program isn't stored on the Pico. Unplug it, and the program has gone, although the file is still on your computer.

To make a robot that works on its own, you need the program stored on the board. Save it there with the name `main.py`. MicroPython runs a file called `main.py` automatically every time the board powers up.

```mermaid
flowchart TB
    A["Press Run"] --> B["Program runs once, stays on your computer"]
    C["Save to the Pico as main.py"] --> D["Program runs every time the Pico powers up"]
```

In Thonny, choose `File > Save as...` and then pick `Raspberry Pi Pico` as the place to save. To do the same in Snakie, see the lesson [Files, local and on the device](/learn/snakie_getting_started/09_files_local_and_device.html).

> **Teachers**: if boards are shared between classes, a leftover `main.py` will run the next time someone plugs in. Agree a rule for deleting saved files at the end of a lesson, and let learners use `Run` rather than saving to the board until they are building something to keep.

## Try it Yourself

1. Save the program as `hello_world.py` on your computer, run it, then change the message and run it again.
2. Add a second line that prints the name of your robot.
3. Save the program to the Pico as `main.py`, unplug the USB cable, and plug it back in. Does anything happen? (Your program prints to a console, so you need to connect again to see it.)
4. Challenge: how would you see the output from a program that runs by itself?

## Common Issues

- **Problem**: I saved `hello_world.py` but nothing runs when I power the Pico.
- **Solution**: Save the file on the Pico under the name `main.py`.
- **Why**: Only a file called `main.py` is run automatically at power up.

- **Problem**: A program starts running when I connect and I can't get control of the board.
- **Solution**: Press the stop button in your editor, or press `Ctrl + C` in the shell.
- **Why**: A saved `main.py` is already running, and `Ctrl + C` interrupts it.

## Check your understanding

{% quiz %}
? What file name makes MicroPython run a program automatically at power up?
- [ ] `hello_world.py`
- [x] `main.py`
- [ ] `start.txt`
> MicroPython runs `main.py` at power up.

? You press Run, then unplug the Pico. What happens to the program?
- [ ] It is saved on the Pico forever
- [x] It is gone from the Pico, because it was only sent to run
- [ ] It runs by itself on the next power up
> Run sends the program over USB to run. It is only stored if you save it to the board.

? What does the `.py` ending on a file name mean?
- [ ] It is a picture
- [x] It is a Python file
> The ending tells the computer what kind of file it is.
{% endquiz %}
