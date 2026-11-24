---
layout: lesson
title: Summary and next steps
author: Kevin McAleer
type: page
cover: /learn/how_to_install_micropython/assets/banners/07_summary.jpg
date: 2024-08-10
previous: 06_update_and_reset.html
description: Recap how to install MicroPython on a Raspberry Pi Pico and where to
  go next in the MicroPython and Pico learning pathways.
percent: 100
duration: 3
quiz_total: 17
redirect_from: /learn/how_to_install_micropython/04_next_steps.html
date_updated: 2026-11-24
navigation:
- name: How to install MicroPython
- content:
  - section: Introduction
    content:
    - name: Introduction
      link: 00_intro.html
  - section: Raspberry Pi Pico
    content:
    - name: Which Pico and which firmware
      link: 02_types.html
    - name: Install MicroPython with Thonny
      link: 03_install.html
    - name: Install by drag and drop
      link: 04_drag_and_drop.html
  - section: Check and maintain
    content:
    - name: Check it worked
      link: 05_check_it_worked.html
    - name: Update or reset your Pico
      link: 06_update_and_reset.html
  - section: Where to go from here
    content:
    - name: Summary and next steps
      link: 07_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to recall the steps for installing MicroPython and choose a course to continue with.

Well done, maker! Your Pico now speaks Python, and you know exactly how it got there. Let's recap, then plan what to build next.

## What you learned

```mermaid
flowchart LR
    A["Know your model"] --> B["Pick the UF2 file"]
    B --> C["BOOTSEL mode"]
    C --> D["Thonny, Snakie<br>or drag and drop"]
    D --> E["Check in the REPL"]
    E --> F["Update or wipe<br>when needed"]
```

- The Pico, Pico H, Pico W and Pico WH use the RP2040. The Pico 2 range uses the RP2350.
- Firmware is board-specific. Pick the page on micropython.org that matches your exact board.
- Hold BOOTSEL as you plug in to make the Pico appear as a USB drive (`RPI-RP2` or `RP2350`).
- Thonny, Snakie and plain drag and drop all do the same thing: put a UF2 file on that drive.
- The REPL banner and `sys.implementation._machine` confirm what you installed.
- Updating is just installing again, and a full wipe means reinstalling afterwards.

## Check your understanding

{% quiz %}
? You hold BOOTSEL and plug in a Pico 2. What does it look like to your computer?
- [ ] A serial port named RPI-RP2
- [x] A USB drive named RP2350
- [ ] Nothing at all
> The Pico 2 range appears as RP2350, and the original Pico range appears as RPI-RP2.

? Which is the safest way to confirm the board name after installing?
- [ ] Look at the colour of the LED
- [x] Read the REPL banner or print `sys.implementation._machine`
- [ ] Open the UF2 file
> Both report the board the firmware was built for.
{% endquiz %}

## Where to go next

- [Getting Started with Snakie](/learn/snakie_getting_started/) shows the site's own IDE, including its firmware flasher, and lets you practise on a simulator.
- [Learn MicroPython - The basics](/learn/micropython/) teaches variables, loops and functions.
- [Raspberry Pi Pico with MicroPython - GPIO Mastery](/learn/micropython_gpio/) covers the pins, so you can start blinking lights and reading buttons.
- [Using the Raspberry Pi Pico's Built-in Temperature Sensor](/learn/pico_temp_sensor/) is a small first project.
- [The MicroPython Learning Pathway](/learn/learning_pathways/micropython) and [The Pico Learning Pathway](/learn/learning_pathways/pico) collect the courses in order.
- [Raspberry Pi Pico Ideas](/ideas/pico) is full of project inspiration.

## Try it Yourself

1. Write down from memory the seven steps for drag and drop installing, then check against lesson 4.
2. Help a friend install MicroPython on their Pico. Teaching is the best way to remember.
3. Challenge: pick one project from the Pico ideas page and list the parts you would need.
