---
title: Summary
description: What you learned in Getting Started with Snakie, and where to go next
layout: lesson
type: page
cover: /learn/snakie_getting_started/assets/banners/11_summary.jpg
date_updated: 2026-11-20
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Well done! You've gone from opening Snakie for the first time to a traffic light that runs by itself on a Raspberry Pi Pico.

---

## What you learned

- Snakie runs in the browser at [app.snakie.org](https://app.snakie.org/) and as a desktop app for Windows, macOS, Linux and Raspberry Pi
- It has four workspaces: **Blocks**, **Code**, **Electronics** and **Build**
- The **Simulated device** lets you run MicroPython with no hardware
- The **REPL** runs one line at a time, which is ideal for testing pins and wiring
- The **Board view** shows the pins your code uses, as you type
- The **Board Finder** identifies your board and the firmware it needs
- The **firmware flasher** puts MicroPython onto a board
- **Run** runs code from memory; saving it to the board as **`main.py`** makes it start at power-up

## Quick reference

| I want to... | Do this |
|---|---|
| Run my program | Press **Run** |
| Stop a program | Press **Stop**, or <kbd>Ctrl</kbd>+<kbd>C</kbd> in the console |
| Restart the board | Press **Reset** |
| Try a single line | Type it at the `>>>` prompt |
| See my pins | Look at the Board view and turn on **Show pin labels** |
| Put MicroPython on a board | **Flash firmware** in the status bar |
| Use a real board | **Disconnect** from the simulator, then connect to the board |
| Make a program run at power-up | Upload it as `/main.py` |
{:class="table table-single"}

## Watch more

See Snakie in action, and hear why Kev built it:

{% include youtubeplayer.html id="qfT_Ndq-tJI" %}

## What's Next

Carry on along the **Start with Snakie** pathway:

- [Snakie Blocks: From Blocks to Python](/learn/snakie_blocks/) - build programs by snapping blocks together, and watch Snakie write the Python beside them
- [Learn MicroPython - The basics](/learn/micropython/) - learn the language itself: variables, loops, functions and modules
- [Raspberry Pi Pico with MicroPython - GPIO Mastery](/learn/micropython_gpio/) - go deeper on pins, PWM, analogue inputs, I2C and SPI

And when you're ready to build something that moves, try [MicroPython Robotics Projects with the Raspberry Pi Pico](/learn/micropython_robotics/).
