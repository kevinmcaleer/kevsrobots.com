---
layout: lesson
title: Summary and Next Steps
author: Kevin McAleer
type: page
cover: assets/10.jpg
date: 2025-06-16
previous: 09_blinking_led.html
description: Review what you learned about C and the Raspberry Pi Pico, and find where
  to go next.
percent: 100
duration: 4
quiz_total: 30
date_updated: 2026-11-24
navigation:
- name: Getting Started with C on the Raspberry Pi Pico
- content:
  - section: Introduction
    content:
    - name: Introduction to Programming the Raspberry Pi Pico in C
      link: 00_intro.html
  - section: Programming Fundamentals
    content:
    - name: What is C?
      link: 02_what_is_c.html
    - name: Variables and Data Types
      link: 03_variables_and_types.html
    - name: Conditionals and Loops
      link: 04_conditionals_and_loops.html
    - name: Functions in C
      link: 05_functions.html
  - section: Raspberry Pi Pico Setup
    content:
    - name: Setting Up the Toolchain
      link: 06_setting_up_toolchain.html
    - name: Your First Program in C
      link: 07_first_program.html
  - section: GPIO Basics
    content:
    - name: GPIO Basics
      link: 08_gpio_basics.html
    - name: Blinking an LED
      link: 09_blinking_led.html
  - section: Summary and Next Steps
    content:
    - name: Summary and Next Steps
      link: 10_summary.html
---


![Cover]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to list what you can now do in C on the Pico, and choose what to learn next.

Congratulations, makers! You started with a compiler and a blank file, and finished with a program of your own making a real LED flash. That's the same loop of write, build, flash and check that professional firmware engineers use every day.

---

## What you've learned

```mermaid
flowchart LR
    A["C basics<br/>variables, if, loops,<br/>functions"] --> B["Toolchain<br/>SDK + CMake"]
    B --> C["First program<br/>printf over USB"]
    C --> D["GPIO<br/>gpio_init, gpio_set_dir,<br/>gpio_put"]
    D --> E["Blinking LED<br/>project"]
```

- Variables, types, and the integer-division and overflow traps
- Decisions with `if`, loops with `for` and `while`, and your own functions
- Installing the Pico SDK and building with CMake into a `.uf2` file
- Writing a `CMakeLists.txt`, and printing over USB with `pico_enable_stdio_usb`
- Controlling a GPIO pin from C, and wiring an LED with a resistor

## Quick reference

| You want to | Use |
|-------------|-----|
| Print a message | `printf("...\n");` with `pico_enable_stdio_usb` |
| Wait | `sleep_ms(500);` |
| Prepare a pin | `gpio_init(pin);` |
| Make a pin an output | `gpio_set_dir(pin, GPIO_OUT);` |
| Set a pin high or low | `gpio_put(pin, 1);` or `gpio_put(pin, 0);` |
| Build for a Pico 2 | `cmake -DPICO_BOARD=pico2 ..` |
{:class="table table-single"}

## Where to go next

**More C.** Next in C are arrays, structs and pointers, then interrupts and timers. The Pico SDK documentation and Raspberry Pi's "Getting started with Raspberry Pi Pico-series" guide cover them.

**More hardware.** Try reading a button with `gpio_get`, or driving a servo or buzzer with PWM from the SDK's `hardware_pwm` library. For the electronics behind what you just wired, try [Electronics Basics](/learn/electronics_basics/).

**Compare with MicroPython.** The same LED takes a handful of lines in MicroPython. Try the [MicroPython course](/learn/micropython/) and [MicroPython GPIO](/learn/micropython_gpio/) and see how the two languages feel. If you want to connect sensors, [I2C and SPI](/learn/i2c_spi/) and the [Pico temperature sensor course](/learn/pico_temp_sensor/) are good next steps.

**Explore online.**

- [Raspberry Pi Pico C/C++ SDK documentation](https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html)
- [Pico SDK reference (Doxygen)](https://raspberrypi.github.io/pico-sdk-doxygen/)
- [Pico examples on GitHub](https://github.com/raspberrypi/pico-examples), which has dozens of small, working programs for every feature

## Final thoughts

Keep one habit from this course: build small, test often. A blinking LED proves your whole chain works, the compiler, the SDK, the cable and the wiring, which is exactly what you want before adding a motor or sensor.

Keep building, and share what you make!

## Try it Yourself

1. Combine everything: write a program with a function that blinks a number of times based on a value you pass in, and print that value.
2. Wire a second LED and blink the two in alternation.
3. Open `pico-examples` and read the `gpio` folder. Build one example and work out what it does.
4. Challenge: pick one project, such as a traffic light with three LEDs, and sketch the wiring and the functions before you write any code.

## Check your understanding

{% quiz %}
? Which CMake function makes a `.uf2` file?
- [ ] `pico_sdk_init`
- [x] `pico_add_extra_outputs`
- [ ] `target_link_libraries`
> `pico_add_extra_outputs` creates the extra output files including `.uf2`.

? What is the usual order of the GPIO calls to blink an LED?
- [x] `gpio_init`, `gpio_set_dir`, then `gpio_put` in a loop
- [ ] `gpio_put`, then `gpio_init`
- [ ] `sleep_ms` only
> The pin must be prepared and set as an output before you set it high and low.

? Which flag tells CMake you have a Pico 2?
- [ ] `-DPICO_SDK_PATH`
- [x] `-DPICO_BOARD=pico2`
- [ ] `-DPICO_UF2`
> `PICO_BOARD` selects the board.
{% endquiz %}
