---
layout: lesson
title: Install MicroPython
author: Kevin McAleer
type: page
cover: /learn/how_to_install_micropython/assets/cover.jpg
date: 2024-08-10
previous: 02_types.html
next: 04_next_steps.html
description: 'Learn how to install MicroPython on your Raspberry Pi Pico '
percent: 75
duration: 3
navigation:
- name: How to install MicroPython
- content:
  - section: Introduction
    content:
    - name: Introduction
      link: 01_intro.html
  - section: Raspberry Pi Pico
    content:
    - name: Types of Pico
      link: 02_types.html
    - name: Install MicroPython
      link: 03_install.html
  - section: Where to go from here
    content:
    - name: Next Steps
      link: 04_next_steps.html
---


## How to install MicroPython on your Raspberry Pi Pico

The simplest way to install MicroPython is by using a software tool such as Thonny (<https://thonny.org>). Thonny is a Python Integrated Development Environment (IDE) that makes it easy to write and run Python code on your Raspberry Pi Pico. It also enables us to select from a range of MicroPython firmware versions to install on the Pico.

---

### Steps to install MicroPython on your Raspberry Pi Pico

1. Download and install `Thonny` from <https://thonny.org>.

1. Hold down the `boot` button on your Raspberry Pi Pico and then connect it to your computer using a USB cable.

1. Release the `boot` button. This will put the Pico into `bootloader mode`, and it will appear on your computer as a USB drive called `RPI-RP2`.

1. Open `Thonny` and select your Pico from the list of devices, from the bottom right of the screen. (It might say `Local Python 3 * Thonny's Python` until you click it).

1. Select the version of MicroPython you want to install and click `Install`. If you are not sure which version you need, first check the type of Pico you have using the picture in the [previous lesson](02_types#different-types-of-pico). 

    You can then select the correct `variant` of MicroPython from the list. The `Raspberry Pi * Pico / Pico H'` versions are for the original Pico and Pico H. The `Raspberry Pi * Pico W / Pico WH'` versions are for the Pico W and Pico WH. The `Raspberry Pi * Pico 2'` version is for the Pico 2. These appear under the `MOST POPULAR` section.

1. The `Version` dropdown is there if you want to install an older version of MicroPython, its probably best to install the latest version unless you have a specific reason to install an older version.

1. Click on `Install MicroPython`

1. MicroPython will then install and tell you when it is complete.

1. You can now write and run MicroPython code on your Raspberry Pi Pico. **Note** you may have to click the `Stop` button at the top of the Thonny window to reset the Pico and run your code.

Congratulations! You have successfully installed MicroPython on your Raspberry Pi Pico.

---

> ## UF2 files
>
> UF2 files are a type of file format that can be used to flash firmware onto microcontrollers. They are easy to use and are supported by a wide range of development tools. When we select the options from Thonny the specific UF2 file is downloaded and installed onto the Pico for us. You can do this manually by visiting <https://www.micropython.org> and downloading the UF2 file for your Pico, then drag and dropping the UF2 file onto the drive when it is in Bootloader mode (see step 2 above).
>
> ---
>
> ## Variants
>
> Variants are different builds of MicroPython that are optimized for different types of microcontrollers. For example, there are different variants of MicroPython for the Raspberry Pi Pico, the ESP32 and the Adafruit Feather M0 Express. Each variant is designed to work with a specific type of microcontroller and may include additional features or optimizations.

---
