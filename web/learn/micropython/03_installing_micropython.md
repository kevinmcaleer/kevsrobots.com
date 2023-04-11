---
layout: lesson
title: How to Install MicroPython
author: Kevin McAleer
type: lesson
cover: /learn/micropython/assets/micropython.jpg
previous: 02_where_to_get_micropython.html
next: 04_why_use_micropython.html
description: Easy Install with Thonny
percent: 25
duration: 2
thanks: false
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


![Cover photo of a laptop with code on it](assets/how_install.jpg){:class="cover"}

To make installing MicroPython easier you can use `Thonny`, an Integrated Development Environment (IDE) for Python and MicroPython. To download Thonny:

1. Go <https://www.thonny.org> and download the version your computer (`Windows`, `Mac` or `Linux`).

![Screenshot of the Thonny download page](assets/thonny.png){:class="img-fluid w-50 shadow-lg"}

Screenshot of the Thonny download page
{:.caption}

---

## Change to Standard Mode

1. After you have started `Thonny`, make sure you're in the `regular mode` by clicking on the purple text link - it will recommend that you **quit** and then **restart** `Thonny`
1. Go ahead and `quit` and `restart` Thonny.

![Screenshot of the standard mode link](assets/standard_mode.png){:class="img-fluid w-75 shadow-lg my-3"}

Screenshot of the standard mode link
{:.caption}

---

## Installing MicroPython

1. Click on on the bottom right corner and select `Install MicroPython...`

![Screenshot of the Thonny Install dialog option](assets/thonny_install.png){:class="img-fluid w-75 shadow-lg my-3"}

Screenshot of the Thonny Install dialog option
{:.caption}

---

![Screenshot of the Thonny Install dialog box](assets/install_dialog.png){:class="img-fluid w-75 shadow-lg my-3"}

Screenshot of the Thonny Install dialog box
{:.caption}

---

1. Select variant (the version specific to your board), for example the version for the Raspberry Pi Pico is `Raspberry Pi - Pico / Pico H`.
1. Then click the `Install` button. The firmware will now be uploaded to your microcontroller.

---

## Well done

Congratulations, you now have the most up-to-date MicroPython installed on your MicroController.

---
