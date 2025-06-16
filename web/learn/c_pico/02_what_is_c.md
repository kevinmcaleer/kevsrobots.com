---
layout: lesson
title: What is C?
author: Kevin McAleer
type: page
cover: assets/2.jpg
date: 2025-06-16
previous: 01_intro.html
next: 03_variables_and_types.html
description: "Learn what the C programming language is, where it\u2019s used, and\
  \ why it\u2019s a great choice for embedded systems like the Raspberry Pi Pico."
percent: 20
duration: 4
date_updated: 2025-06-15
navigation:
- name: Getting Started with C on the Raspberry Pi Pico
- content:
  - section: Introduction
    content:
    - name: Introduction to Programming the Raspberry Pi Pico in C
      link: 01_intro.html
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

`C` is one of the oldest and most widely-used programming languages.  
It’s fast, powerful, and gives you direct control over the hardware — which makes it perfect for microcontrollers like the **Raspberry Pi Pico**.

---

## What is C?

C is a **procedural programming language** developed in the early 1970s.

Key features:

- Compiled (not interpreted)
- Low-level (close to the machine, but easier than assembly)
- Portable (runs on many platforms)
- Efficient and fast
- Foundation for many modern languages (like C++, Python, and Java)

> ## Origins of 'C'
>
> The name "C" comes from an earlier language called "B", which was itself derived from the language "BCPL". C was developed by Dennis Ritchie at Bell Labs in 1972.
> C was designed to be a **system programming language** for writing operating systems and low-level applications, but it quickly became popular for all kinds of software development.
> C is still widely used today, especially in embedded systems, operating systems, and performance-critical applications, and languages such as Processing, Arduino, and even Python have roots in C.
> C is often called the "mother of all programming languages" because it has influenced so many others.
> It's a procedural language, meaning it focuses on functions and procedures to operate on data, it was later extended with object-oriented features in C++.

---

## Where is C Used?

C is used in:

- Operating systems (Linux, Windows, macOS internals)
- Embedded systems (Arduino, Raspberry Pi Pico)
- Game engines
- Robotics
- Device drivers and firmware

Anywhere speed and control matter — you’ll likely find C.

---

## Why Use C on the Pico?

While you can use Python (via MicroPython) on the Pico, C has some big advantages:

| MicroPython        | C (with Pico SDK)              |
|--------------------|--------------------------------|
| Easy to learn      | More powerful & efficient      |
| Slower runtime     | Much faster execution          |
| Limited features   | Full access to microcontroller |
| Good for beginners | Better for production projects |
{:class="table table-striped"}

With C, you get **full control of the hardware**, **faster code**, and **better use of memory** — essential for building real-world embedded systems.

---

## What Does C Code Look Like?

Here’s a tiny example:

```c
#include <stdio.h>

int main() {
    printf("Hello, world!\n");
    return 0;
}
```

This program prints a message to the terminal. Don’t worry about the syntax yet — we’ll explain everything step-by-step in future lessons.

Notice the '`{ }`' brackets? They define blocks of code, like functions or loops; in MicroPython, you would use indentation instead.

Also notice each line ends with a semicolon (`;`). This tells the **compiler** that the statement is complete. In Python, you don’t need semicolons.

C is a **compiled language**, which means you write the code, then use a **compiler** to turn it into machine code that the Pico can run. This is different from interpreted languages like Python, where you run the code directly.

C is a **typed language**, meaning you must declare the type of each variable (like `int`, `float`, `char`, etc.) before using it. This helps catch errors early and makes your code more efficient.

---

## Summary

C is a powerful, efficient, and long-standing language that gives you precise control over how your programs run — ideal for working on microcontrollers like the Raspberry Pi Pico.

In the next lesson, we’ll dive into **variables and data types** in C so you can start writing your own logic.

Next up: [Variables and Data Types](03_variables_and_types)

---
