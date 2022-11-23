---
layout: lesson
title: Why use MicroPython?
author: Kevin McAleer
type: page
previous: 03_installing_micropython.html
next: 05_ides.html
description: It's designed to be easy to read and write, perfect for beginners
percent: 25
navigation:
- name: Learn MicroPython - The basics
- content:
  - section: Overview
    content:
    - name: Introduction
      link: 00_intro.html
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
---


![Cover photo of a jungle](assets/why_micropython.jpg){:class="cover"}

MicroPython is ***easier to write***, ***cleaner*** and ***clearer to read***, and ***faster to get the results you want***, than other languages such as C++.

Compare these two examples below. 

First is a piece of code written for the `Arduino` in `C++`:

## C++

```C
#include <iostream>

void main() {
  std::cout << "hello world";
}
```

The second piece of code is written in `MicroPython`:

## MicroPython

```python
  print(“hello world)
```

As you can see, the MicroPython code is simpler, easier to read, shorter and faster to write.
