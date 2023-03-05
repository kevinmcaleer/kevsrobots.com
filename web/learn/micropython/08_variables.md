---
layout: lesson
title: Variables and Constants
author: Kevin McAleer
type: page
previous: 07_hello_world.html
next: 09_example02.html
description: A Place to store values
percent: 50
duration: 2
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


![Pigeon Holes Photo](assets/pigeon_holes.jpg){:class="cover"}

##### Example

```python
a = 1
```

When we want to store a value we need to use a `variable`. In the example we looked at above we stored the number `1` in the variable `a`, we assigned it by using the equals sign.

When we think about variables, we can imagine they are like little boxes that store things, and we give those boxes names so that it makes referring to them easier. We can put things into the boxes, look what’s in them or compare what’s in them with another box.

We could just give our variables a numbered label but that would make it difficult to remember what is in each one. By naming our variables we can easily understand what we expect the variable to contain. For example if we want to storge the age of a person we could use a variable named `age`.

```python
age = 21
```

---

> ## MicroPython Facts
>
> Like Python, MicroPython variables don’t actually store the value, they just point to the thing that does.
{:class="blockquote bg-blue"}

---

## Constants

If `variables` contain values that change, `Constants` contain values that dont change.
For example, if we wanted to store the value of `pi`, the value of `pi` does not change, so we can store this in a special type of object called a `constant`.

##### Example

```python
PI = const(3.141592654)
```

In the example above we use the `const()` function to define the value `3.141592654` as the constant `PI`. The convention is that constants are always in uppercase.

> ## Const()
>
> `const()` is a specific to MicroPython, in regular python constants are treated as varibles, just in upper case.

---
