---
layout: lesson
title: Variables
author: Kevin McAleer
type: page
previous: 07_hello_world.html
next: 09_example02.html
description: A Place to store values
percent: 63
navigation:
- name: Learn MicroPython
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
    - name: Variables
      link: 08_variables.html
    - name: Example 02
      link: 09_example02.html
    - name: Values & Variables Types
      link: 10_values_and_variable_types.html
    - name: Reserved Words
      link: 11_reserved_words.html
    - name: Loops
      link: 12_loops.html
---


![Pigeon Holes Photo](assets/pigeon_holes.jpg){:class="cover"}

## Example

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
> In MicroPython variables don’t actually store the value, they just point to the thing that does.
