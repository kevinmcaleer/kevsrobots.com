---
layout: lesson
title: Variables
author: Kevin McAleer
type: page
previous: lesson06.html
next: values_and_variable_types.html
description: A Place to store values
percent: 64
navigation:
- name: Learn MicroPython
- content:
  - section: Overview
    content:
    - name: Introduction
      link: intro.html
  - section: Introduction
    content:
    - name: Why is it called Python?
      link: lesson01.html
    - name: Where to get MicroPython
      link: lesson02.html
    - name: Why use MicroPython?
      link: lesson03.html
    - name: Python Development Environments
      link: lesson04.html
    - name: Our first program
      link: lesson05.html
    - name: Example 01
      link: lesson06.html
  - section: Variables and Reserved Words
    content:
    - name: Variables
      link: lesson07.html
    - name: Values & Variables Types
      link: values_and_variable_types.html
    - name: Reserved Words
      link: reserved_words.html
    - name: Loops
      link: loops.html
    - name: Example 02
      link: example02.html
---


![Pigeon Holes Photo](assets/pigeon_holes.jpg){:class="cover"}

## Example

```python
a = 1
```

When we want to store a value we need to use a variable. In the example we looked at above we stored the number `1` in the variable `a`.

When we think about variables, we can imagine they are like little boxes that store things, and we give those boxes names so that it makes referring to them easier. We can put things into the boxes, look what’s in them or compare what’s in them with another box.

We could just give our variables a numbered label but that would make it difficult to remember what is in each one. By naming our variables we can easily understand what we expect the variable to contain. For example if we want to storge the age of a person we could use a variable named `age`.

```python
age = 21
```

In Python variables don’t actually store the value, they just point to the thing that does.