---
layout: lesson
title: Values & Variables Types
author: Kevin McAleer
type: page
previous: 09_example02.html
next: 11_reserved_words.html
description: NUMBERS, TEXT, LISTS AND DICTIONARIES
percent: 77
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


Programs often need to work with numbers and text, adding and subtracting, asking questions and processing answers.

Python is able to label different types of values so that it can make different commands available to us.

The different types of values are listed below:

| Value Types | Description                                                  | Example                                            |
|:------------|:-------------------------------------------------------------|----------------------------------------------------|
| `integer`   | A whole number, which can be positive or negative            | `1, 2, 3, 4, -500, 1000`                            |
| `float`     | A decimal number, which can be positive or negative          | `3.14, -2.5, 1.66666 `                               |
| `string`    | A line of text                                               | `'hello world', 'my name is Kevin'  `                |
| `list`      | A collection of words or numbers, or objects                 | `[‘cat’,’dog’,’fish’,’gecko’], `or `[1,2,3,5,8,13,21]` |
| `dict`      | A dictionary - a list, with pairs of values                  | `{‘name’:’Kevin’, ‘address’:’UK’, ‘age’:45} `        |
| `object`    | An instance of a class (we’ll cover this in a later session) | Object of `'Cat'` Class                              |
{:class="table-w100 table table-bordered"}

When we assign a value to a variable, it is said to have the a *type*.
e.g.

```python
a = 1
b = 2.1
c = "hello"
```

In this example `a` contains the value `1` and is of type `integer`, `b` contains the value `2.1` and is of type `float` (because it has a decimal point), and `c` contains `hello` and is of type `string` because it contains a text wrapped inside some speech marks.

---