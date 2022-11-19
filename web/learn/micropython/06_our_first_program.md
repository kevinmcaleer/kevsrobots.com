---
layout: lesson
title: Our first program
author: Kevin McAleer
type: page
previous: 05_ides.html
next: 07_hello_world.html
description: "\u2018Hello World\u2019 - its a tradition"
percent: 49
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


Lets create a simple program that will display the message ‘Hello World’[^1].

First, here is the code, type this into your Python editor:

```python
print ('Hello World!')
```

When we run this, we will see the following printed to our screen (also refered to as the console):

``` bash
Hello World!
```
As you can see this is a very simple piece of code. The word `print` is a special word that Python understands. Python expects a line of text between the brackets, inside of the speech marks.

Notice that the `print` command is in lowercase; typing `Print` or `PRINT` will result in an error message.

We call this case sensitive; we need to ensure that the commands we type are in the correct case for Python to work properly.

[^1]: This is a tradition that many programming language tutorials use to help you understand how to write a simple program.