---
layout: lesson
title: Example 02
author: Kevin McAleer
type: page
previous: 08_variables.html
next: 10_values_and_variable_types.html
description: Variables
percent: 50
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
  - section: Summary and Review
    content:
    - name: Summary and Review
      link: 18_summary.html
---


Let’s do something more interesting.

We'll make the Raspberry Pi Pico (or whichever board you are using) ask us a question, then reply back to us.

## name.py

Type of the program below into your Python Editor

```python
# Variables
# name.py

print("Please type your name:")
name = input()
print("Hello", name)
```

Save the file and then run it by pressing the green `run` button.

The `name` variable holds the value that it is assigned, and we assigned it by using the `=` equals sign.

The function `input`, is another built-in function that MicroPython provides to get user input from the keyboard and return it to our program.

The variable is like a box that we can put things in, in this case a persons name.

## Numbers

We can also store `numbers` in variables; let's extend our example and add an extra question

type:

```python
name = input('hello, please enter your name > ')
age = input('please enter your age > ')
print('hello', name, 'how does', age, 'feel?')
```

![Age Program](assets/age_program.png){:class="img-fluid w-100"}
![Age Console Output](assets/age_console.png){:class="img-fluid w-100"}

---

> ## Note
>
> `input()` only returns text (`str`) types of data, this means if you try to do any math with the age
> it will not work in the way you expect. 
>
> If you want `age` to be a number you will too to wrap it in the `int()` type to change it. 
>
> E.g.
>
> ```python
> age = int(input('please enter your age >'))
> print('next year you will be', age + 1, 'years old')
> ```
>
> This is called `casting` and we'll look at this in more depth in later modules.

We can use variables in maths, for example if we want to find the missing angle in a triangle, and we have the other two angles we can use a formula: `180 - a + b = c`

type:

```python
a = 65
b = 42
c = 180 - a + b 
print('the missing angle is: ', c)
```

---
