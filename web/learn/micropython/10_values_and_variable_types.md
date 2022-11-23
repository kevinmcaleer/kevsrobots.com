---
layout: lesson
title: Values & Variables Types
author: Kevin McAleer
type: page
previous: 09_example02.html
next: 11_reserved_words.html
description: NUMBERS, TEXT, LISTS AND DICTIONARIES
percent: 55
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


![Cover photo of a yellow typewritter](assets/types.jpg){:class="cover"}

## Data Types

Programs often need to work with numbers and text, adding and subtracting, asking questions and processing answers.

MicroPython is able to label different types of values so that it can make different commands available to us.

The different types of values are listed below:

{:.caption}
List of MicroPython data types

| **Text type:** | `str`
| **Numeric types:** | `int`, `float`, `complex`|
| **Lists and sequences:** | `list`, `tuple` |
| **Dictionaries (mapping of key-value pairs)** | `dict` |
| **Set types:** | `set`, `frozenset` |
| **True/False (Boolean) type:** | `bool`|
| **Binary types:** | `bytes`, `bytearray`, `memoryview` |
| **None type:** | `NoneType`|
{:class="table w-auto table-bordered"}

---

{:.caption}
List of common MicroPython data types with examples

| Value Types | Description                                                        | Example                                                |
|:------------|:-------------------------------------------------------------------|--------------------------------------------------------|
| `int`       | A whole number, which can be positive or negative                  | `1`, `2`, `3`, `4`, `-500`, `1000`                     |
| `float`     | A decimal number, which can be positive or negative                | `3.14`, `-2.5`, `1.66666`                              |
| `str`       | A line of text                                                     | `'hello world'`, `'my name is Kevin'`                  |
| `list`      | A collection of words or numbers, or objects, (with no duplicates) | `[‘cat’,’dog’,’fish’,’gecko’],` or `[1,2,3,5,8,13,21]` |
| `dict`      | A dictionary - a list, with pairs of values                        | `{‘name’:’Kevin’, ‘address’:’UK’, ‘age’:45}`          |
| `tuple`     | A collection of ordered items, can have duplicates                 | `("fish",1,2,3,)`                                      |
| `object`    | An instance of a class (we’ll cover this in a later session)       | Object of `'Cat()'` Class                                |
| `byte`      | A single 8 bit byte, which is a value between 0 and 255            | `255`, `'0xFF'`, `'0b11111111'`                        |
{:class="table-w100 table table-bordered"}

**Note** The more complex data types have been excluded from this list. We'll look at them in more detail in later modules.

---

When we assign a value to a variable, it is said to have the a *type*.
e.g.

```python
a = 1
b = 2.1
c = "hello"
```

In this example `a` contains the value `1` and is of type `integer`, `b` contains the value `2.1` and is of type `float` (because it has a decimal point), and `c` contains `hello` and is of type `string` because it contains a text wrapped inside some speech marks.

---

> ## Implied Types
>
> In other languages, such as `C/C++` you will need to tell the compiler what kind of datatype a variable
> is before it is used.
> This is known as a `strongly typed` language; it helps the compiler allocate the correct amount of
> memory for the variable before it is used.
>
> ##### C++ Example
>
> ```c++
> int a;   // tells the compiler that 'a' is an integer
> float b; // tells the compiler that 'b' is a floating point number 
> a = 1;
> b = 2.3;
> ```
>
> ---
>
> In `Python` and `MicroPython` you do not need to assign a type to a variable, Python is smart enough to
> figure this out by the value being assigned to it. This is known as `weakly typed` language.
> The trade off is that Python looks cleaner to read and doesn't need data types to be assigned before
> they're used, however this can lead to confusion if another type of value is expected or assigned to a
> variable of a differnet type.
>
> ##### MicroPython Example
>
> ```python
> a = 1   # python knows that 1 is an integer value so 'a' becomes an 'int' type
> b = 2.3 # python knows that 2.3 is a floating point value so 'b' becomes a 'float' type
> ```
>

---

> ## George Boole the Father of Boolean Logic
>
> English mathematician and logician [George Boole](https://en.wikipedia.org/wiki/George_Boole) was the author of `The Laws of Thought` *(1854)*
> which documents the *algebra of logic*. He founded the idea that logic can be expressed in a series
> of symbols that show an output of `True` or `False` given a set of inputs. These can be expressed as
> `Truth Tables`, and is now commonly called `Boolean Logic`, which is what all modern computing is
> based upon.
>
> ---
> 
> ##### Example Truth Table
>
> **'AND' Logic**
>
> | input a | input b | output x |
> |:--:|:--:|:--:|
> |False|False|False|
> |`True`|False|False|
> |`True`|`True`|`True`|
> {:class="table w-auto table-bordered"}
> 
> An `AND` logic gate has two inputs; `a` and `b` and one output `x`. The table shows each of the possible state the inputs and the outputs can be in. With `AND` gates both inputs need to be `True` for the output to be `True`. This is used a lot in graphics processing too.
>
> We will look at logic operators in MicroPython in [the Operators lesson](14_operators).
{:.bg-blue}

---
