---
layout: lesson
title: Reserved Words
author: Kevin McAleer
type: page
cover: /learn/micropython/assets/micropython.jpg
date: 2022-12-04
previous: 10_values_and_variable_types.html
next: 12_if_elif_else.html
description: All the words that have a special meaning in Python
percent: 65
duration: 4
quiz_total: 57
date_updated: 2026-11-24
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


![Reserved sign on a table](assets/reserved.jpg){:class="cover"}

---

By the end of this lesson you will be able to explain what a reserved word is, spot one in a program, and avoid using one as a variable name.

## Special keywords

MicroPython has a list of special keywords that can't be used as names for variables, because each one has a job in the language. They are called `reserved words`. When you write `if`, MicroPython knows you are about to make a decision, so it can't also mean "a box called if".

Here is the full list of reserved words in MicroPython:

| Reserved Words |         |          |            |          |            |
|:--------------:|:-------:|:--------:|:----------:|:--------:|:----------:|
|     `and`      |  `as`   | `assert` |  `async`   | `await`  |  `break`   |
|    `class`     | `continue` | `def` |   `del`    |  `elif`  |   `else`   |
|    `except`    | `finally` | `for`  |   `from`   | `global` |    `if`    |
|    `import`    |  `in`   |   `is`   |  `lambda`  | `nonlocal` |  `not`   |
|      `or`      | `pass`  | `raise`  |  `return`  |  `try`   |  `while`   |
|     `with`     | `yield` |  `True`  |  `False`   |  `None`  |            |
{:class="table table-single"}

You don't need to memorise them. You've already met some (`if`, `else`, `import`, `True`), and we'll meet most of the others in this course and the later ones. Your editor helps too, because it colours reserved words differently from your own names.

## What happens if you use one?

Try to name a variable after a reserved word and MicroPython stops straight away:

<!-- code-check: skip -->
```python
class = "robot"
```

You get a `SyntaxError: invalid syntax`. The fix is simply to choose a more descriptive name, such as `robot_class` or `kind`:

```python
robot_kind = "rover"
print(robot_kind)
```

## Sneaky names that are not reserved

Some words, such as `print`, `input`, `type` and `list`, are not reserved, but they are the names of built-in functions. MicroPython lets you reuse them, which is a trap, because the function stops working:

<!-- code-check: skip -->
```python
print = 5
print("Hello")
```

The second line fails with `TypeError: 'int' object isn't callable`, because `print` is now a box holding 5. If this happens, press the board's reset or restart your session to get `print` back. A good habit is to use names like `speed_list` or `sensor_type` instead.

## Try it Yourself

1. Look at the table above. Find three words that you recognise from earlier lessons.
2. Type `for` as a variable name in your editor. What does it say?
3. Rename a reserved-word variable so the program runs.
4. Challenge: think of three good names for a variable holding a robot's battery voltage, and choose the best.

## Common Issues

- **Problem**: `SyntaxError: invalid syntax` on a line that looks fine.
- **Solution**: Check whether the variable name is a reserved word, such as `class`, `from` or `in`.
- **Why**: Reserved words are part of the language, so they can't be used as names.

- **Problem**: `TypeError: 'int' object isn't callable` when I call a function that worked before.
- **Solution**: You probably gave a variable the same name as a function, such as `print`. Rename it and restart.
- **Why**: Your variable replaced the built-in function with a number.

## Check your understanding

{% quiz %}
? What is a reserved word?
- [ ] A word you must always type in capitals
- [x] A word with a special meaning that you can't use as a variable name
- [ ] A word you can only use once
> Reserved words are part of the language, such as `if` and `while`.

? Which of these is a reserved word?
- [ ] `speed`
- [x] `while`
- [ ] `robot`
> `while` is in the list of reserved words.

? What happens if you run `print = 5` and then `print("Hi")`?
- [ ] It prints Hi
- [x] You get a `TypeError` because `print` is now the number 5
- [ ] The program closes
> Reusing a built-in function's name replaces it.
{% endquiz %}
