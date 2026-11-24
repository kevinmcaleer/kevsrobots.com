---
layout: lesson
title: Our first program
author: Kevin McAleer
type: page
cover: /learn/micropython/assets/micropython.jpg
date: 2022-12-04
previous: 05_ides.html
next: 07_hello_world.html
description: Hello World, it's a tradition
percent: 40
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


![Cover photo of a duckling running on grass](assets/hello_world.jpg){:class="cover"}

---

By the end of this lesson you will be able to write and run a program that prints a message, and explain what `print`, brackets and quotes do.

Let's create a simple program that will display the message 'Hello World'[^1].

First, here is the code. Type this into your MicroPython editor:

{% capture snakie_code %}
print("Hello World!")
{% endcapture %}
{% include snakie.html code=snakie_code name="our_first_program.py" %}

When we run this, we will see the following printed to the screen (also referred to as the console or shell):

``` bash
Hello World!
```

To run it in Thonny, click the green `run` button. In Snakie, click `Run`.

![picture of run button](assets/run.png){:class="img-fluid w-100 shadow-lg"}

Congratulations, you're now a MicroPython programmer!

## Anatomy of the line

```mermaid
flowchart LR
    A["print"] --> B["( )"] --> C["quotes"] --> D["Hello World!"]
```

* `print` is a **function**, a named block of code that is built in to MicroPython, so it already knows how to show things
* The brackets `( )` tell MicroPython to run the function now. What goes inside is what the function should work on
* The speech marks (quotes) mark where the text `starts` and `ends`. Text in a program is called a `string`

You can use single quotes `' '` or double quotes `" "`, as long as you open and close with the same kind. Double quotes are a good habit, because they let you write an apostrophe inside your message: `print("It's alive!")`.

## Case sensitive

Notice that `print` is lowercase. Typing `Print` or `PRINT` gives an error message. We say MicroPython is `case sensitive`, so `print`, `Print` and `PRINT` are three different names, and only the first one exists.

## Why print?

It doesn't come out of my printer? Back in the early days of computing there were no screens, just an electronic typewriter called a `teleprinter` or `teletype` machine, which would `print` out the results of programs. That is also why serial devices in Unix have names starting with `TTY`. The name stuck, even though it now prints to the screen.

## Comments

The hash symbol `#` means the rest of the line is a comment. MicroPython ignores comments, so you can leave reminders for yourself and others:

```python
# this is a single line comment
print("Robot online")  # a comment can follow code, too
```

For longer notes you can wrap text in triple quotes:

```python
"""This note
spans several lines"""
print("Ready")
```

## Try it Yourself

1. Change the message so your program prints the name of your robot.
2. Add a second `print` line underneath. In which order do the messages appear?
3. Print a message that has an apostrophe in it, such as `It's alive!`, in two different ways. Hint: use different quotes.
4. Challenge: print three lines that draw a tiny robot face using symbols such as `[ o_o ]`.

## Common Issues

- **Problem**: `NameError: name 'Print' isn't defined`.
- **Solution**: Change `Print` to lowercase `print`.
- **Why**: MicroPython is case sensitive, so the capital letter makes it a different, unknown name.

- **Problem**: `SyntaxError: invalid syntax` pointing at the end of the line.
- **Solution**: Check every opening quote and bracket has a matching closing one.
- **Why**: MicroPython can't tell where your text or function call ends.

- **Problem**: Nothing is printed.
- **Solution**: Look in the shell or console panel, and check you pressed `Run`.
- **Why**: Output appears in the console, not in the editor window.

## Check your understanding

{% quiz %}
? What does `print("Hello")` do?
- [ ] Sends the word Hello to a paper printer
- [x] Shows the word Hello in the console
- [ ] Saves the word Hello in a file
> `print` displays its input on the screen (the console).

? Why does `Print("Hi")` give an error?
- [ ] Brackets are not allowed
- [x] MicroPython is case sensitive, and only lowercase `print` exists
- [ ] Hi is too short
> `print`, `Print` and `PRINT` are all different names.

? What does the `#` symbol do in a program?
- [ ] It prints a hash
- [x] It starts a comment that MicroPython ignores
- [ ] It stops the program
> Everything after `#` on the line is a note for people.
{% endquiz %}

---

[^1]: This is a tradition that many programming language tutorials use to help you understand how to write a simple program.
