---
layout: lesson
title: Our first program
author: Kevin McAleer
type: page
previous: 05_ides.html
next: 07_hello_world.html
description: "\u2018Hello World\u2019 - its a tradition"
percent: 40
navigation:
- name: Learn MicroPython - The basics
- content:
  - section: Overview
    content:
    - name: Introduction
      link: 00_intro.html
    - name: Learm MicroPython Introduction Video
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

Lets create a simple program that will display the message ‘Hello World’[^1].

First, here is the code, type this into your MicroPython editor:

```python
print ("Hello World!")
```

When we run this, we will see the following printed to our screen (also refered to as the console):

``` bash
Hello World!
```

To run this in Thonny, click the green `run` button:

![picture of run button](assets/run.png){:class="img-fluid w-100 shadow-lg"}

As you can see this is a very simple piece of code. The word `print` is a special word that Python understands. Python expects a line of text between the brackets, inside of the speech marks.

Notice that the `print` command is in lowercase; typing `Print` or `PRINT` will result in an error message.

We call this case sensitive; we need to ensure that the commands we type are in the correct case for Python to work properly.

Congratulations, you’re now a MicroPython programmer!

---

> ## Note
>
> The pound (or hash) symbol `#` means the reset of the line is a comment
> MicroPython ignores comments, so you can type useful reminders or notes to others here:
>
> ```python
> # a single line comment
>```
>
> For comments that span multiple lines, you can use 3 double quotations:
>
> ```python
> """ This is a multi -
> line comment """
> ```
{:class="blockquote bg-blue"}

---

## Why Print?

But why do we use the word `print`? It doesn’t come out of my printer?

Back in the early days of computing there were no screens, just a form of electronic typewriter that would `print` out the results of programs running on the attached computer. These terminals were called `teleprinters` or `teletype` machines which is why serial devices in Unix have the prefix `TTY`. A lot of computing stems from these early days and still remains in some form.

We put the `hello world` message in speech marks so that the computer knows where our message `starts` and `ends`. The speech marks can be either `' ' single` or `" " double` quotes.

> ## Quotes
>
> In MicroPython you can use double `" "` or single quotes `' '` when working with text strings,
> the preferred method is to use double quotes: read the [Black](https://black.readthedocs.io/en/stable/the_black_code_style/current_style.html#strings) style guide for more background on this. Double quotes anticipate the need for apostrophies within text so it makes sense to use them by default.

We use the brackets `( )` to tell the computer that we want to run a `function` (a named block of code), the function named `print`.

Also notice that the commands are in `lowercase` - no capital letters, this is because MicroPython (and Python) are `case sensitive` meaning the words `print`, `Print` and `PRINT` are not all the same.

The `print` function is built-in to MicroPython, it already knows how to print things, so we don’t have to include any extra commands to make that work

---

[^1]: This is a tradition that many programming language tutorials use to help you understand how to write a simple program.

---
