---
layout: lesson
title: If, elif, else
author: Kevin McAleer
type: page
previous: 11_reserved_words.html
next: 13_loops.html
description: Conditional control
percent: 70
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


![If Else Finally Diagram](assets/if_elif_else.jpg){:class="cover"}

## Contolling the flow of our program

So far, our programs have been simple lists of tasks that MicroPython carries out for us. But what if we want the program to do different things depending on the value of a variable?

We can test or check the value of our variables using the `if` statement.

---

## Conditional Statements

Using the `if` statement is simple, we need a condition to check, and then an action to follow if that condition is `True`.

We also need to do something different in our code, we need to indent the code under the `if` statement to tell MicroPython that this block of code is only to be run `if` the condition is `True`.

##### Example

```python
if name == 'Kevin':
    print('Hi Boss')
```

Notice that there are two equals signs `==`. This is because we use one equal sign `=` to assign values, but two equal signs `==` for comparing two value.

### Comparing values

We an also compare values to check if they are the same:

```python
a = 1
b = 1

if a == b: print('They are the same')
```

In the example above we assign the value `1` to the variable named `a`, and then we assign the value `1` to the variable `b`. We then compare the two using the `if a == b` condition. 

> ## Colons
>
> Notice the `:` colon after the condtional statement, this tells MicroPython that what follows is a
> block of code. We can write the code on the same line if its just one line, otherwise we need to
> indent the block of code like this:

```python
if a == b: 
    print('They are the same')
    print('a has the value', a, 'and b has the value',b)
```

This will output:

```bash
>>>
  They are the same
  a has the value 1 and b has the value 1
>>>
```

---

## If, Else

So far we've checked for a condition and `if` that condition is `True` then we've run a block of code, but what if we want to run another block of code if the condion is `False`. We ***could*** do this:

```python
if a == 1:
    print('a is 1')
if a != 1:
    print('a is not 1')
```

In the code above we check if `a` is equal to `1` (`if a == 1`) and if it is we print the message `a is q`, we then do another check to see if `a` is not equal to 1 (`if a != 1`), and then print the message `a is not 1`.

However, there is a quicker way to do the second test without another `if` statement. We can use the `else` keyword. `else` means run the block below if the first condition was `False`.

```python
if a == 1:
    print('a is 1')
else:
    print('a is not 1')
```

> ## Code Blocks & Indentation
>
> Note that Python and MicroPython recognise blocks of code by their indentation, which is defined by white space (typically `4` spaces per indented block).
> Other languages like `JavaScript` and `C` use curly braces `{ }` to define blocks of code.
>
> ##### Python Example
>
> ```python
> if a == 1:
>     # this is a code block, notice its indented by 4 spaces
>     print('a = ', a)
> ```
>
> ---
>
> ##### C++ Example
>
> ```c++
> #include <iostream>
>
> void main(){
>  int a;
>  if (a ==1) {
>  std::cout << "a = " << a;
>  }
> }
> ```
>
> You can see in the C++ example blocks of code are defined by the curly braces `{}`, and these are
> nested. Curly braces make the code more cluttered and also mean to have to remember to close the curly braces otherwise you'll have errors.
{:.bg-blue}

## elif

We're not done yet; `if` statements have one more trick, it can run another alternative block of code after checking and running the first block of code, or before running the `else` statement.

Elif, a shortening of "else if", sits between the first `if` statement test, and the final `else` statement. It can provide for additional checks each with their own blocks of code.

Only one of the blocks of code is run using the `if`, `elif` and `else` statement.

```python
if a == 1:
    print('a is 1')
elif a <= 1:
    print('a is less than 1')
else:
    print('a is not 1')
```

> ## Assumed True
>
> When checking to see if a value is `True` or `False` we can could write this:
>
> ```python
> a = True
> if a == True:
>    print('a is True')
> ```
>
> However we can shorten this:
>
> ```python
> a = True
> if a:
>   print('a is True')
> ```
>
> **Note** `1` is also `True` and `0` is also `False`
>

---
