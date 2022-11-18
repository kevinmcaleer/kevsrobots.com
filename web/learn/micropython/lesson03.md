---
layout: lesson
title: Why use MicroPython?
author: Kevin McAleer
type: page
previous: installing_micropython.html
next: lesson04.html
description: It's designed to be easy to read and write, perfect for beginners
percent: 35
navigation:
- name: Learn MicroPython
- content:
  - section: Overview
    content:
    - name: Introduction
      link: intro.html
  - section: Introduction
    content:
    - name: Why is it called MicroPython?
      link: why_is_it_called_micropython.html
    - name: Where to get MicroPython
      link: lesson02.html
    - name: How to Install MicroPython
      link: installing_micropython.html
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


MicroPython is easier to write, cleaner and clearer to read, and faster to get the results you want, than other languages such as C++.

Compare these two examples below. First is a piece of code written for the Arduino in C++:

```C
#include “stdio.h”
void helloWorld() {
  printf(“hello world”);
}
void main() {
  helloWorld();
}
```

The second piece of code is written in MicroPython:

```python
def hello_world():
  print(“hello world)

hello_world()
```

As you can see, the MicroPython code is simpler, easier to read, shorter and faster to write.
