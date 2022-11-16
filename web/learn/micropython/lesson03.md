---
layout: lesson
title: Why use MicroPython?
author: Kevin McAleer
type: page
previous: lesson02.html
description: It's designed to be easy to read and write, perfect for beginners
percent: 100
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
