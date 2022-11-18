---
layout: lesson
title: Why use MicroPython?
description: It's designed to be easy to read and write, perfect for beginners
---

MicroPython is ***easier to write***, ***cleaner*** and ***clearer to read***, and ***faster to get the results you want***, than other languages such as C++.

Compare these two examples below. 

First is a piece of code written for the `Arduino` in `C++`:

## C++

```C
#include “stdio.h”
void helloWorld() {
  printf(“hello world”);
}
void main() {
  helloWorld();
}
```

The second piece of code is written in `MicroPython`:

## MicroPython

```python
def hello_world():
  print(“hello world)

hello_world()
```

As you can see, the MicroPython code is simpler, easier to read, shorter and faster to write.
