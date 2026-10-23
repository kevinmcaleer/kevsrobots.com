---
title: Advanced Python Topics
description: Dive into some advanced topics in Python, including decorators, generators, and context managers.
layout: lesson
cover: /learn/python/assets/2.png
---

![cover image]({{page.cover}}){:class="cover"}

## Introduction

Python is a rich language with many advanced features. This lesson will introduce you to a few of these features: decorators, generators, and context managers. These tools can help you write more efficient and cleaner code.

---

## Watch the video

{% include youtubeplayer.html id="uRNsqHZEzUM" %}

In this livestream I go beyond the basics with Python for robotics.

---

## Learning Objectives

- Understand what decorators are and how to use them.
- Learn how to create and use generators.
- Understand what context managers are and how to use them.

---

### Python Decorators

Decorators allow us to wrap another function in order to extend the behavior of the wrapped function, without permanently modifying it.

```python
def my_decorator(func):
    def wrapper():
        print("Before function call")
        func()
        print("After function call")
    return wrapper

@my_decorator
def say_hello():
    print("Hello!")

say_hello()  # prints: Before function call, Hello!, After function call
```

That `wrapper()` only works for functions with no arguments. A more general decorator passes everything through and keeps the original function's name with `functools.wraps`:

```python
import functools
import time

def timer(func):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        start = time.perf_counter()
        result = func(*args, **kwargs)
        print(f"{func.__name__} took {time.perf_counter() - start:.3f}s")
        return result
    return wrapper

@timer
def add(a, b):
    return a + b

print(add(2, 3))
```

---

### Python Generators

Generators are a type of iterable, like lists or tuples. Unlike lists, they produce values one at a time, only when asked, so they use very little memory even for huge sequences. You can't index them, but you can loop over them.

```python
def simple_generator():
    yield 1
    yield 2
    yield 3

for value in simple_generator():
    print(value)  # prints: 1, 2, 3
```

---

### Python Context Managers

Context managers allow you to allocate and release resources precisely when you want to. The most widely used example of context managers is the `with` statement.

```python
with open('file.txt', 'r') as f:
    file_contents = f.read()
# the file is automatically closed outside of the with block
```

---

{% quiz %}
? What keyword turns a function into a generator?
- [ ] `return`
- [x] `yield`
- [ ] `generate`
> A function containing `yield` hands back values one at a time.

? What does the `@timer` line above a function do?
- [x] Wraps the function with the `timer` decorator
- [ ] Runs the function on a schedule
- [ ] Makes the function private
> `@timer` is shorthand for `add = timer(add)`.
{% endquiz %}

---

## Summary

In this lesson, you've learned about some of Python's advanced features: decorators, generators, and context managers. These features can help you write more efficient and cleaner code. Understanding these concepts can be a stepping stone to mastering Python.

---
