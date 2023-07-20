---
title: Error Handling and Exceptions in Python
description: Learn how to handle errors and exceptions in your Python programs to ensure they can recover gracefully from unexpected issues.
layout: lesson
cover: assets/6.png
---

![cover image]({{page.cover}}){:class="cover"}

## Introduction

In this lesson, we'll learn about error handling in Python. `Errors` are bound to happen in your code. Sometimes they are due to programmer error, and sometimes they are due to external circumstances beyond your control. Handling these errors properly can be the difference between a program crashing unexpectedly and a program failing gracefully.

---

## Learning Objectives

- Understand what `exceptions` are.
- Understand how to use `try`/`except` blocks to catch `exceptions`.
- Understand how to `raise` exceptions.

---

### What are Exceptions?

In Python, `exceptions` are events that occur during the execution of a program that disrupt the normal flow of the program's instructions. When a Python script encounters a situation that it can't cope with, it raises an exception. If the exception is not handled, the script stops executing and an error message is printed.

---

### Catching Exceptions

You can use a `try/except` block to catch exceptions and define how your program should respond to them.

```python
try:
    x = 1 / 0  # This will raise a ZeroDivisionError
except ZeroDivisionError:
    x = 0  # This code will run if a ZeroDivisionError occurs

print(x)  # Prints '0'
```

You can catch multiple exceptions by providing multiple `except` blocks. If you want to execute a block of code regardless of whether an exception was raised, you can use a `finally` block.

```python
try:
    # Try to open a non-existent file
    f = open("non_existent_file.txt", "r")
except FileNotFoundError:
    print("File not found.")
finally:
    print("This gets executed no matter what.")
```

---

### Raising Exceptions

You can raise exceptions with the `raise` statement. This is useful when you want to indicate that an error has occurred.

```python
x = -5

if x < 0:
    raise ValueError("x cannot be negative")
```

---

## Summary

In this lesson, you've learned about handling errors and exceptions in Python. You now know how to catch exceptions using try/except blocks and how to raise exceptions in your code. These tools are crucial for building robust Python applications that can withstand unexpected input and recover from errors gracefully.

---
