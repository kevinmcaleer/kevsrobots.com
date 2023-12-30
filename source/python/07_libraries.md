---
title: Python Libraries and Modules
description: Get introduced to the use of Python libraries and modules, learn how to import them and explore a few popular libraries like math, random and datetime.
layout: lesson
cover: /learn/python/assets/7.png
---

![cover image]({{page.cover}}){:class="cover"}

## Introduction

Python is known for its rich set of libraries and modules which simplifies the coding experience. In this lesson, we will understand what Python libraries and modules are, how to import them, and get a brief look at a few popular libraries.

---

## Learning Objectives

- Understand what Python `libraries` and `modules` are.
- Learn how to import libraries and modules.
- Get familiar with a few popular Python libraries: `math`, `random`, and `datetime`.

---

### What are Libraries and Modules?

In Python, a `module` is a `file` containing Python code. A `library` is a collection of modules. They define `functions`, `classes`, and `variables` that you can reuse in your programs.

---

### Importing Modules

You can use the `import` keyword to import a module or a library. Once imported, you can use the dot notation to access the functions, classes, and variables defined in that module.

```python
# Importing the math module
import math

# Using a function from the math module
print(math.sqrt(16))  # Prints '4.0'
```

---

### The `math` Module

The `math` module provides mathematical functions and constants.

```python
import math

# Constants
print(math.pi)  # Prints '3.141592653589793'

# Trigonometric functions
print(math.sin(math.pi/2))  # Prints '1.0'

# Logarithmic functions
print(math.log(100, 10))  # Prints '2.0'
```

---

### The `random` Module

The `random` module provides functions for generating random numbers.

```python
import random

# Generate a random float between 0 and 1
print(random.random())

# Generate a random integer between 1 and 10
print(random.randint(1, 10))

# Randomly choose an item from a list
print(random.choice(['apple', 'banana', 'cherry']))
```

---

### The `datetime` Module

The `datetime` module provides classes for manipulating dates and times.

```python
import datetime

# Get the current date and time
now = datetime.datetime.now()
print(now)

# Create a specific date
independence_day = datetime.datetime(1776, 7, 4)
print(independence_day)

# Calculate the difference between two dates
delta = now - independence_day
print(delta.days)
```

---

## Summary

In this lesson, you've learned about Python libraries and modules. You've learned how to import them and explored a few popular libraries: `math`, `random`, and `datetime`. Libraries and modules are powerful tools that allow you to leverage existing code and build complex applications more easily.

---
