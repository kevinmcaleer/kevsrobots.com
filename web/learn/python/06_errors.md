---
layout: lesson
title: Error Handling and Exceptions in Python
author: Kevin McAleer
type: page
cover: /learn/python/assets/6.png
date: 2023-07-20
previous: 05_files.html
next: 07_libraries.html
description: Learn how to handle errors and exceptions in your Python programs to
  ensure they can recover gracefully from unexpected issues.
percent: 30
duration: 2
navigation:
- name: Python for beginners
- content:
  - section: Introduction
    content:
    - name: Introduction to Python Programming
      link: 01_intro.html
    - name: Python Basics
      link: 02_basics.html
    - name: Python Data Structures
      link: 03_data_structures.html
    - name: Control Flow in Python
      link: 04_flow.html
    - name: Working with Files in Python
      link: 05_files.html
    - name: Error Handling and Exceptions in Python
      link: 06_errors.html
    - name: Python Libraries and Modules
      link: 07_libraries.html
    - name: Python Object-Oriented Programming (OOP)
      link: 08_oop.html
    - name: Working with Data in Python
      link: 09_data.html
  - section: Real world Python
    content:
    - name: Web Scraping with Python
      link: 10_webscraping.html
    - name: Data Visualization
      link: 11_data_visualisation.html
    - name: Machine Learning with Python
      link: 12_ml.html
    - name: Deep Learning with Python
      link: 13_deep_learning.html
    - name: Python for Automating Tasks
      link: 14_automation.html
    - name: Python and Databases
      link: 15_databases.html
    - name: Advanced Python Topics
      link: 16_advanced.html
    - name: 'Project: Building a Simple Python Application'
      link: 17_apps.html
    - name: 'Bonus: Python Tips, Tricks and Best Practices'
      link: 18_tips_and_tricks.html
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
