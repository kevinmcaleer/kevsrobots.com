---
layout: lesson
title: Advanced Python Topics
author: Kevin McAleer
type: page
cover: /learn/python/assets/2.png
date: 2023-07-20
previous: 15_databases.html
next: 17_apps.html
description: Dive into some advanced topics in Python, including decorators, generators,
  and context managers.
percent: 80
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

Python is a rich language with many advanced features. This lesson will introduce you to a few of these features: decorators, generators, and context managers. These tools can help you write more efficient and cleaner code.

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

---

### Python Generators

Generators are a type of iterable, like lists or tuples. Unlike lists, they don't allow indexing with arbitrary indices, but they can still be iterated through with for loops.

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

## Summary

In this lesson, you've learned about some of Python's advanced features: decorators, generators, and context managers. These features can help you write more efficient and cleaner code. Understanding these concepts can be a stepping stone to mastering Python.

---
