---
layout: lesson
title: Python Basics
author: Kevin McAleer
type: page
cover: /learn/python/assets/2.png
date: 2023-07-20
previous: 01_intro.html
next: 03_data_structures.html
description: Dive into the basics of Python, including variables, data types, syntax,
  comments, and operators.
percent: 10
duration: 3
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

In this lesson, we'll explore the basics of Python programming. This includes understanding `variables`, `data types`, `basic syntax`, `comments`, and `operators`.

---

## Learning Objectives

- Understand and use variables and data types.
- Learn and apply basic Python syntax.
- Learn how to comment code.
- Understand and use operators in Python.

---

### Variables and Data Types

Variables are containers that hold data. Python has various data types including:

- **Integers**: These are whole numbers, such as `10`, `15`, `22`.
- **Floats**: These are decimal numbers, such as `1.5`, `2.67`, `3.14`.
- **Booleans**: These represent truth values and can be either `True` or `False`.
- **Strings**: These are sequences of characters, such as `'hello'`, `'Python'`.

Here's how you can create variables of different types:

```python
# An integer
age = 25

# A float
height = 5.6

# A boolean
is_adult = True

# A string
name = "Alice"
```

---

### Basic Python Syntax

Python has a clean and easy-to-understand syntax. Here are some key points:

- **Indentation**: Python uses indentation (spaces or tabs at the beginning of lines) to define blocks of code.

```python
if age > 18:
    print("You are an adult.")
```

- **Statements**: In Python, each logical line of code is called a "statement". Python interprets these statements in the order they appear.

```python
print("Hello, World!")
print("Welcome to Python programming.")
```

- **Line Continuation**: For breaking long statements into multiple lines, you can use the line continuation character (`\`).

```python
total = 1 + 2 + 3 + \
        4 + 5 + 6
```

---

### Comments

Comments are essential in making your code understandable to others and your future self. They are lines that are not executed by the Python interpreter.

```python
# This is a single-line comment
print("Hello, World!")  # This is an inline comment

"""
This is a
multi-line comment
"""
```

---

### Operators

Python supports a variety of operators:

- **Arithmetic Operators**: `+`, `-`, `*`, `/`, `//`, `%`, `**`

```python
x = 10
y = 3

print(x + y)  # Addition
print(x - y)  # Subtraction
print(x * y)  # Multiplication
print(x / y)  # Division
print(x // y)  # Floor division
print(x % y)  # Modulus
print(x ** y)  # Exponentiation
```

- **Comparison Operators**: `==`, `!=`, `>`, `<`, `>=`, `<=`

```python
print(x == y)  # Equal to
print(x != y)  # Not equal to
print(x > y)  # Greater than
print(x < y)  # Less than
print(x >= y)  # Greater than or equal to
print(x <= y)  # Less than or equal to
```

- **Logical Operators**: `and`, `or`, `not`

```python
age = 22
is_student = True

if age > 18 and is_student:
    print("You are an adult student.")
```

---

## Summary

This lesson covered the basics of Python, including variables, data types, basic syntax, comments, and operators. These are foundational concepts that you'll use in every Python program you write.

---
