---
title: Python Basics
description: Dive into the basics of Python, including variables, data types, syntax, comments, and operators.
layout: lesson
cover: /learn/python/assets/2.png
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
