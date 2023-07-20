---
layout: lesson
title: Control Flow in Python
author: Kevin McAleer
type: page
cover: assets/4.png
previous: 03_data_structures.html
next: 05_files.html
description: Understand how to control the flow of your Python programs using conditional
  statements, loops, and functions.
percent: 20
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

In this lesson, we'll explore how to control the flow of your Python programs using conditional statements (`if`, `elif`, `else`), loops (`for`, `while`), and `functions`.

---

## Learning Objectives

- Understand and use conditional statements: `if`, `elif`, `else`.
- Understand and use loops: `for`, `while`.
- Understand how to define and call `functions`.

---

### Conditional Statements

In Python, we use `if`, `elif` (else if), and `else` to control conditional flows. They help the program to make decisions based on certain conditions.

```python
age = 20

if age < 13:
    print("You are a kid.")
elif age < 20:
    print("You are a teenager.")
else:
    print("You are an adult.")
```

---

### Loops

Python has two types of loops - `for` and `while`.

- The `for` loop is used for iterating over a sequence or performing an action a specific number of times:

```python
for i in range(5):
    print(i)
```

- The `while` loop repeats a block of code as long as a certain condition is true:

```python
counter = 0
while counter < 5:
    print(counter)
    counter += 1
```

---

### Functions

Functions are reusable pieces of code. They only run when called and can receive parameters and return data.

```python
# Defining a function
def greet(name):
    return f"Hello, {name}!"

# Calling a function
message = greet("Alice")
print(message)  # Prints "Hello, Alice!"
```

---

## Summary

In this lesson, you've learned about controlling the flow of your Python programs using conditional statements, loops, and functions. Understanding control flow is crucial in writing dynamic and flexible Python programs.

---
