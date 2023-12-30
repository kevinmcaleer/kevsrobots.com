---
title: Control Flow in Python
description: Understand how to control the flow of your Python programs using conditional statements, loops, and functions.
layout: lesson
cover: /learn/python/assets/4.png
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
