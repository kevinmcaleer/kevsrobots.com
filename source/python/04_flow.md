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

{% capture snakie_code %}
age = 20

if age < 13:
    print("You are a kid.")
elif age < 20:
    print("You are a teenager.")
else:
    print("You are an adult.")
{% endcapture %}
{% include snakie.html code=snakie_code name="flow_1.py" engine="pyodide" %}

---

### Loops

Python has two types of loops - `for` and `while`.

- The `for` loop is used for iterating over a sequence or performing an action a specific number of times:

{% capture snakie_code %}
for i in range(5):
    print(i)
{% endcapture %}
{% include snakie.html code=snakie_code name="flow_2.py" engine="pyodide" %}

- The `while` loop repeats a block of code as long as a certain condition is true:

{% capture snakie_code %}
counter = 0
while counter < 5:
    print(counter)
    counter += 1
{% endcapture %}
{% include snakie.html code=snakie_code name="flow_3.py" engine="pyodide" %}

---

### Functions

Functions are reusable pieces of code. They only run when called and can receive parameters and return data.

{% capture snakie_code %}
# Defining a function
def greet(name):
    return f"Hello, {name}!"

# Calling a function
message = greet("Alice")
print(message)  # Prints "Hello, Alice!"
{% endcapture %}
{% include snakie.html code=snakie_code name="flow_4.py" engine="pyodide" %}

---

## Try it yourself

Write a function `is_even(n)` that returns `True` for even numbers. Then use a `for` loop to print every even number from 1 to 20.

{% quiz %}
? What does `range(5)` produce in a `for` loop?
- [ ] 1, 2, 3, 4, 5
- [x] 0, 1, 2, 3, 4
- [ ] 0, 1, 2, 3, 4, 5
> `range(5)` starts at 0 and stops before 5.

? When does a `while` loop stop?
- [x] When its condition becomes `False`
- [ ] After five repeats
- [ ] When it reaches the end of the file
> A `while` loop repeats as long as its condition is true, so make sure something changes inside the loop.
{% endquiz %}

---

## Summary

In this lesson, you've learned about controlling the flow of your Python programs using conditional statements, loops, and functions. Understanding control flow is crucial in writing dynamic and flexible Python programs.

---
