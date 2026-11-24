---
layout: lesson
title: Control Flow in Python
author: Kevin McAleer
type: page
cover: /learn/python/assets/banners/04_flow.jpg
date: 2023-07-20
previous: 03_data_structures.html
next: 05_files.html
description: Make your programs decide and repeat with if statements, loops and functions,
  using a line-following robot as the example.
percent: 20
duration: 5
quiz_total: 58
date_updated: 2026-11-24
navigation:
- name: Python for beginners
- content:
  - section: Introduction
    content:
    - name: Introduction to Python Programming
      link: 00_intro.html
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
    - name: Data Visualisation
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


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to make decisions with `if`, `elif` and `else`, repeat work with `for` and `while`, and package code into reusable functions.

---

So far our programs run straight from the top to the bottom. Real robots need to **decide** ("is something in the way?") and **repeat** ("keep checking, forever"). That is control flow.

```mermaid
flowchart TD
    S["Read distance sensor"] --> D{"distance < 20 cm?"}
    D -- yes --> T["Turn left"]
    D -- no --> F["Drive forward"]
    T --> S
    F --> S
```

## Conditional statements

`if`, `elif` (short for else if) and `else` pick which block of code runs. Python checks the conditions from the top and runs the first one that is true.

{% capture snakie_code %}
distance_cm = 12

if distance_cm < 10:
    print("Stop!")
elif distance_cm < 30:
    print("Slow down")
else:
    print("Full speed ahead")
{% endcapture %}
{% include snakie.html code=snakie_code name="flow_1.py" engine="pyodide" %}

The colon at the end of each condition and the indented line below it are both required. Try changing `distance_cm` to 5, 25 and 100 and see which branch runs.

## Loops

Python has two kinds of loop.

A `for` loop works through a sequence, or repeats a set number of times:

{% capture snakie_code %}
for i in range(5):
    print(i)          # 0, 1, 2, 3, 4

for sensor in ["left", "middle", "right"]:
    print("Reading the", sensor, "sensor")
{% endcapture %}
{% include snakie.html code=snakie_code name="flow_2.py" engine="pyodide" %}

A `while` loop repeats as long as its condition is true:

{% capture snakie_code %}
battery_percent = 100

while battery_percent > 70:
    print("Driving, battery at", battery_percent)
    battery_percent -= 10     # something must change, or the loop never ends
{% endcapture %}
{% include snakie.html code=snakie_code name="flow_3.py" engine="pyodide" %}

Two helpers are worth knowing: `break` leaves a loop early, and `continue` skips to the next time round.

## Functions

A function is a named, reusable block of code. It only runs when you call it, it can take **parameters** (inputs) and it can **return** a result.

{% capture snakie_code %}
def choose_action(distance_cm):
    if distance_cm < 10:
        return "stop"
    elif distance_cm < 30:
        return "slow"
    return "go"

for reading in [80, 25, 8]:
    print(reading, "cm ->", choose_action(reading))
{% endcapture %}
{% include snakie.html code=snakie_code name="flow_4.py" engine="pyodide" %}

**Why this matters:** once a decision lives in a function you can test it on its own, reuse it, and give it a name that explains what it does. Aim for functions that do one thing.

---

## Try it Yourself

1. Write a function `is_even(n)` that returns `True` for even numbers (hint: `n % 2 == 0`). Use a `for` loop to print every even number from 1 to 20.
1. Change `choose_action()` so it also returns `"reverse"` when the distance is under 5 cm.
1. Use a `while` loop to count down from 10 to 1, then print `Lift off!`.
1. Challenge: write a function `average(readings)` that returns the average of a list, without using `sum()`.

## Common Issues

- **Problem**: A `while` loop never stops and the program seems frozen.
- **Solution**: Make sure something inside the loop changes the condition. In a terminal, press Ctrl+C to stop it.
- **Why**: A `while` loop only ends when its condition becomes false.

- **Problem**: `SyntaxError: expected ':'`.
- **Solution**: Add the colon at the end of the `if`, `elif`, `else`, `for`, `while` or `def` line.
- **Why**: The colon tells Python that an indented block follows.

- **Problem**: A function prints `None`.
- **Solution**: Use `return` to hand back a value. `print()` only displays it.
- **Why**: A function with no `return` gives back `None` by default.

## Check your understanding

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

? In `if`, `elif`, `else`, how many of the blocks run?
- [ ] All of them
- [x] Only the first one whose condition is true (or the `else` if none are)
- [ ] The last one
> Python works down the chain and runs just one branch.

? What does a function return if it has no `return` statement?
- [ ] 0
- [ ] An empty string
- [x] `None`
> `None` is Python's way of saying "no value".
{% endquiz %}

---

## Summary

You can now make decisions, repeat actions and wrap code into functions. In the next lesson we will save data to files so it survives after the program stops.
