---
layout: lesson
title: Timers
author: Kevin McAleer
type: page
cover: assets/cover.png
date: 2024-07-07
previous: 11_interrupts.html
next: 13_mp_remote.html
description: Learn about timers in MicroPython and how to use them for time-sensitive
  applications.
percent: 84
duration: 3
navigation:
- name: Intermediate level MicroPython
- content:
  - section: Introduction
    content:
    - name: Introduction to Intermediate MicroPython
      link: 01_intro.html
  - section: Object Oriented Programming
    content:
    - name: Object-Oriented Programming in MicroPython
      link: 02_oop.html
    - name: Classes and Objects
      link: 02a_classes.html
    - name: Abstraction
      link: 03_abstraction.html
    - name: Inheritance
      link: 04_inheritance.html
    - name: Encapsulation
      link: 05_encapsulation.html
    - name: Polymorphism
      link: 06_polymorphism.html
  - section: Decorators
    content:
    - name: Decorators
      link: 06a_decorators.html
  - section: Modules and Packages
    content:
    - name: Modules & Libraries
      link: 07_modules.html
    - name: Packages
      link: 08_packages.html
    - name: PyPi & MIP
      link: 09_pypi.html
  - section: DocStrings
    content:
    - name: DocStrings
      link: 10_docstrings.html
  - section: Interrupts and Timers
    content:
    - name: Interrupts
      link: 11_interrupts.html
    - name: Timers
      link: 12_timers.html
  - section: MP Remote
    content:
    - name: MP Remote
      link: 13_mp_remote.html
  - section: Conclusion
    content:
    - name: MP Remote
      link: 14_summary.html
---


## Timers

In MicroPython, a `timer` is a hardware component that can be used to measure time intervals, trigger events, and schedule tasks. Timers are essential for time-sensitive applications, such as controlling motors, reading sensors, and managing communication protocols.

---

## What is a Timer?

A timer is a hardware component that keeps track of time intervals. It can be configured to trigger events at specific intervals, making it useful for a variety of applications that require precise timing.

### Types of Timers

- **One-Shot Timer**: Triggers an event once after a specified interval.
- **Periodic Timer**: Triggers events repeatedly at regular intervals.

---

## Using Timers in MicroPython

In MicroPython, the `machine` module provides functionality to configure and use timers. Here’s how you can set up and use timers in your projects.

### Example: Setting Up a Periodic Timer

Here’s an example of setting up a periodic timer that triggers an event every second:

```python
import machine

# Define the timer callback function
def tick(timer):
    print("Tick!")

# Create a timer object
timer = machine.Timer(-1)

# Initialize the timer to trigger the callback every second (1000 ms)
timer.init(period=1000, mode=machine.Timer.PERIODIC, callback=tick)
```

In this example, the `tick` function is called every second by the timer.

### Example: One-Shot Timer

Here’s an example of setting up a one-shot timer that triggers an event after 5 seconds:

```python
import machine

# Define the timer callback function
def timeout(timer):
    print("Timeout!")

# Create a timer object
timer = machine.Timer(-1)

# Initialize the timer to trigger the callback once after 5 seconds (5000 ms)
timer.init(period=5000, mode=machine.Timer.ONE_SHOT, callback=timeout)
```

In this example, the `timeout` function is called once after 5 seconds.

---

## Best Practices for Using Timers

When using timers in MicroPython, consider the following best practices:

- **Keep Callbacks Short and Efficient**: Ensure that the timer callback functions execute quickly to avoid delays and potential issues with timing accuracy.
- **Avoid Blocking Code in Callbacks**: Do not include blocking code (e.g., long loops, delays) in timer callbacks, as this can interfere with the timer's operation.
- **Use Global Variables Carefully**: If you need to modify global variables within a timer callback, use appropriate synchronization mechanisms to avoid race conditions.

---

### Advanced Timer Usage

For more advanced applications, you can use multiple timers, configure different modes, and combine timers with interrupts for precise control over time-sensitive tasks.

### Example: Using Multiple Timers

Here’s an example of using two timers with different intervals:

```python
import machine

# Define the first timer callback function
def timer1_callback(timer):
    print("Timer 1 Tick!")

# Define the second timer callback function
def timer2_callback(timer):
    print("Timer 2 Tick!")

# Create two timer objects
timer1 = machine.Timer(0)
timer2 = machine.Timer(1)

# Initialize the first timer to trigger the callback every second (1000 ms)
timer1.init(period=1000, mode=machine.Timer.PERIODIC, callback=timer1_callback)

# Initialize the second timer to trigger the callback every half second (500 ms)
timer2.init(period=500, mode=machine.Timer.PERIODIC, callback=timer2_callback)
```

In this example, `timer1` triggers every second, and `timer2` triggers every half second.

---

### Summary

Timers in MicroPython are powerful tools for managing time-sensitive tasks. By configuring and using one-shot and periodic timers, you can create precise and efficient programs. Following best practices ensures that your timer callbacks are efficient and do not interfere with the overall operation of your program.

---
