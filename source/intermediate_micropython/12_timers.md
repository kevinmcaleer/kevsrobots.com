---
title: Timers
description: Learn about timers in MicroPython and how to use them for time-sensitive applications.
layout: lesson
type: page
cover: assets/cover.png
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
