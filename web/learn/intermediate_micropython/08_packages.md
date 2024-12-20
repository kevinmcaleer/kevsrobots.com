---
layout: lesson
title: Packages
author: Kevin McAleer
type: page
cover: assets/cover.png
date: 2024-07-07
previous: 07_modules.html
next: 09_pypi.html
description: Learn about packages in MicroPython and how they help organize and reuse
  code.
percent: 60
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


## Packages

In MicroPython, a `package` is a collection of modules that are organized in a directory structure. Packages help in organizing code and making it reusable. A package can contain sub-packages, modules, and other resources.

---

## What is a Package?

A `package` is a directory containing a special file named `__init__.py`. This file can be empty or contain initialization code for the package. The presence of the `__init__.py` file indicates to Python that the directory should be treated as a package.

### Structure of a Package

Here’s an example of a simple package structure:

```
my_package/
    __init__.py
    module1.py
    module2.py
    sub_package/
        __init__.py
        module3.py
```

In this example, `my_package` is a package that contains two modules (`module1.py` and `module2.py`) and a sub-package (`sub_package`) which itself contains an `__init__.py` file and a module (`module3.py`).

---

## Creating and Using Packages

### Step 1: Create a Package

Create a directory for your package. Inside this directory, create an `__init__.py` file and the modules you need. For example, create the following structure:

```
my_robot_package/
    __init__.py
    motors.py
    sensors.py
```

### Step 2: Add Code to Modules

Add the following code to `motors.py`:

```python
# motors.py

class Motor:
    def __init__(self, power):
        self.power = power

    def move(self, direction):
        print(f"Moving {direction} with power {self.power}")
```

And add the following code to `sensors.py`:

```python
# sensors.py

class Sensor:
    def __init__(self, type):
        self.type = type

    def read_value(self):
        # Simulate reading a sensor value
        return 42
```

### Step 3: Use the Package in Your Main Program

Create a main program file and import the package modules:

```python
# main.py

from my_robot_package.motors import Motor
from my_robot_package.sensors import Sensor

motor = Motor(100)
motor.move("forward")

sensor = Sensor("Ultrasonic")
print(sensor.read_value())
```

In this example, you create a package `my_robot_package` with two modules: `motors.py` and `sensors.py`. The main program imports and uses the classes from these modules.

---

## Benefits of Using Packages

- **Code Organization**: Packages help you organize your code into a structured directory, making it easier to navigate and manage.
- **Reusability**: Packages allow you to reuse code across different projects, reducing duplication and effort.
- **Namespace Management**: Packages provide a way to manage namespaces, avoiding conflicts between module names in different packages.

---

## Best Practices for Using Packages

1. **Meaningful Names**: Use meaningful names for your packages and modules to make your code more readable.
2. **Keep `__init__.py` Minimal**: Keep the `__init__.py` file minimal to avoid unnecessary complexity.
3. **Document Your Code**: Add documentation to your packages and modules to help other developers understand and use your code.

---

### Summary

Packages in MicroPython are directories containing modules and an `__init__.py` file. They help organize and reuse code by grouping related modules together. By creating and using packages, you can improve the structure, maintainability, and reusability of your code.

---
