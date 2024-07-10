---
layout: lesson
title: Modules & Libraries
author: Kevin McAleer
type: page
cover: assets/cover.png
date: 2024-07-07
previous: 06a_decorators.html
next: 08_packages.html
description: Learn about modules and libraries in MicroPython and how they enhance
  your projects.
percent: 54
duration: 4
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


## Modules and Libraries

In MicroPython, a `module` is a file that contains Python code, including functions, classes, and variables. A `library` is a collection of modules that provide additional functionality for your projects.

---

## What is a Module?

A `module` is a file containing Python code that defines functions, classes, and variables. Modules help in organizing code and making it reusable. You can import a module into your MicroPython program to use its functions and classes.

### Example of a Module

Here’s an example of a simple module named `my_module.py`:

```python
# my_module.py

def greet(name):
    return f"Hello, {name}!"

class Calculator:
    def add(self, x, y):
        return x + y

PI = 3.14159
```

You can use this module in your main program by importing it:

```python
# main.py

import my_module

print(my_module.greet("Alice"))
calc = my_module.Calculator()
print(calc.add(5, 3))
print(my_module.PI)
```

In this example, `my_module.py` contains a function, a class, and a variable. By importing `my_module` in `main.py`, you can access and use these elements.

---

## What is a Library?

A `library` is a collection of modules that provide additional functionality to your MicroPython projects. Libraries extend the capabilities of your code by providing pre-written functions and classes.

### Using Standard Libraries

MicroPython includes several built-in libraries you can use. For example, the `math` library offers mathematical functions:

```python
import math

print(math.sqrt(16))  # Outputs: 4.0
print(math.pi)        # Outputs: 3.141592653589793
```

---

### Installing External Libraries

You can also install external libraries to add more features to your projects. This is often done using a package manager like `upip`.

For instance, to install the `urequests` library for making HTTP requests, you can use:

```python
import upip
upip.install('micropython-urequests')
```

Then, you can use the installed library in your program:

```python
import urequests

response = urequests.get('http://example.com')
print(response.text)
```

---

## Creating Your Own Modules

Creating your own modules helps you organize your code and make it reusable across different projects. Here’s how you can create and use your own module:

### Step 1: Create a Module

Create a new file named `my_robot.py` with the following content:

```python
# my_robot.py

class Robot:
    def __init__(self, name):
        self.name = name

    def greet(self):
        return f"Hello, I am {self.name}"
```

### Step 2: Use the Module in Your Main Program

In your main program, import and use the `my_robot` module:

```python
# main.py

import my_robot

r1 = my_robot.Robot("Robo")
print(r1.greet())
```

In this example, you create a module `my_robot.py` that defines a `Robot` class. In `main.py`, you import and use the `Robot` class from the `my_robot` module.

---

## Benefits of Using Modules and Libraries

- **Code Reusability**: Modules and libraries allow you to reuse code across different projects, reducing duplication and effort.
- **Organization**: They help you organize your code into manageable and logical parts.
- **Community Support**: Using libraries allows you to leverage the work of the community, saving time and effort in writing complex functionality from scratch.
- **Maintainability**: Modular code is easier to maintain and update.

---

## MIP: Installing Packages

MicroPython includes the `mip` module, which can install packages from micropython-lib and third-party sites like GitHub and GitLab. The `mip` tool is similar to Python’s pip but uses micropython-lib as its default index. It also automatically fetches compiled .mpy files when downloading from micropython-lib.

You can use `mip` from the REPL to install packages:

```python
import mip

# Install the latest version of "pkgname" (and dependencies)
mip.install("pkgname")

# Install a specific version of "pkgname"
mip.install("pkgname", version="x.y")

# Install the source version (i.e., .py rather than .mpy files)
mip.install("pkgname", mpy=False)
```

---

### Summary

Modules and libraries in MicroPython enhance your projects by providing reusable and organized code. Whether you are using built-in libraries, installing external ones, or creating your own modules, they help you build more efficient and maintainable programs.

---
