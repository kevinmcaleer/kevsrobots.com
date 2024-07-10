---
layout: lesson
title: PyPi & MIP
author: Kevin McAleer
type: page
cover: assets/cover.png
date: 2024-07-07
previous: 08_packages.html
next: 10_docstrings.html
description: Learn about PyPi and MIP in MicroPython and how they can enhance your
  projects.
percent: 66
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


## PyPi

In MicroPython, the Python Package Index `PyPi` (<https://www.pypi.org>) is an opensource repository of software packages that you can install to extend the functionality of your projects. PyPi contains libraries, modules, and tools that you can use in your MicroPython programs. You can also create a free account on PyPi and submit your own packages to share with the community.

---

## What is PyPi?

The Python Package Index (`PyPi`) is a repository of software packages for the Python programming language. It contains libraries, modules, and tools that you can install and use in your Python projects. PyPi is a valuable resource for developers, as it provides access to a wide range of packages that can help you build and enhance your projects.

### Benefits of Using PyPi

- **Extensive Library Collection**: PyPi hosts thousands of packages, ranging from data analysis tools to web frameworks, making it a comprehensive resource for developers.
- **Easy Installation**: Installing packages from PyPi is straightforward, typically requiring just a single command.
- **Community Support**: Many packages on PyPi are maintained by a community of developers, ensuring continuous updates and support.

### Using PyPi in MicroPython

While PyPi is primarily designed for standard Python, many packages are also compatible with MicroPython. However, due to the limited resources of microcontrollers, not all PyPi packages will work with MicroPython.

To install a package from PyPi, you would typically use the `pip` tool in standard Python. However, for MicroPython, you will often use a similar tool called `mip`.

---

## MIP

MicroPython includes the `mip` module, which can install packages from the `micropython-lib` repository and third-party sites like GitHub and GitLab. The `mip` tool is similar to Python’s `pip` but is tailored for MicroPython’s ecosystem. It does not use the PyPI index; instead, it defaults to `micropython-lib`.

### What is MIP?

`MIP` (MicroPython Install Packages) is a package manager for MicroPython. It allows you to install and manage packages specifically designed for MicroPython. This includes libraries that are optimized for the constraints of microcontrollers.

### Using MIP

You can use `mip` from the REPL to install packages. Here are some examples:

```python
import mip

# Install the latest version of "pkgname" (and dependencies)
mip.install("pkgname")

# Install a specific version of "pkgname"
mip.install("pkgname", version="x.y")

# Install the source version (i.e., .py rather than .mpy files)
mip.install("pkgname", mpy=False)
```

### Benefits of Using MIP

- **Optimized for MicroPython**: Packages are tailored for the limitations and capabilities of microcontrollers.
- **Supports Multiple Sources**: MIP can install packages from `micropython-lib` and third-party repositories like GitHub and GitLab.
- **Efficient Installation**: Automatically fetches compiled `.mpy` files when available, ensuring efficient use of memory and processing power.

---

### Summary

PyPi and MIP are essential tools for enhancing your MicroPython projects. PyPi provides a vast repository of packages for standard Python, many of which are compatible with MicroPython. MIP, on the other hand, is specifically designed for MicroPython, allowing you to install and manage packages optimized for microcontrollers. By leveraging these tools, you can significantly extend the functionality and capabilities of your MicroPython projects.

---
