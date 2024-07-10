---
layout: lesson
title: Object-Oriented Programming in MicroPython
author: Kevin McAleer
type: page
cover: assets/cover.png
date: 2024-07-07
previous: 01_intro.html
next: 02a_classes.html
description: Learn how to create classes and objects in MicroPython, and the concepts
  of Object-Oriented Programming.
percent: 12
duration: 2
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


## What is Object-Oriented Programming (OOP)

`Object-Oriented` Programming is a method of programming that is based on the concept of "objects", which can contain data in the form of variables (also known as attributes or properties), and code in the form of functions (methods).

In OOP, `objects` are instances of `classes`, which are templates that define the structure and behavior of objects. Classes can inherit from other classes, which allows for code reuse and the creation of hierarchies of classes.

![Cover](assets/oop.png){:class="img-fluid w-100 card-shadow card-hover rounded-3"}

---

## OOP Concepts

There are 4 core concepts to Object-Oriented Programming in MicroPython:

- [Abstraction](03_abstraction)
- [Inheritance](04_inheritance)
- [Encapsulation](05_encapsulation)
- [Polymorphism](06_polymorphism)


We will learn about each of these concepts in the lessons that follow.

OOP has become the dominant programming paradigm in the software industry, and is widely used in many programming languages, including MicroPython. It focuses on creating reusable code that is easy to maintain and understand, through implementation of the core concepts.

OOP also lends itself to event-driven programming, where code is executed in response to events or user actions. When working with robotics and micropcontrollers this can be very useful.

---

## OOP in MicroPython

Python, and by virtue MicroPython, is an object-oriented programming language. This means that you can create classes and objects in MicroPython, and use them to organize your code and create reusable components.

Not all the concepts are fully supported in MicroPython, some might argue that polymorphism is not fully supported in MicroPython, but the core concepts of OOP are still very much applicable. We'll look into this futher in the following lessons.

---
