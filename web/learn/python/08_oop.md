---
layout: lesson
title: Python Object-Oriented Programming (OOP)
author: Kevin McAleer
type: page
cover: /learn/python/assets/1.png
date: 2023-07-20
previous: 07_libraries.html
next: 09_data.html
description: Introduction to object-oriented programming in Python, including classes,
  objects, methods, and inheritance.
percent: 40
duration: 3
quiz_total: 27
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

In this lesson, we will explore the principles of `object-oriented programming` (OOP) in Python. Python is an object-oriented language, and OOP concepts provide a clear structure for writing code which makes it easier to create complex applications.

---

## Learning Objectives

- Understand what `classes` and `objects` are.
- Know how to define a `class` and create an `object`.
- Understand what `methods` are and how to define them.
- Understand the concept of `inheritance`.

---

### Classes and Objects

In Python, almost everything is an `object`, with its `properties` and `methods`. A Class is like an object constructor, or a blueprint for creating objects.

```python
# Define a class
class Car:
    def __init__(self, color, brand):
        self.color = color
        self.brand = brand

# Create an object
my_car = Car('red', 'Toyota')
print(my_car.color)  # Prints 'red'
print(my_car.brand)  # Prints 'Toyota'
```

---

### Methods

`Methods` are functions defined inside the body of a class. They are used to perform operations with the attributes of our objects.

```python
class Car:
    def __init__(self, color, brand):
        self.color = color
        self.brand = brand

    # Define a method
    def honk(self):
        return "Beep beep!"

my_car = Car('red', 'Toyota')
print(my_car.honk())  # Prints 'Beep beep!'
```

---

### Inheritance

`Inheritance` is a powerful feature in object-oriented programming. It refers to defining a new class with little or no modification to an existing class. The new class is called `derived` (or child) class and the one from which it inherits is called the `base` (or parent) class.

```python
class Vehicle:
    def __init__(self, color):
        self.color = color

    def honk(self):
        return "Honk honk!"

# Car class inherits from Vehicle
class Car(Vehicle):
    def __init__(self, color, brand):
        super().__init__(color)
        self.brand = brand

my_car = Car('red', 'Toyota')
print(my_car.honk())  # Prints 'Honk honk!'
```

---

## Try it yourself

Create a `Robot` class with a `name` and a `battery` level, and a `drive()` method that reduces the battery by 10 and prints a message. Then make a `FlyingRobot` that inherits from `Robot` and adds a `fly()` method.

{% quiz %}
? What does `self` refer to inside a method?
- [ ] The class itself
- [x] The particular object the method was called on
- [ ] The parent class
> `self` is the instance, so `self.color` is that object's own colour.

? What does `super().__init__(color)` do in the `Car` class?
- [x] Runs the parent class's `__init__` so the `color` is set up
- [ ] Creates a new `Vehicle` object
- [ ] Deletes the parent class
> `super()` gives you access to the parent class, so `Car` can reuse `Vehicle`'s setup.
{% endquiz %}

---

## Summary

In this lesson, you've learned about object-oriented programming in Python. We've covered the basic principles of OOP, including classes, objects, methods, and inheritance. These concepts provide a clear structure for writing code and make it easier to develop complex Python applications.

---
