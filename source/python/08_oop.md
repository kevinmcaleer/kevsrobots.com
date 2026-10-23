---
title: Python Object-Oriented Programming (OOP)
description: Introduction to object-oriented programming in Python, including classes, objects, methods, and inheritance.
layout: lesson
cover: /learn/python/assets/1.png
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
