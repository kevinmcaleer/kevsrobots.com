---
layout: lesson
title: Decorators
author: Kevin McAleer
type: page
cover: assets/cover.png
date: 2024-07-07
previous: 06_polymorphism.html
next: 07_modules.html
description: Learn about decorators in MicroPython and how they extend the functionality
  of functions and methods.
percent: 48
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


## Decorators

In MicroPython, a ***decorator*** is a function that takes another function as an argument and extends its functionality without modifying it.

Decorators are a powerful tool that allows us to add functionality to a function without changing its code. They are a way to ***wrap*** a function, modifying its behavior.

![Decorators](assets/decorators.png){:class="img-fluid rounded-3 w-100 card-shadow card-hover"}

---

## How Decorators Work

Decorators are a way to wrap a function, modifying its behavior. They take another function as an argument and extend its functionality without modifying the original function.

Here is a simple example of a decorator:

```python
def my_decorator(func):
    def wrapper():
        print("Something is happening before the function is called.")
        func()
        print("Something is happening after the function is called.")
    return wrapper

def say_hello():
    print("Hello!")

say_hello = my_decorator(say_hello)
say_hello()
```

In the example above, we define a decorator `my_decorator` that takes a function `func` as an argument. The `wrapper` function is used to wrap the `func` function, adding functionality before and after the function is called.

We then apply the decorator to the `say_hello` function by reassigning `say_hello` to the result of calling `my_decorator(say_hello)`.

![Inner functions](assets/innerfunctions.png){:class="img-fluid rounded-3 w-100 card-shadow card-hover"}

---

## Using the `@` Symbol

In Python, we can use the `@` symbol to apply a decorator to a function. This is a more concise way of applying a decorator.

Here is the same example using the `@` symbol:

```python
def my_decorator(func):
    def wrapper():
        print("Something is happening before the function is called.")
        func()
        print("Something is happening after the function is called.")
    return wrapper

@my_decorator
def say_hello():
    print("Hello!")

say_hello()
```

The `@my_decorator` syntax is a shorthand for applying the decorator to the function.

---

## Setters and Getters

In MicroPython, we can use decorators to create ***setters*** and ***getters*** for class properties. Setters and getters control access to class properties, ensuring values are valid and within a specified range.

Here is an example of a class with a setter and getter:

```python
class Robot:
    def __init__(self, name):
        self.name = name
        self.__speed = 0

    @property
    def speed(self):
        return self.__speed

    @speed.setter
    def speed(self, value):
        if value < 0:
            self.__speed = 0
        elif value > 100:
            self.__speed = 100
        else:
            self.__speed = value

r = Robot("Robbie")
r.speed = 50
print(r.speed)  
r.speed = 150
print(r.speed)  # notice the speed was capped at 100
```

In this example, the `@property` decorator is used to define a getter method for the `speed` attribute, and the `@speed.setter` decorator is used to define a setter method.

![properties](assets/properties.png){:class="img-fluid rounded-3 w-100 card-shadow card-hover"}

> ## Don't Go Crazy with Property Decorators
>
> There is a temptation to use property decorators for every class property. This is not necessary and can make your code harder to read. Only use property decorators when you need to control access to a class property.

---

### Benefits of Decorators

- **Code Reusability**: Decorators allow you to reuse common functionality across different functions and methods.
- **Code Organization**: They help in organizing code by separating concerns, keeping core logic clean.
- **Enhanced Functionality**: Decorators add additional behavior to functions and methods without modifying their code.

---

### Summary

Decorators in MicroPython are a powerful feature that allows you to extend the functionality of functions and methods without modifying their code. By using decorators, you can create more reusable, organized, and maintainable code.

---
