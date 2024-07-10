---
title: Abstraction
description: Hide complexity by exposing only the necessary details
layout: lesson
type: page
cover: assets/cover.png
---

## Abstraction

In [OOP](02_oop), `Abstraction` is the term used to describe the concept of hiding the complexity of a system by exposing only the necessary details.

We hide complexity by creating a simple interface that allows us to interact with code without needing to know the details of how it works.

This means we can also change *how* the system works without affecting the code that interacts with it, or the user knowing that anything has changed.

![Abstraction](assets/abstraction.png){:class="img-fluid rounded-3 w-100 card-shadow card-hover"}

---

## How we use Abstraction in MicroPython

We use [`classes`](02a_classes) in MicroPython to create abstractions. A class is a blueprint for creating objects that represent real-world entities.

In MicroPython we can hide ***properties*** and ***methods*** by using what is known as an `access modifier`.

---

## Access Modifiers

An ***access modifier** in MicroPython can be used to control the visibility of properties and methods in a class.

### Private Properties and Methods

We can mark a variable as `private` by using the double-underscore character `__` in front of a regular variable name, when used in a class:

```python
class Robot():
    __a_private_value = 10
```

---

### Protected Properties and Methods

We can mark a variable as `protected` by using the single-underscore character `_` in front of a regular variable name, when used in a class:

```python
class Robot():
    _a_protected_value = 20
```

***Both*** class functions and methods can be marked as `private` (hidden) or `protected` (cannot be changed) from the end user of the class.

The methods and properties are private or protected in so much that the Python interpreter and IDEs will not present these to the user either as an autocorrect or as a suggestion (depending on the editor you are using).

We can create a simple interface by using class methods and properties to interact with the code, and by using the underscore and double underscore characters to tell MicroPython to that the functions are `private` or `protected`.

```python
class Robot():
    _battery_max_voltage = 4.2
    _battery_min_voltage = 3.0
    _battery_voltage = 3.7
    __serial_number = "123456"

    def _battery_level(self):
        # This method is hidden; it calculates the battery percentage using the internal voltage ranges
        battery_percentage = ((self._battery_voltage - self._battery_min_voltage) / (self._battery_max_voltage - self._battery_min_voltage)) * 100
        return int(battery_percentage)
    
    def battery(self):
        # This method is publicly visible
        return f"{self._battery_level()}%"
    
r  = Robot() # create a robot instance
print(r.battery()) # print the battery percentage for this robot
```

In the example above, we hide the complexity of how the battery level is actually calcuated, in fact the user never needs to know what the min, max and current voltage levels are, or what the formula is to calculate the current battery level.

---

A hidden property or method cannot be ***seen*** outside of the class, and is for internal use only. A protected property or method is one that cannot be ***changed*** and accessed from outside the class.

### However...

MicroPython does not truly hide or protect these methods and properties, it just makes it harder to access them. If you really want to access them you can, but you shouldn't. MicroPython uses a technique called name mangling to obscure the names of private and protected properties and methods.

---

### Example

Imagine you have a car. You don't need to know how the engine works to drive the car. You just need to know how to use the pedals, steering wheel, and gear stick.

The car's engine is abstracted away from you. You don't need to know how it works to drive the car.

---

### Benefits

- **Simplicity**: Abstraction allows us to simplify complex systems by hiding unnecessary details.
- **Flexibility**: We can change the implementation of a system without affecting the code that interacts with it.
- **Reusability**: Abstraction allows us to reuse code by creating interfaces that can be used in different contexts.

---

[Encapsulation](05_encapsulation) and [Inheritance](04_inheritance) can also be used to create abstractions in MicroPython. We will explore these concepts in the following lessons.

---

### Summary

Abstraction is a powerful concept in OOP that allows us to hide complexity and create simple interfaces for interacting with systems. It helps us to simplify, flexibly, and reuse code in our programs.