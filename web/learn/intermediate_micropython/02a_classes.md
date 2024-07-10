---
layout: lesson
title: Classes and Objects
author: Kevin McAleer
type: page
cover: assets/cover.png
date: 2024-07-07
previous: 02_oop.html
next: 03_abstraction.html
description: Learn how to create classes and objects in MicroPython
percent: 18
duration: 5
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
    - name: PyPi
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
---


## What is a Class?

![Classes](assets/whatisaclass.png){:class="img-fluid w-100 card-shadow card-hover rounded-3"}

A `class` is a ***blueprint*** for creating `objects`. It defines the `properties` and `methods` that an object will have. In MicroPython, you can define a class using the `class` keyword.

An `instance` of a class is also known as an `Object`. By *instance* we mean a single occurrence of an object;

```python
a = 1         # a is an instance of an integer object
b = "hello"   # b is an instance of a string object
c = [1, 2, 3] # c is an instance of a list object
```

---

## Properties and Methods

A class `Property` is the name we give to ***variables*** within a class, a class `Method` is the name we give to ***functions*** within a class.

```python
class Robot():

    robot_name = "" # this is a class property
    color = ""      # this is another class property

    def say_hello(self):
        # This is a class Method
        print(f"Hello, I am {self.robot_name}")

    def change_color(self, new_color):
        # This is another class Method
        self.color = new_color
```

---

## Classes Model the real world

We use classes to model real world things, such as a Robot, or a Car. For Example, we can create a class called `Robot` that has properties such as `robot_name` and `color`, and methods such as `say_hello` and `change_color`; things that a real world robot would have and do. This is why we refer to the properties and methods of a class, because they are like the properties and actions of a real world object.

```python
class Robot():

    robot_name = ""

    def __init__(self, name, color):
        self.robot_name = name
        self.color = color

    def say_hello(self):
        print(f"Hello, I am {self.robot_name}")

    def change_color(self, new_color):
        self.color = new_color
```

You can create multiple objects from the same class, each with its own set of properties and methods. Note the property values can be different for each object, but the methods are the same.

```python
r2d2 = Robot("R2D2", "Blue")
c3po = Robot("C3PO", "Gold")
```

---

### Naming conventions & Indentation

The convention is to name classes with an ***uppercase*** letter at the start of each word. In the example above, we have a class called `Robot`.

Notice how the ***functions*** within the class are indented, this means these functions are only accessible *within* the class, or via the dot "`.`" operator. We call these functions `methods`.

Also notice that there is a variable called `robot_name` within the class, this is a `class variable`; It is shared by all instances of the class.

![Blueprints](assets/blueprints.png){:class="img-fluid w-100 card-shadow card-hover rounded-3"}

---

## What is an Object?

In Python, and MicroPython, an `object` is an *instance* of a class. It is created using the *class* as a blueprint.

![Objects](assets/objects.png){:class="img-fluid w-100 card-shadow card-hover rounded-3"}

An **object** is like a variable that also has functions *attached* to it. These functions are the `class functions`, or `methods` to use the correct terminology.

You can call the **methods** of an object using the dot "`.`" operator.

```python

# Create an object of the Robot class
r1 = Robot("R2D2", "Blue")

# Call the say_hello method
r1.say_hello()

# Change the color of the robot
r1.change_color("Red")

# Access the color of the robot

print(r1.color)

```

---

## Everything in Python is an Object

Well, mostly everything - all the entities such as `int`, `float`, `list`, `dict`, `str`, etc., are objects in Python. This means they have properties and methods that you can access.

You may value already used objects and their methods without realising it. For example, the `str` object has a method called `upper()` that converts the string to uppercase.

```python
message = "hello" # create a string called message and store the value "hello"
print(message.upper()) # prints out the message string in uppercase, this uses the str objects upper() method
```

---

## What is Self?

In the `__init__` method and other class methods, you will see a parameter called `self`. This is a reference to the current instance of the class. It is used to access the variables and methods of the class.

`self` is always the first parameter in a class method; it cannot be omitted.

```python

class Robot():

    # Class Proprties
    robot_name = "" # this is a class property
    color = ""      # this is another class property

    def __init__(self, name, color):
        self.robot_name = name # changes this objects robot_name to the name provided
        self.color = color     # changes this objects color to the color provided

    def change_color(self, new_color):
        # This is a class function, or Method
        self.color = new_color
```

> ## Did you know
>
> "`self`" can actually have ***any*** name, but it is a convention to use "`self`" in Python.
>
> You could use "`this`" or "`me`" or any other name, but it is best to stick with "`self`" to avoid confusion. As long as the first parameter of a class method is the reference to the current instance of the class, it will work.

---
