---
layout: lesson
title: Inheritance
author: Kevin McAleer
type: page
cover: assets/cover.png
date: 2024-07-07
previous: 03_abstraction.html
next: 05_encapsulation.html
description: Learn about inheritance and composition in object-oriented programming.
percent: 30
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


## Composition vs inheritance

Inheritance is a way to form new classes using classes that have already been defined. The new classes, known as derived classes, inherit attributes and methods from the classes that are used to create them, known as base classes. This is a powerful feature of object-oriented programming.

Composition on the other hand, is a way to combine objects or classes together. It is used to represent a has-a relationship. For example, a car has an engine, a person has a heart, etc.

We use composition in our projects to create objects that are made up of other objects. This allows us to create complex objects that are made up of simpler objects. For example if we want to create a robot we could create a class for the robot and then create objects for the sensors, motors, and other components that make up the robot.

```python
class Robot:
    
        def __init__(self, name):
            self.name = name
            self.sensors = Sensors()       # create a class property for sensors, which are defined elsewhere
            self.motors = Motors()         # create a class property for motors, which are defined elsewhere
            self.controller = Controller() # create a class property for controller, which are defined elsewhere
    
        def move_forward(self):
            self.motors.move_forward()
    
        def move_backward(self):
            self.motors.move_backward()
    
        def turn_left(self):
            self.motors.turn_left()
    
        def turn_right(self):
            self.motors.turn_right()
    
        def read_sensor(self):
            return self.sensors.read_sensor()
    ```
