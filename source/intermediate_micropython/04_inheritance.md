---
title: Inheritance
description: Learn about inheritance and composition in object-oriented programming.
layout: lesson
type: page
cover: assets/cover.png
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
