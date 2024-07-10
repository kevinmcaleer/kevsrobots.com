---
title: DocStrings
description: Learn about DocStrings in MicroPython and how they help document your code.
layout: lesson
type: page
cover: assets/cover.png
---

## DocStrings

`DocStrings` provide a convenient way of associating documentation with Python code. They help developers understand the purpose and usage of the code without reading through the implementation. You can use the `help()` function to access the docstring of an object.

---

## What are DocStrings?

DocStrings, or documentation strings, are a special kind of comment used to describe what a function, method, class, or module does. They are written inside triple quotes (`"""` or `'''`) and are placed immediately after the definition of a function, method, class, or module.

### Example of a DocString

Here’s an example of a simple function with a DocString:

```python
def greet(name):
    """
    This function greets the person whose name is passed as a parameter.
    
    Parameters:
    name (str): The name of the person to greet.
    
    Returns:
    str: A greeting message.
    """
    return f"Hello, {name}!"
```

In this example, the DocString explains what the `greet` function does, describes its parameter, and mentions the return value.

---

## Benefits of Using DocStrings

- **Improved Readability**: DocStrings make your code more readable and understandable by providing clear documentation.
- **Ease of Use**: Developers can quickly understand how to use a function, class, or module without diving into the implementation details.
- **Consistency**: Using DocStrings ensures consistent documentation practices across your codebase.

---

## Accessing DocStrings

You can access the DocString of a function, method, class, or module using the `help()` function or the `__doc__` attribute.

### Using the `help()` Function

The `help()` function displays the DocString and other relevant information about an object.

```python
def greet(name):
    """
    This function greets the person whose name is passed as a parameter.
    
    Parameters:
    name (str): The name of the person to greet.
    
    Returns:
    str: A greeting message.
    """
    return f"Hello, {name}!"

help(greet)
```

When you run this code, it will display the DocString for the `greet` function.

### Using the `__doc__` Attribute

You can also access the DocString directly using the `__doc__` attribute of the object.

```python
print(greet.__doc__)
```

This will print the DocString of the `greet` function.

---

## Best Practices for Writing DocStrings

1. **Be Concise**: Keep your DocStrings concise but informative. Describe what the function, method, class, or module does without unnecessary details.
2. **Use Standard Conventions**: Follow standard conventions for writing DocStrings, such as PEP 257.
3. **Include Parameters and Return Values**: Clearly describe the parameters and return values, including their types.
4. **Use Triple Quotes**: Always use triple quotes for DocStrings, even if the DocString fits on one line.

### Example of a Well-Documented Class

```python
class Robot:
    """
    A class to represent a robot.
    
    Attributes:
    name (str): The name of the robot.
    speed (int): The speed of the robot.
    
    Methods:
    greet():
        Greets the user.
    set_speed(speed):
        Sets the speed of the robot.
    """

    def __init__(self, name, speed=0):
        """
        Constructs all the necessary attributes for the robot object.

        Parameters:
        name (str): The name of the robot.
        speed (int): The speed of the robot.
        """
        self.name = name
        self.speed = speed

    def greet(self):
        """
        Greets the user by printing a greeting message.
        """
        print(f"Hello, I am {self.name}")

    def set_speed(self, speed):
        """
        Sets the speed of the robot.

        Parameters:
        speed (int): The new speed of the robot.
        """
        self.speed = speed
```

In this example, the `Robot` class is well-documented with DocStrings for the class itself, the `__init__` method, and other methods.

---

### Summary

DocStrings in MicroPython provide a way to document your code, making it easier to understand and use. By following best practices for writing DocStrings, you can ensure that your code is well-documented and accessible to other developers.

---
