---
title: Python Data Structures
description: Discover the primary data structures in Python, including strings, lists, tuples, dictionaries, and sets.
layout: lesson
cover: /learn/python/assets/3.png
---

![cover image]({{page.cover}}){:class="cover"}

## Introduction

Data structures in Python are containers that organize and group data types together in different ways. This lesson will cover the primary data structures: `strings`, `lists`, `tuples`, `dictionaries`, and `sets`.

---

## Learning Objectives

- Understand and use Python's primary data structures: strings, lists, tuples, dictionaries, and sets.

---

### Strings

`Strings` are sequences of characters, and they are created by enclosing characters in quotes. Python treats single quotes the same as double quotes[^1].

[^1]: It's best practice to use double quotes, as these anticipate punctuation used in strings such as appostrophies. It's also part of the [Black](18_tips_and_tricks#pep-8-and-the-black-code-formatter) Python coding best practice

```python
# A string in Python
greeting = "Hello, World!"

# Accessing characters in a string
first_char = greeting[0]
print(first_char)  # Prints 'H'

# Strings are immutable
# The following line will raise an error
# greeting[0] = 'h'
```

---

### Lists

`Lists` are ordered and mutable collections, which means you can replace, add or remove elements. Lists can contain items of different data types.

```python
# Creating a list
numbers = [1, 2, 3, 4, 5]

# Accessing list items
first_number = numbers[0]
print(first_number)  # Prints '1'

# Modifying a list
numbers[0] = 10
print(numbers)  # Prints '[10, 2, 3, 4, 5]'
```

---

### Tuples

`Tuples` are similar to lists but they are immutable, which means you can't change elements of a tuple once it's defined.

```python
# Creating a tuple
coordinates = (10.0, 20.0)

# Accessing tuple items
x_coordinate = coordinates[0]
print(x_coordinate)  # Prints '10.0'

# Tuples are immutable
# The following line will raise an error
# coordinates[0] = 20.0
```

---

### Dictionaries

`Dictionaries` are unordered collections of key-value pairs. They are mutable and indexed by keys.

```python
# Creating a dictionary
student = {'name': 'John', 'age': 15, 'grade': 'A'}

# Accessing dictionary values
name = student['name']
print(name)  # Prints 'John'

# Modifying a dictionary
student['age'] = 16
print(student)  # Prints "{'name': 'John', 'age': 16, 'grade': 'A'}"
```

---

### Sets

`Sets` are unordered collections of unique elements. They are mutable, but they cannot contain mutable elements.

```python
# Creating a set
fruits = {'apple', 'banana', 'cherry'}

# Checking if an element is in the set
print('apple' in fruits)  # Prints 'True'

# Adding an element to the set
fruits.add('orange')
print(fruits)  # May print "{'cherry', 'orange', 'banana', 'apple'}"

# Removing an element from the set
fruits.remove('apple')
print(fruits)  # May print "{'cherry', 'orange', 'banana'}"
```

---

## Summary

You've learned about Python's main data structures and how to work with them. These structures are fundamental to organizing and storing data in your Python programs.

---
