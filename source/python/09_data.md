---
title: Working with Data in Python
description: Learn about Python's capabilities for data manipulation and analysis, focusing on using lists, dictionaries, and introducing Pandas.
layout: lesson
cover: /learn/python/assets/2.png
---

![cover image]({{page.cover}}){:class="cover"}

## Introduction

Python is a powerful tool for working with data. In this lesson, we will learn about Python's built-in data types and structures, as well as a brief introduction to the popular data analysis library Pandas.

---

## Learning Objectives

- Understand how to use lists and dictionaries for data manipulation.
- Learn the basics of the Pandas library for data analysis.

---

### Lists

A list is a collection which is ordered and changeable. Lists are written with square brackets.

```python
# Create a list
fruits = ["apple", "banana", "cherry"]

# Access items in a list
print(fruits[0])  # Prints 'apple'

# Modify items in a list
fruits[1] = "blueberry"
print(fruits)  # Prints '['apple', 'blueberry', 'cherry']'
```

---

### Dictionaries

A dictionary is a collection which is unordered, changeable and indexed. Dictionaries are written with curly brackets, and have keys and values.

```python
# Create a dictionary
fruit_colors = {"apple": "red", "banana": "yellow", "cherry": "red"}

# Access items in a dictionary
print(fruit_colors["apple"])  # Prints 'red'

# Modify items in a dictionary
fruit_colors["banana"] = "green"
print(fruit_colors)  # Prints '{"apple": "red", "banana": "green", "cherry": "red"}'
```

---

### Introduction to Pandas

Pandas is a powerful library for data manipulation and analysis. It provides data structures and functions needed to manipulate structured data, including functionality for manipulating tables, time series data and more.

```python
# Import the pandas library
import pandas as pd

# Create a DataFrame
df = pd.DataFrame({
    "Fruit": ["apple", "banana", "cherry"],
    "Color": ["red", "yellow", "red"],
})

# Display the DataFrame
print(df)

# Access columns in a DataFrame
print(df["Fruit"])

# Access rows in a DataFrame
print(df.loc[0])
```

---

## Summary

In this lesson, you've learned about Python's capabilities for data manipulation and analysis. We've covered lists and dictionaries, two of Python's built-in data structures, and we've introduced the powerful Pandas library for more complex data tasks. Understanding these concepts is crucial for doing data science in Python.

---
