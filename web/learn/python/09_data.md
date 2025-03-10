---
layout: lesson
title: Working with Data in Python
author: Kevin McAleer
type: page
cover: /learn/python/assets/2.png
date: 2023-07-20
previous: 08_oop.html
next: 10_webscraping.html
description: Learn about Python's capabilities for data manipulation and analysis,
  focusing on using lists, dictionaries, and introducing Pandas.
percent: 45
duration: 2
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
