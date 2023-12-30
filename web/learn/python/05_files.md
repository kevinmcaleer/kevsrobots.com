---
layout: lesson
title: Working with Files in Python
author: Kevin McAleer
type: page
cover: /learn/python/assets/5.png
date: 2023-07-20
previous: 04_flow.html
next: 06_errors.html
description: Learn how to read from and write to files in Python, handling both text
  and binary files.
percent: 25
duration: 3
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

Being able to interact with files is an important skill for any programmer. In this lesson, we'll learn how to read from and write to files in Python, including both text and binary files.

---

## Learning Objectives

- Understand how to open and close files.
- Understand how to read from and write to text files.
- Understand how to read from and write to binary files.

---

### Opening and Closing Files

You can open a file using the built-in `open()` function. This function returns a file object and is most commonly used with two arguments: `open(filename, mode)`. The `mode` argument is a string that contains multiple characters representing how you want to open the file.

```python
# Open a file for writing
f = open("test.txt", "w")

# Always remember to close files
f.close()
```

The most commonly used modes are:

- `'r'`: read (default)
- `'w'`: write (creates a new file or overwrite existing content)
- `'a'`: append (adds new data to the end of the file)
- `'b'`: binary mode
- `'t'`: text mode (default)
- `'+'`: read and write

---

### Reading and Writing Text Files

You can read from a text file using the `read()`, `readline()`, or `readlines()` methods, and you can write using the `write()` or `writelines()` methods.

```python
# Writing to a file
f = open("test.txt", "w")
f.write("Hello, World!")
f.close()

# Reading from a file
f = open("test.txt", "r")
content = f.read()
f.close()

print(content)  # Prints "Hello, World!"
```

---

### Reading and Writing Binary Files

You can read from and write to binary files just like text files, but you use the `'b'` mode.

```python
# Writing binary data to a file
data = bytes(range(5))
f = open("test.bin", "wb")
f.write(data)
f.close()

# Reading binary data from a file
f = open("test.bin", "rb")
data = f.read()
f.close()

print(list(data))  # Prints "[0, 1, 2, 3, 4]"
```

---

### The `with` Statement

Python provides the `with` statement to make working with files easier and cleaner. It automatically closes the file when you're done with it.

```python
with open("test.txt", "w") as f:
    f.write("Hello, World!")

with open("test.txt", "r") as f:
    print(f.read())  # Prints "Hello, World!"
```

---

## Summary

You've learned how to work with files in Python, including opening and closing files, reading from and writing to text and binary files, and using the `with` statement for better file handling.

---
