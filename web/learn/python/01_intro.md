---
layout: lesson
title: Introduction to Python Programming
author: Kevin McAleer
type: page
cover: /learn/python/assets/1.png
date: 2023-07-20
next: 02_basics.html
description: Understand the fundamentals of Python, install it, and write your first
  program.
percent: 5
duration: 4
quiz_total: 27
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

`Python` is a powerful and versatile programming language, suitable for various applications ranging from simple scripts to large-scale software. This lesson introduces Python programming and guides you on installing Python and writing your first program.

---

## About this course

This course takes you from your very first line of Python to building small applications. The first half covers the core language: variables, data structures, control flow, files, errors, modules and classes. The second half is a tour of what people do with Python in the real world, from web scraping and charts to machine learning, automation and databases.

Each lesson explains one topic with short examples you can run yourself, and most end with a quick quiz. Your progress and quiz score are saved in your browser and shown in the sidebar.

**What you'll need:** a computer running Windows, macOS or Linux (a Raspberry Pi is perfect), and an internet connection to download Python.

---

## Learning Objectives

- Understand why Python is a beneficial programming language to learn.
- Successfully install Python on your computer.
- Write and execute your first Python program.

---

### Why Python?

Python is one of the most popular programming languages in the world due to its simplicity and readability. It's used in various fields such as web development, data analysis, AI, and machine learning. Here are some reasons to learn Python:

1. **Simplicity**: Python's syntax is designed to be readable and straightforward, which makes it an excellent language for beginner programmers.
1. **Versatility**: From web and game development to machine learning, Python can do it all.
1. **Community**: Python has a large and active community, which means you'll find plenty of resources and answers to your coding problems.
1. **Career Opportunities**: Python developers are in demand and are often well-compensated.

---

### Installing Python

Before you can start coding in Python, you need to install the Python interpreter on your computer. Follow these steps to install Python:

1. Visit the [official Python website](https://www.python.org/downloads/).
1. Download the latest version available for your operating system.
1. Run the installer and follow the prompts to install Python. On Windows, make sure to check the box that says "Add python.exe to PATH" before completing the installation.

> ## Already installed?
>
> macOS and Linux (including Raspberry Pi OS) usually come with Python 3. Open a terminal and type `python3 --version` to check.

---

### Setting Up the Python Environment

The next step is to set up your Python development environment. You'll need a text editor or an Integrated Development Environment (IDE) to write your code. [VS Code](https://code.visualstudio.com/), [PyCharm](https://www.jetbrains.com/pycharm/) and [Thonny](https://www.thonny.org) are great options.

---

### Your First Python Program

Let's write your first Python program, a classic 'Hello, World!' example:

```python
print("Hello, World!")
```

To run this program:

1. Open your IDE or text editor.
1. Copy and paste the code into a new file.
1. Save the file with a `.py` extension, such as `hello_world.py`.
1. Open a terminal/command prompt.
1. Navigate to the directory where you saved `hello_world.py`.
1. Type `python hello_world.py` and press enter to run the program. On macOS and Linux, use `python3 hello_world.py`.

You should see `Hello, World!` printed in the terminal.

---

## Try it yourself

Change the message to print your own name, then add a second `print()` line. Each `print()` starts a new line.

{% quiz %}
? Which of these prints a message in Python?
- [ ] `echo("Hello")`
- [x] `print("Hello")`
- [ ] `console.log("Hello")`
> `print()` is Python's built-in function for showing text. The others come from other languages.

? What file extension do Python programs use?
- [ ] `.txt`
- [x] `.py`
- [ ] `.pyc`
> Python source files end in `.py`.
{% endquiz %}

---

## Summary

In this lesson, you learned why Python is an essential language to learn, how to install Python, and how to write your first Python program. Great job getting started!

---
