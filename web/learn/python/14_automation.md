---
layout: lesson
title: Python for Automating Tasks
author: Kevin McAleer
type: page
cover: assets/7.png
previous: 13_deep_learning.html
next: 15_databases.html
description: Learn how Python can be used to automate tasks such as file manipulation
  and web browsing.
percent: 70
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

One of the many use cases of Python is task automation. Python provides several libraries for automating tasks, such as file manipulation, sending emails, and web browsing. In this lesson, we'll explore some of the basics of task automation with Python.

---

## Learning Objectives

- Understand what task automation is.
- Learn how to automate file and directory manipulation with the os and shutil libraries.
- Learn how to automate web browsing with the selenium library.

---

### What is Task Automation?

Task automation is the use of software to reduce the manual handling of different tasks. In programming, this typically means writing software scripts that can automate repetitive tasks.

---

### File and Directory Manipulation with `os` and `shutil`

Python's built-in `os` library can be used to perform tasks such as navigating the file system, retrieving file information, and modifying file data. The `shutil` library can be used for high-level file operations.

```python
import os
import shutil

# List files and directories
print(os.listdir())

# Change the current working directory
os.chdir('/path/to/directory')
print(os.getcwd())

# Copy file
shutil.copy('source.txt', 'destination.txt')
```

---

### Web Browsing Automation with `selenium`

The `selenium` library is a powerful tool for controlling a web browser through the program. It can automate real user behaviors such as clicking and scrolling.

```python
from selenium import webdriver

# Create a new instance of the Firefox driver
driver = webdriver.Firefox()

# Go to a webpage
driver.get("http://www.google.com")

# Find the search box element
search_box = driver.find_element_by_name('q')

# Type into the search box
search_box.send_keys('Python')

# Submit the form
search_box.submit()

# Close the browser
driver.quit()
```

---

## Summary

In this lesson, you've learned about how Python can be used to automate tasks. We've covered file and directory manipulation with the os and shutil libraries, and web browsing automation with the selenium library. Task automation is a powerful way to save time and effort on repetitive tasks.

---
