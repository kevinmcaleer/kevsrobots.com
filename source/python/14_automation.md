---
title: Python for Automating Tasks
description: Learn how Python can be used to automate tasks such as file manipulation and web browsing.
layout: lesson
cover: /learn/python/assets/7.png
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
