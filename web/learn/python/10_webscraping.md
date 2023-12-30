---
layout: lesson
title: Web Scraping with Python
author: Kevin McAleer
type: page
cover: /learn/python/assets/3.png
date: 2023-07-20
previous: 09_data.html
next: 11_data_visualisation.html
description: Understand the basics of web scraping using Python, including the use
  of libraries like Beautiful Soup and requests.
percent: 50
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

In this lesson, we will introduce the concept of web scraping, which is a method of extracting information from websites. Python offers several libraries for web scraping, including Beautiful Soup and requests.

---

## Learning Objectives

- Understand what web scraping is.
- Learn how to use the requests library to download web pages.
- Learn how to use the Beautiful Soup library to parse HTML and extract information.

---

### What is Web Scraping?

Web scraping is the process of extracting information directly from a web page. It involves making a request to a web page, downloading its HTML content, and parsing that content to extract the information you need.

---

### Downloading Web Pages with `requests`

The `requests` library allows you to send HTTP requests using Python. You can use it to download web pages.

```python
import requests

# Make a request to a web page
response = requests.get('https://www.example.com')

# Print the status code (200 means success)
print(response.status_code)

# Print the first 500 characters of the HTML content
print(response.text[:500])
```

---

### Parsing HTML with `Beautiful Soup`

Beautiful Soup is a library for parsing HTML and XML documents. It provides methods and Pythonic idioms for iterating, searching, and modifying the parse tree.

```python
from bs4 import BeautifulSoup
import requests

# Make a request to a web page
response = requests.get('https://www.example.com')

# Create a Beautiful Soup object
soup = BeautifulSoup(response.text, 'html.parser')

# Find the title tag
title_tag = soup.find('title')

# Print the text of the title tag
print(title_tag.text)
```

---

## Summary

In this lesson, you've learned about web scraping with Python. We've covered how to use the requests library to download web pages and the Beautiful Soup library to parse HTML and extract information. Web scraping is a powerful tool for gathering data from the internet.

---
