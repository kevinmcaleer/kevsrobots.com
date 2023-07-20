---
layout: lesson
title: Data Visualization
author: Kevin McAleer
type: page
cover: assets/4.png
previous: 10_webscraping.html
next: 12_ml.html
description: Explore the basics of data visualization in Python, focusing on using
  Matplotlib and Seaborn libraries.
percent: 55
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

Data visualization is an important part of data analysis. Python offers multiple libraries for creating static, animated, and interactive visualizations, including Matplotlib and Seaborn. In this lesson, we'll explore the basics of these two libraries.

---

## Learning Objectives

- Understand what data visualization is and its importance.
- Learn how to create basic plots using Matplotlib.
- Learn how to create statistical plots using Seaborn.

---

### What is Data Visualization?

Data visualization is the graphical representation of data and information. It uses visual elements like charts, graphs, and maps to provide an easy way to understand trends, outliers, and patterns in data.

---

### Introduction to Matplotlib

Matplotlib is a plotting library for Python. It provides an object-oriented API for embedding plots into applications.

```python
import matplotlib.pyplot as plt

# Create a simple line plot
plt.plot([1, 2, 3, 4])
plt.ylabel('Some Numbers')
plt.show()
```

---

### Introduction to Seaborn

Seaborn is a Python data visualization library based on Matplotlib. It provides a high-level interface for creating attractive graphics.

```python
import seaborn as sns

# Load an example dataset
tips = sns.load_dataset("tips")

# Create a simple histogram
sns.histplot(data=tips, x="total_bill")
```

---

## Summary

In this lesson, you've learned about the basics of data visualization in Python. We've covered the importance of data visualization and explored two Python libraries, Matplotlib and Seaborn, for creating static visualizations. Data visualization is a powerful tool for understanding data and communicating results.

---
