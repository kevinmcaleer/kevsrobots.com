---
layout: lesson
title: Python and Databases
author: Kevin McAleer
type: page
cover: assets/1.png
previous: 14_automation.html
next: 16_advanced.html
description: Learn how Python can be used to interact with databases, including creating,
  reading, updating, and deleting data.
percent: 75
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

Python provides several libraries to interact with databases. Whether you are dealing with a relational database like MySQL or PostgreSQL, or a NoSQL database like MongoDB, Python has you covered. This lesson will introduce some of the basic concepts in database manipulation using Python.

---

## Learning Objectives

- Understand what databases are and the different types of databases.
- Learn how to use the sqlite3 module to interact with a SQLite database.

---

### What are Databases?

A database is an organized collection of data stored and accessed electronically. Databases can be classified based on types of content: bibliographic, full-text, numeric, and images.

---

### Introduction to SQLite

SQLite is a C library that provides a lightweight disk-based database. It doesn't require a separate server process and allows accessing the database using a nonstandard variant of the SQL query language. Python provides the sqlite3 module which complies with the DB-API 2.0 specification.

> ## Learn More about SQLite
>
> Learn more about SQLite in our dedicated [SQLite3 course](/learn/sqlite3/)

```python
import sqlite3

# Connect to SQLite database
conn = sqlite3.connect('example.db')

# Create a cursor object
c = conn.cursor()

# Create table
c.execute('''CREATE TABLE stocks
             (date text, trans text, symbol text, qty real, price real)''')

# Insert a row of data
c.execute("INSERT INTO stocks VALUES ('2006-01-05','BUY','RHAT',100,35.14)")

# Save (commit) the changes
conn.commit()

# Close the connection
conn.close()
```

---

## Summary

In this lesson, you've learned about the basics of databases and how Python can be used to interact with them. We've covered the sqlite3 module for interacting with SQLite databases. Understanding databases and knowing how to manipulate them with Python is a crucial skill in many areas of software and data engineering.

---
