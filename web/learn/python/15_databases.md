---
layout: lesson
title: Python and Databases
author: Kevin McAleer
type: page
cover: /learn/python/assets/1.png
date: 2023-07-20
previous: 14_automation.html
next: 16_advanced.html
description: Learn how Python can be used to interact with databases, including creating,
  reading, updating, and deleting data.
percent: 75
duration: 3
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

Python provides several libraries to interact with databases. Whether you are dealing with a relational database like MySQL or PostgreSQL, or a NoSQL database like MongoDB, Python has you covered. This lesson will introduce some of the basic concepts in database manipulation using Python.

---

## Learning Objectives

- Understand what databases are and the different types of databases.
- Learn how to use the sqlite3 module to interact with a SQLite database.

---

### What are Databases?

A database is an organized collection of data stored and accessed electronically. There are two main families:

- **Relational (SQL) databases** like SQLite, MySQL and PostgreSQL store data in tables of rows and columns, and you query them with SQL
- **NoSQL databases** like MongoDB and Redis store data as documents, key-value pairs or other shapes

---

### Introduction to SQLite

SQLite is a lightweight database that keeps everything in a single file. It doesn't need a separate server, and Python comes with the `sqlite3` module built in, so there's nothing to install.

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
c.execute('''CREATE TABLE IF NOT EXISTS stocks
             (date text, trans text, symbol text, qty real, price real)''')

# Insert a row of data, using ? placeholders for the values
c.execute("INSERT INTO stocks VALUES (?, ?, ?, ?, ?)", ('2006-01-05', 'BUY', 'RHAT', 100, 35.14))

# Save (commit) the changes
conn.commit()

# Read the data back
for row in c.execute("SELECT * FROM stocks"):
    print(row)

# Close the connection
conn.close()
```

---

> ## Always use placeholders
>
> Use `?` placeholders rather than building SQL with f-strings. Placeholders stop odd characters in your data (or deliberately malicious ones, called SQL injection) from breaking your query.

---

{% quiz %}
? Why use `?` placeholders in SQL queries?
- [ ] They make queries run faster
- [x] They safely insert values and prevent SQL injection
- [ ] SQLite doesn't support any other way
> Placeholders let the database handle quoting, so data can't be mistaken for SQL.
{% endquiz %}

---

## Summary

In this lesson, you've learned about the basics of databases and how Python can be used to interact with them. We've covered the sqlite3 module for interacting with SQLite databases. Understanding databases and knowing how to manipulate them with Python is a crucial skill in many areas of software and data engineering.

---
