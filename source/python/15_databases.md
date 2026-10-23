---
title: Python and Databases
description: Learn how Python can be used to interact with databases, including creating, reading, updating, and deleting data.
layout: lesson
cover: /learn/python/assets/1.png
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
