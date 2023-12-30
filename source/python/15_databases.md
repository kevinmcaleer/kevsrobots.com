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
