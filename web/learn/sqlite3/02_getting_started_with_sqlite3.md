---
layout: lesson
title: Getting Started with Python and SQLite3
author: Kevin McAleer
type: page
cover: /learn/sqlite3/assets/database.png
date: 2023-07-15
previous: 01_introduction_to_databases.html
next: 03_creating_tables_and_schema.html
description: We will cover the initial steps to set up Python and SQLite3, establish
  a connection to a database, and perform basic operations using Python's sqlite3
  module
percent: 15
duration: 4
navigation:
- name: Create Databases with Python and SQLite3
- content:
  - section: Overview
    content:
    - name: Course Overview
      link: 00_intro.html
  - section: Getting Started
    content:
    - name: Introduction to Databases
      link: 01_introduction_to_databases.html
    - name: Getting Started with Python and SQLite3
      link: 02_getting_started_with_sqlite3.html
    - name: Creating Tables and Schema Design
      link: 03_creating_tables_and_schema.html
    - name: Inserting and Retrieving Data
      link: 04.inserting_and_retrieving_data.html
    - name: Updating and Deleting Data
      link: 05_updating_and_deleting_data.html
  - section: Advanced topics
    content:
    - name: Querying and Joining Tables
      link: 06_querying_and_joining_tables.html
    - name: Advanced SQL Queries
      link: 07_advanced_queries.html
    - name: Database Transactions and Error Handling
      link: 08_transations_and_error_handling.html
    - name: Working with Indexes and Performance Optimization
      link: 09_indexes_and_performance.html
    - name: Securing and Backing Up SQLite Databases
      link: 10_securing_and_backing_up.html
  - section: Summary and Conclusion
    content:
    - name: Course Summary
      link: 11_summary.html
  - section: bonus
    content:
    - name: First Normal Form (1NF)
      link: 13_1nf.html
    - name: Second Normal Form (2NF)
      link: 14_2nf.html
    - name: Third Normal Form (3NF)
      link: 15_3nf.html
    - name: Boyce-Codd Normal Form (BCNF)
      link: 12_boyce_codd_normal_form.html
    - name: Demo program
      link: 16_demo_program.html
    - name: Tools
      link: 17_tools.html
---


In Module 2, we will cover the initial steps to set up Python and SQLite3, establish a connection to a database, and perform basic operations using Python's `sqlite3` module.

### Setting up Python and SQLite3

Before we start working with SQLite databases in Python, we need to ensure that Python and the `sqlite3` module are installed on our system.

#### Installing Python

If Python is not already installed, you can download and install it from the official Python website (<https://www.python.org>). Choose the appropriate version for your operating system and follow the installation instructions.

#### SQLite3 and the `sqlite3` Module

Python's `sqlite3` module comes pre-installed with Python, so you don't need to install any additional packages.

### Connecting to a Database

To work with a SQLite database in Python, we first need to establish a connection to the database. The `sqlite3` module provides the necessary functions and methods to connect to a database file.

#### Creating a Connection

In Python, we use the `sqlite3.connect()` function to establish a connection to a SQLite database. We pass the path to the database file as an argument.

```python
import sqlite3

# Connect to a database
connection = sqlite3.connect('mydatabase.db')
```

This code creates a connection to a database file named `mydatabase.db`. If the file doesn't exist, SQLite will create a new database file at that location.

#### Closing the Connection

After we finish working with the database, it's important to close the connection to release system resources. We can do this by calling the `close()` method on the connection object.

```python
connection.close()
```

By closing the connection, we ensure that any changes made to the database are properly saved.

### Executing SQL Statements

Once we establish a connection to the database, we can execute SQL statements using the connection object.

#### Creating Tables

To create a table in a SQLite database, we use the `execute()` method and provide a SQL `CREATE TABLE` statement as a string.

```python
# Create a table
connection.execute('''
    CREATE TABLE books (
        id INTEGER PRIMARY KEY,
        title TEXT,
        author TEXT,
        year INTEGER
    )
''')
```

In this example, we create a table named `books` with columns for `id`, `title`, `author`, and `year`. The `id` column is defined as the primary key.

#### Inserting Data

To insert data into a table, we use the `execute()` method with an SQL `INSERT` statement.

```python
# Insert data into the table
connection.execute("INSERT INTO books (title, author, year) VALUES (?, ?, ?)", ("The Great Gatsby", "F. Scott Fitzgerald", 1925))
```

In this example, we insert a record into the `books` table, providing values for the `title`, `author`, and `year` columns.

#### Querying Data

To retrieve data from a table, we use the `execute()` method with an SQL `SELECT` statement. We then fetch the results using the `fetchall()` method.

```python
# Query data from the table
result = connection.execute("SELECT * FROM books")
data = result.fetchall()

# Process the retrieved data
for row in data:
    print(row)
```

This code retrieves all records from the `books` table and prints each row of data.

### Handling Exceptions

When working with databases, it's essential to handle potential errors and exceptions gracefully. Python's `try` and `except` statements can be used to catch and handle exceptions raised during database operations.

```python
try:
    # Database operations
    connection.execute("...")
    # ...
except sqlite3.Error as e:
    # Handle the exception
    print(f"An error occurred: {e}")
```

By incorporating exception handling, we can provide informative error messages and take appropriate actions in case of any issues.

---
