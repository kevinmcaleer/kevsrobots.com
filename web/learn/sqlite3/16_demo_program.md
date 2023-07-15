---
layout: lesson
title: Demo program
author: Kevin McAleer
type: page
cover: /learn/sqlite3/assets/database.png
previous: 12_boyce_codd_normal_form.html
next: 17_tools.html
description: This is an example program highlighting some of the simple concepts covered
  in this course
percent: 85
duration: 2
navigation:
- name: Creating Databases with Python and SQLite3 for Beginners
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
    - name: Desktop Apps for managing SQlite3 databases
      link: 17_tools.html
---


```python
import sqlite3

# Create a database and establish a connection
connection = sqlite3.connect('mydatabase.db')

# Create a table
connection.execute('''
    CREATE TABLE IF NOT EXISTS books (
        id INTEGER PRIMARY KEY,
        title TEXT,
        author TEXT,
        year INTEGER
    )
''')

# Insert data into the table
books_data = [
    ("The Great Gatsby", "F. Scott Fitzgerald", 1925),
    ("To Kill a Mockingbird", "Harper Lee", 1960),
    ("Pride and Prejudice", "Jane Austen", 1813)
]

connection.executemany('INSERT INTO books (title, author, year) VALUES (?, ?, ?)', books_data)

# Query data from the table
result = connection.execute('SELECT * FROM books')
data = result.fetchall()

# Display the retrieved data
for row in data:
    print(f"Title: {row[1]}")
    print(f"Author: {row[2]}")
    print(f"Year: {row[3]}")
    print()

# Close the database connection
connection.close()
```

In this demo program, we create a database named "mydatabase.db" and establish a connection to it using the `sqlite3` module. We then create a table named "books" with columns for book title, author, and year. Next, we insert some sample data into the table using the `executemany()` method.

After inserting the data, we retrieve it from the table using a SELECT statement. The `fetchall()` method returns all rows as a list, which we iterate over and display the title, author, and year of each book.

Finally, we close the database connection to ensure proper cleanup.

You can run this program to see the database creation, data insertion, and retrieval in action. Make sure to have the `sqlite3` module installed and adjust the database file path as needed.

---
