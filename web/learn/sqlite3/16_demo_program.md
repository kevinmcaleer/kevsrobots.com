---
layout: lesson
title: Demo program
author: Kevin McAleer
type: page
cover: /learn/sqlite3/assets/banners/16_demo_program.jpg
date: 2023-07-15
previous: 12_boyce_codd_normal_form.html
next: 17_tools.html
description: This is an example program highlighting some of the simple concepts covered
  in this course
percent: 85
duration: 4
quiz_total: 60
date_updated: 2026-11-24
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


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to read a complete SQLite program from start to finish and explain what each part does.

{% capture snakie_code %}
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

with connection:   # commits the inserts when the block ends
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
{% endcapture %}
{% include snakie.html code=snakie_code name="demo_program.py" engine="pyodide" %}

In this demo program, we create a database named "mydatabase.db" and establish a connection to it using the `sqlite3` module. We then create a table named "books" with columns for book title, author, and year. Next, we insert some sample data into the table using the `executemany()` method, inside a `with connection:` block so the rows are committed to the file.

After inserting the data, we retrieve it from the table using a SELECT statement. The `fetchall()` method returns all rows as a list, which we iterate over and display the title, author, and year of each book.

Finally, we close the database connection to ensure proper cleanup.

You can run this program to see the database creation, data insertion, and retrieval in action. The `sqlite3` module comes with Python, so there's nothing to install.

Each time you run it, the three books are added again. To stop duplicates, try adding `UNIQUE` to the `title` column and using `INSERT OR IGNORE INTO books ...` instead. Because `CREATE TABLE IF NOT EXISTS` leaves an existing table alone, delete the old `mydatabase.db` file first so the new `UNIQUE` table is created.

---

## Try it Yourself

1. Run the program twice and count the books. Why are there six?
2. Make it safe to run many times with a `UNIQUE` title and `INSERT OR IGNORE`.
3. Add a fourth book, then print the books newest first.
4. Challenge: ask the user for a book with `input()`, and add it using `?` placeholders.

## Common Issues

- **Problem**: The books appear again every time you run the program.
- **Solution**: Delete `mydatabase.db`, or make `title` `UNIQUE` and use `INSERT OR IGNORE`.
- **Why**: The `.db` file keeps its rows between runs, so each run adds the three books again.

> **Teachers**: The run box on this page uses the `pyodide` engine, so learners can try the program in the browser with nothing installed. For a real `.db` file they can open in a tool, they need to run it on their own machine as in lesson 2.

---

## Check your understanding

{% quiz %}
? Why does the demo use `executemany()` inside `with connection:`?
- [x] To insert all three rows in one call and commit them to the file
- [ ] To close the connection
- [ ] To create the table
> The `with` block commits when it finishes.

? What does the demo print for each book?
- [ ] The id, title and author
- [x] The title, author and year (`row[1]`, `row[2]`, `row[3]`)
- [ ] Only the title
> `row[0]` is the id, which the demo skips.

? Why do the books double up on a second run?
- [x] The database file already holds them, and the program inserts them again
- [ ] The commit fails
- [ ] `fetchall()` repeats rows
> Data persists in the `.db` file between runs.
{% endquiz %}

---
