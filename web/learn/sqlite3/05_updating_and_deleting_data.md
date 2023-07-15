---
layout: lesson
title: Updating and Deleting Data
author: Kevin McAleer
type: page
cover: /learn/sqlite3/assets/database.png
previous: 04.inserting_and_retrieving_data.html
next: 06_querying_and_joining_tables.html
description: 'Module 5 focuses on modifying data in SQLite tables. '
percent: 30
duration: 3
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


Module 5 focuses on modifying data in SQLite tables. You will learn how to update existing records and delete data from tables using SQL statements.

### Updating Data

To modify existing data in a SQLite table, we use the SQL `UPDATE` statement. This statement allows us to specify the table name, the columns to update, and the new values.

```python
# Update a record in the table
connection.execute("UPDATE books SET title = ? WHERE id = ?", ("New Title", 1))
```

In this example, we update the title of a book with an ID of 1 in the "books" table.

### Deleting Data

To remove data from a SQLite table, we use the SQL `DELETE` statement. This statement allows us to specify the table name and the conditions for deleting records.

```python
# Delete records from the table
connection.execute("DELETE FROM books WHERE author = ?", ("F. Scott Fitzgerald",))
```

In this example, we delete records from the "books" table where the author is "F. Scott Fitzgerald."

### Batch Updates and Deletes

SQLite allows us to perform batch updates and deletes using the `executemany()` method. This method allows us to execute the same SQL statement multiple times with different parameter values.

```python
# Batch update
data = [("New Title 1", 1), ("New Title 2", 2), ("New Title 3", 3)]
connection.executemany("UPDATE books SET title = ? WHERE id = ?", data)
```

In this example, we perform a batch update on the "books" table, updating the titles of multiple books based on their IDs.

### Handling Cascading Deletes and Referential Integrity

SQLite supports referential integrity, which means that you can define relationships between tables using foreign keys. When a record in a parent table is deleted, you can specify the desired behavior for associated records in child tables.

```python
# Create tables with foreign key relationship
connection.execute('''
    CREATE TABLE artists (
        id INTEGER PRIMARY KEY,
        name TEXT
    )
''')

connection.execute('''
    CREATE TABLE albums (
        id INTEGER PRIMARY KEY,
        title TEXT,
        artist_id INTEGER,
        FOREIGN KEY (artist_id) REFERENCES artists(id) ON DELETE CASCADE
    )
''')
```

In this example, we create two tables: "artists" and "albums." The "albums" table has a foreign key relationship with the "artists" table. The `ON DELETE CASCADE` clause ensures that when an artist is deleted, all associated albums are also deleted.

By understanding how to update and delete data, as well as handle cascading deletes and referential integrity, you will have the tools to effectively modify and manage data in SQLite tables.

---
