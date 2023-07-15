---
layout: lesson
title: Database Transactions and Error Handling
author: Kevin McAleer
type: page
cover: /learn/sqlite3/assets/database.png
previous: 07_advanced_queries.html
next: 09_indexes_and_performance.html
description: Module 8 covers the important topics of database transactions and error
  handling in SQLite
percent: 45
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


Module 8 covers the important topics of database transactions and error handling in SQLite. You will learn how to work with transactions to ensure data integrity and handle errors effectively in your Python code.

### Understanding Database Transactions

A database transaction is a logical unit of work that consists of one or more database operations. Transactions ensure that a group of database operations are executed as a single, atomic unit. If any part of the transaction fails, all changes made within the transaction can be rolled back.

#### ACID Properties

Transactions in databases follow the ACID properties:

- **Atomicity**: Transactions are indivisible and treated as a single unit. Either all changes are applied, or none of them are.
- **Consistency**: Transactions bring the database from one consistent state to another. All data modifications within a transaction must adhere to defined rules and constraints.
- **Isolation**: Transactions occur independently of each other. The changes made within a transaction are isolated from other concurrent transactions until the changes are committed.
- **Durability**: Once a transaction is committed, its changes are permanent and survive any subsequent system failures.

### Working with Transactions in SQLite

SQLite provides support for transactions through the `BEGIN`, `COMMIT`, and `ROLLBACK` statements.

#### Beginning a Transaction

To start a transaction, we use the `BEGIN` statement.

```python
# Begin a transaction
connection.execute("BEGIN")
```

By default, SQLite starts a transaction automatically for each individual SQL statement. However, explicitly beginning a transaction allows us to group multiple SQL statements into a single transaction.

#### Committing a Transaction

To commit the changes made within a transaction, we use the `COMMIT` statement.

```python
# Commit a transaction
connection.execute("COMMIT")
```

By committing a transaction, we make the changes permanent and ensure their durability.

#### Rolling Back a Transaction

If an error occurs or we want to discard the changes made within a transaction, we can roll back the transaction using the `ROLLBACK` statement.

```python
# Roll back a transaction
connection.execute("ROLLBACK")
```

Rolling back a transaction undoes any changes made within the transaction, restoring the database to its state before the transaction began.

### Error Handling in SQLite

When working with databases, it's crucial to handle potential errors gracefully. SQLite provides error handling mechanisms through Python's exception handling.

#### Catching SQLite Errors

To catch and handle SQLite errors, we can use the `try` and `except` statements in Python.

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

### Auto-commit Mode and Transactions

By default, SQLite operates in auto-commit mode, where each SQL statement is treated as a separate transaction. However, explicitly using transactions can provide additional control and ensure the integrity of related operations.

### Atomic Commit and Durability

In SQLite, changes made within a transaction are guaranteed to be atomic. When a transaction is committed, the changes are durable and survive any subsequent system failures.

By understanding database transactions and error handling in SQLite, you will have the knowledge to manage data changes effectively, maintain data integrity, and handle errors in your Python code.

---
