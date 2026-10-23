---
layout: lesson
title: Database Transactions and Error Handling
author: Kevin McAleer
type: page
cover: /learn/sqlite3/assets/database.png
date: 2023-07-15
previous: 07_advanced_queries.html
next: 09_indexes_and_performance.html
description: Module 8 covers the important topics of database transactions and error
  handling in SQLite
percent: 45
duration: 4
quiz_total: 24
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


Module 8 covers the important topics of database transactions and error handling in SQLite. You will learn how to work with transactions to ensure data integrity and handle errors effectively in your Python code.

### Understanding Database Transactions

A database transaction is a logical unit of work that consists of one or more database operations. Transactions ensure that a group of database operations are executed as a single, atomic unit. If any part of the transaction fails, all changes made within the transaction can be rolled back.

The classic example is moving money between two bank accounts: taking it out of one and adding it to the other must either both happen, or neither.

#### ACID Properties

Transactions in databases follow the ACID properties:

- **Atomicity**: Transactions are indivisible and treated as a single unit. Either all changes are applied, or none of them are.
- **Consistency**: Transactions bring the database from one consistent state to another. All data modifications within a transaction must adhere to defined rules and constraints.
- **Isolation**: Transactions occur independently of each other. The changes made within a transaction are isolated from other concurrent transactions until the changes are committed.
- **Durability**: Once a transaction is committed, its changes are permanent and survive any subsequent system failures.

---

### How Python's sqlite3 handles transactions

SQLite itself uses the SQL statements `BEGIN`, `COMMIT` and `ROLLBACK`. Python's `sqlite3` module manages these for you:

- When you run an `INSERT`, `UPDATE` or `DELETE`, the module quietly starts a transaction
- Your changes are only saved when you call `connection.commit()`
- `connection.rollback()` throws away everything since the last commit

```python
import sqlite3

connection = sqlite3.connect('bank.db')
with connection:
    connection.execute("CREATE TABLE IF NOT EXISTS accounts (name TEXT PRIMARY KEY, balance REAL CHECK (balance >= 0))")
    connection.execute("INSERT OR REPLACE INTO accounts VALUES ('alice', 100), ('bob', 50)")

connection.execute("UPDATE accounts SET balance = balance - 30 WHERE name = 'alice'")
connection.execute("UPDATE accounts SET balance = balance + 30 WHERE name = 'bob'")
connection.commit()      # save both changes together
```

> ## Don't send BEGIN yourself
>
> Because the module starts transactions automatically, running `connection.execute("BEGIN")` yourself often fails with `cannot start a transaction within a transaction`. Use `commit()`, `rollback()` or a `with` block instead.

---

### The `with` block: commit or roll back automatically

The neatest way to make a group of changes all-or-nothing is a `with connection:` block. If everything works, it commits. If an exception is raised, it rolls back:

```python
def transfer(connection, sender, receiver, amount):
    try:
        with connection:
            connection.execute("UPDATE accounts SET balance = balance + ? WHERE name = ?", (amount, receiver))
            connection.execute("UPDATE accounts SET balance = balance - ? WHERE name = ?", (amount, sender))
        print("Transfer complete")
    except sqlite3.IntegrityError:
        print("Transfer cancelled: not enough money")

transfer(connection, 'bob', 'alice', 500)
print(connection.execute("SELECT * FROM accounts").fetchall())
```

Bob doesn't have 500, so the second `UPDATE` breaks the `CHECK (balance >= 0)` constraint. The `with` block rolls back, and Alice doesn't get money that never left Bob's account.

---

### Error Handling in SQLite

When working with databases, it's crucial to handle potential errors gracefully. The `sqlite3` module raises exceptions you can catch:

Exception | When it happens
---|---
`sqlite3.IntegrityError` | A constraint is broken, such as `UNIQUE`, `NOT NULL`, `CHECK` or a foreign key
`sqlite3.OperationalError` | Something's wrong with the query or the database, such as a missing table or a locked file
`sqlite3.Error` | The parent of all of them, so catching it catches every SQLite error
{:class="table table-striped"}

```python
try:
    connection.execute("SELECT * FROM missing_table")
except sqlite3.OperationalError as e:
    print(f"Query failed: {e}")
except sqlite3.Error as e:
    print(f"An error occurred: {e}")
finally:
    connection.close()
```

The `finally` block makes sure the connection is closed whatever happens.

> ## Python 3.12 and later
>
> Python 3.12 added an `autocommit` option to `sqlite3.connect()` for finer control over transactions. The default behaviour described here still works in every version.

By understanding database transactions and error handling in SQLite, you will have the knowledge to manage data changes effectively, maintain data integrity, and handle errors in your Python code.

---

{% quiz %}
? What does a `with connection:` block do if an error happens inside it?
- [ ] Commits whatever worked
- [x] Rolls back all the changes made in the block
- [ ] Closes the connection
> That's what makes the changes all-or-nothing. The connection stays open.

? Which exception is raised when a `UNIQUE` constraint is broken?
- [x] `sqlite3.IntegrityError`
- [ ] `sqlite3.OperationalError`
- [ ] `ValueError`
> Integrity errors are about the data breaking the rules you set on the table.
{% endquiz %}

---
