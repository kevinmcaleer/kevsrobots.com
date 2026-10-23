---
title: Getting Started with Python and SQLite3
description: >-
   We will cover the initial steps to set up Python and SQLite3, establish a connection to a database, and perform basic operations using Python's sqlite3 module
layout: lesson
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

> ## Commit before you close
>
> Closing the connection does **not** save your changes. Changes are only saved when you call `connection.commit()`. If you close without committing, anything you inserted, updated or deleted since the last commit is thrown away. We'll see how to commit below.

### Executing SQL Statements

Once we establish a connection to the database, we can execute SQL statements using the connection object.

#### Creating Tables

To create a table in a SQLite database, we use the `execute()` method and provide a SQL `CREATE TABLE` statement as a string.

```python
# Create a table
connection.execute('''
    CREATE TABLE IF NOT EXISTS books (
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

In this example, we insert a record into the `books` table, providing values for the `title`, `author`, and `year` columns. The `?` placeholders are filled in with the values from the tuple, in order.

#### Saving changes

To save the insert to the database file, commit it:

```python
connection.commit()
```

An easier way is to use the connection in a `with` block. It commits automatically when the block finishes, or rolls back if there's an error:

```python
with connection:
    connection.execute("INSERT INTO books (title, author, year) VALUES (?, ?, ?)", ("Dune", "Frank Herbert", 1965))
```

The `with` block doesn't close the connection, so you can keep using it afterwards.

> ## Never build SQL with f-strings
>
> Always pass values with `?` placeholders, never by pasting them into the SQL string. Placeholders stop odd characters in your data, or deliberately malicious input (called SQL injection), from breaking your query.

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
    connection.execute("SELECT * FROM no_such_table")
except sqlite3.Error as e:
    # Handle the exception
    print(f"An error occurred: {e}")
```

By incorporating exception handling, we can provide informative error messages and take appropriate actions in case of any issues. This example prints `An error occurred: no such table: no_such_table`.

---

## Try it yourself

Put it all together in one script: connect, create the `books` table (use `CREATE TABLE IF NOT EXISTS` so you can run it more than once), insert three of your favourite books, commit, print them, and close. Run it, then open the `.db` file with one of the tools in the [Tools](17_tools.html) lesson to see your data.

{% quiz %}
? You insert some rows and call `connection.close()` without `commit()`. What happens to the rows?
- [ ] They're saved
- [x] They're lost
- [ ] SQLite raises an error
> Only committed changes are saved. Use `commit()` or a `with connection:` block.

? Why use `?` placeholders in SQL?
- [ ] They make the SQL shorter
- [x] They safely insert values and prevent SQL injection
- [ ] SQLite requires them for every query
> The database handles the quoting, so your data can never be mistaken for SQL.
{% endquiz %}

---
