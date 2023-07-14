---
title: Updating and Deleting Data
description: >-
    Module 5 focuses on modifying data in SQLite tables. 
layout: lesson
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
