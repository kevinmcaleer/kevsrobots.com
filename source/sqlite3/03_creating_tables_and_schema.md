---
title: Creating Tables and Schema Design
description: >-
   Creating tables in SQLite databases and understanding the principles of schema design. We will explore how to define table structure, data types, constraints, and primary keys
layout: lesson
---

Module 3 focuses on creating tables in SQLite databases and understanding the principles of schema design. We will explore how to define table structure, data types, constraints, and primary keys.

### Understanding Table Structure

Tables are the foundation of a relational database. They organize and store data in a structured manner. In this section, we will explore the components of table structure.

#### Rows (Records) and Columns

A table consists of rows, also known as records or tuples, and columns, also known as fields or attributes.

- **Rows**: Each row represents a distinct record or entry in the table. For example, in a "Books" table, each row might represent a specific book with information like the title, author, and publication year.
- **Columns**: Columns define the specific types of data that can be stored in a table. Each column corresponds to a specific attribute or piece of information about the records. For instance, in the "Books" table, columns might include "Title," "Author," and "Publication Year."

### Creating Tables

To create tables in a SQLite database, we use the SQL `CREATE TABLE` statement. This statement specifies the table name, column names, data types, and constraints.

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

In this example, we create a table named "books" with four columns: "id," "title," "author," and "year." The `INTEGER` data type is used for the "id" and "year" columns, while `TEXT` is used for the "title" and "author" columns. The "id" column is defined as the primary key.

### Defining Primary Keys

A primary key uniquely identifies each record in a table. It ensures data integrity and serves as a reference point for establishing relationships between tables. In SQLite, primary keys can be defined using the `PRIMARY KEY` constraint.

```python
# Create a table with a primary key
connection.execute('''
    CREATE TABLE students (
        id INTEGER PRIMARY KEY,
        name TEXT,
        age INTEGER,
        grade TEXT
    )
''')
```

In this example, we create a "students" table with an "id" column as the primary key. Each record in the table will have a unique "id" value.

### Modifying Tables

Sometimes, we need to modify the structure of existing tables. SQLite provides the `ALTER TABLE` statement to add, modify, or delete columns from a table.

```python
# Add a new column to a table
connection.execute("ALTER TABLE students ADD COLUMN email TEXT")
```

In this example, we add a new "email" column to the "students" table. This new column will store email addresses for each student.

### Schema Design Considerations

When designing a database schema, it's important to consider several factors to ensure efficient data storage and retrieval. Some key considerations include:

- **Normalization**: Organize data into tables and reduce redundancy.
- **Data Types**: Choose appropriate data types for columns to optimize storage and enforce data integrity.
- **Constraints**: Define constraints (e.g., `NOT NULL`, `UNIQUE`) to maintain data integrity and consistency.
- **Relationships**: Establish relationships between tables using foreign keys to represent connections between entities.
- **Indexing**: Use indexes to improve query performance on frequently searched columns.

By understanding these considerations, we can design efficient and well-structured databases.

---
