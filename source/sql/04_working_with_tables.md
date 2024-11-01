---
title: Working with Tables and Data Types
description: >-
    How to create tables and define data types, with an introduction to primary keys and constraints
layout: lesson
---

In this lesson, we’ll explore how to structure data in tables and choose appropriate data types for each column. You’ll also learn about primary keys and constraints, essential concepts for organizing and securing data in a relational database.

---

## 1. Understanding Tables in SQL

In a relational database, data is organized into **tables**. A table represents a specific entity or concept, like "Users" or "Products." Each row in the table represents a unique instance of that entity (e.g., a specific user), while each column represents a characteristic or attribute of that entity (e.g., name, age, or email).

### Example Table Structure

Here’s an example of a simple table called `users`:

```plaintext
| id | name       | age | email              |
|----|------------|-----|--------------------|
| 1  | Alice      | 30  | alice@example.com  |
| 2  | Bob        | 25  | bob@example.com    |
```

In this table:

- **id** uniquely identifies each user.
- **name**, **age**, and **email** store specific details about each user.

---

## 2. Creating a Table with `CREATE TABLE`

The `CREATE TABLE` statement defines the structure of a new table, specifying each column’s name and data type.

### Basic Syntax

```sql
CREATE TABLE table_name (
    column1 data_type constraints,
    column2 data_type constraints,
    ...
);
```

### Example: Creating a `users` Table

Here’s how to create the `users` table with columns for `id`, `name`, `age`, and `email`.

```sql
CREATE TABLE users (
    id INTEGER PRIMARY KEY,
    name TEXT NOT NULL,
    age INTEGER,
    email TEXT
);
```

- **id**: An integer that uniquely identifies each user. It’s designated as the `PRIMARY KEY`.
- **name**: A text field that cannot be null (`NOT NULL`), ensuring each user has a name.
- **age**: An integer storing the user’s age. No constraints are applied, so this column is optional.
- **email**: A text field storing the user’s email address.

> **Note**: Constraints like `PRIMARY KEY` and `NOT NULL` help ensure data integrity. We’ll discuss them in more detail below.

---

## 3. Choosing Data Types

SQL provides various data types to represent different types of data. Choosing the correct data type for each column is essential for efficient storage and data integrity.

### Common Data Types

Type          | Description                                                              | Example
--------------|--------------------------------------------------------------------------|----------------------
**INTEGER**   | Whole numbers without decimal places                                     | `0`, `1`, `2` ,`3`
**REAL**      | Floating-point numbers                                                   | `3.14`, `2.718`
**TEXT**      | Alphanumeric characters and strings                                      | `"Alice"`
**BLOB**      | Binary Large Object, for storing binary data like images or files        | `[0x00,0x01,x010]`
**DATE**      | Stores date values                                                       | `2023-07-14`
**BOOLEAN**   | True or false values                                                     | `1` for true, `0` for false
{:class="table table-striped"}

### Data Type Examples in SQL

- **INTEGER** for columns storing whole numbers, like `age`.
- **TEXT** for columns storing alphanumeric data, like `name` or `email`.
- **REAL** for decimal numbers, like a `price` or `salary`.
- **DATE** for date values, like `birth_date` or `hire_date`.

> **Tip**: Choose the most specific data type to save space and improve performance.

---

## 4. Adding Constraints

Constraints are rules applied to columns to enforce **data integrity**. They help ensure that the data is ***accurate***, ***consistent***, and ***meaningful***.

### Common Constraints

Constraint      | Description
----------------|------------------------------------------------------------
**PRIMARY KEY** | Uniquely identifies each row in a table. Often set on an `id` column.
**NOT NULL**    | Ensures a column cannot be empty. Useful for mandatory fields like `name`.
**UNIQUE**      | Ensures all values in a column are unique. Useful for fields like `email` or `username`.
**DEFAULT**     | Sets a default value for a column if no value is specified.
**FOREIGN KEY** | Links one table to another, establishing relationships between tables.
{:class="table table-striped"}

---

### Adding Constraints in SQL

- **PRIMARY KEY**: Ensures each row has a unique identifier.
    ```sql
    id INTEGER PRIMARY KEY
    ```
- **NOT NULL**: Requires the column to have a value.
    ```sql
    name TEXT NOT NULL
    ```
- **UNIQUE**: Ensures all entries in the column are distinct.
    ```sql
    email TEXT UNIQUE
    ```

### Example Table with Constraints

```sql
CREATE TABLE users (
    id INTEGER PRIMARY KEY,
    name TEXT NOT NULL,
    age INTEGER DEFAULT 18,
    email TEXT UNIQUE
);
```

In this example:

- `id` is a primary key, ensuring each user has a unique identifier.
- `name` cannot be null, so every user must have a name.
- `age` defaults to 18 if no value is provided.
- `email` must be unique across all users.

---

## 5. Using `AUTO_INCREMENT` for Primary Keys (MySQL) or `AUTOINCREMENT` (SQLite)

In some databases, you can set a primary key column to automatically generate a unique number for each row. In SQLite, this is achieved with `AUTOINCREMENT`.

### Example with AUTOINCREMENT (SQLite)

```sql
CREATE TABLE users (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL,
    email TEXT UNIQUE
);
```

Now, each new row added to the `users` table will automatically assign a unique `id`.

---

## 6. Modifying a Table with `ALTER TABLE`

Once a table is created, you can modify its structure using the `ALTER TABLE` command. Common modifications include adding, dropping, or renaming columns.

### Adding a New Column

```sql
ALTER TABLE users
ADD COLUMN phone TEXT;
```

This adds a `phone` column to the `users` table.

### Dropping a Column

If your database supports it (e.g., MySQL), you can remove columns.

```sql
ALTER TABLE users
DROP COLUMN phone;
```

---

## Practice Exercise: Creating a New Table

1. **Define a table structure** for a table called `products` with the following columns:
   - `product_id`: integer, primary key with auto-increment.
   - `product_name`: text, not null.
   - `price`: real.
   - `stock`: integer with a default value of `0`.

2. **Write the SQL** to create this table:
   ```sql
   CREATE TABLE products (
       product_id INTEGER PRIMARY KEY AUTOINCREMENT,
       product_name TEXT NOT NULL,
       price REAL,
       stock INTEGER DEFAULT 0
   );
   ```

3. **Verify the table structure** by running:
   ```sql
   .schema products
   ```
   This will display the `products` table’s structure if you’re using SQLite.

---

## Summary of Key Commands

Command                    | Description
---------------------------|-----------------------------------------------------
`CREATE TABLE`             | Defines a new table and its columns.
`ALTER TABLE`              | Modifies an existing table by adding or removing columns.
`INTEGER`, `TEXT`, etc.    | Data types used for defining columns.
`PRIMARY KEY`, `NOT NULL`  | Constraints to enforce data integrity.
`AUTOINCREMENT`            | Automatically increments a primary key value for each new row.
{:class="table table-striped"}

With this knowledge, you’re ready to create well-structured tables and define columns with data types and constraints. In the next lesson, we’ll cover how to add, modify, and delete data in your tables.

---
