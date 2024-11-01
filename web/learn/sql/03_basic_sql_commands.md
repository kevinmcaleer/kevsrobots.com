---
layout: lesson
title: Basic SQL Commands and Syntax
author: Kevin McAleer
type: page
cover: /learn/sql/assets/cover.jpg
date: 2024-11-01
previous: 02_setting_up_environment.html
next: 04_working_with_tables.html
description: Introduction to essential SQL commands such as SELECT, INSERT, UPDATE,
  DELETE
percent: 24
duration: 4
navigation:
- name: Getting Started with SQL
- content:
  - section: Introduction
    content:
    - name: Course Overview
      link: 00_course_overview.html
  - section: SQL Basics
    content:
    - name: Introduction to SQL and Relational Databases
      link: 01_introduction_to_sql.html
    - name: Setting Up Your SQL Environment
      link: 02_setting_up_environment.html
    - name: Basic SQL Commands and Syntax
      link: 03_basic_sql_commands.html
    - name: Working with Tables and Data Types
      link: 04_working_with_tables.html
    - name: Data Insertion, Modification, and Deletion
      link: 05_data_insertion_and_modification.html
    - name: Querying Data with SELECT
      link: 06_querying_data_with_select.html
    - name: Advanced Filtering and Conditional Logic
      link: 07_advanced_filtering.html
    - name: Working with Joins
      link: 08_working_with_joins.html
  - section: Intermediate Concepts
    content:
    - name: Grouping and Aggregating Data
      link: 09_grouping_and_aggregating_data.html
    - name: Introduction to Database Normalization
      link: 10_introduction_to_normalization.html
    - name: Boyce-Codd Normal Form (BCNF)
      link: 11_boyce_codd_normal_form.html
    - name: Subqueries and Nested Queries
      link: 12_subqueries_and_nested_queries.html
    - name: Views and Indexes
      link: 13_views_and_indexes.html
    - name: SQL Functions and Expressions
      link: 14_sql_functions_and_expressions.html
  - section: Conclusion
    content:
    - name: Wrap-up and Final Project
      link: 15_wrap_up_and_final_project.html
---


In this lesson, we’ll cover the essential SQL commands used for managing data in databases. These commands—SELECT, INSERT, UPDATE, and DELETE—form the basis of SQL’s ability to interact with and manipulate data.

---

## Core SQL Commands

In SQL, commands are structured as statements. Each statement has a specific purpose, and together, they enable us to create, modify, retrieve, and delete data within a database.

---

## 1. SELECT: Retrieving Data

The `SELECT` statement is used to retrieve data from one or more tables. It’s one of the most commonly used SQL commands and provides a range of options for filtering and sorting data.

### Basic SELECT Syntax

```sql
SELECT column1, column2, ...
FROM table_name;
```

- **Example**: Retrieving all columns from a table named `users`.
    ```sql
    SELECT * FROM users;
    ```
- **Example**: Retrieving only specific columns (e.g., `id` and `name`).
    ```sql
    SELECT id, name FROM users;
    ```

### Filtering Results with WHERE

You can filter results to retrieve only specific data using the `WHERE` clause.

- **Example**: Retrieve all users named "Alice".
    ```sql
    SELECT * FROM users WHERE name = 'Alice';
    ```

---

## 2. INSERT: Adding New Data

The `INSERT` statement is used to add new rows (records) into a table. It specifies the table to add data to, along with the columns and values.

### Basic INSERT Syntax

```sql
INSERT INTO table_name (column1, column2, ...)
VALUES (value1, value2, ...);
```

- **Example**: Inserting a new user into the `users` table.
    ```sql
    INSERT INTO users (name) VALUES ('David');
    ```
- **Example**: Inserting multiple columns of data.
    ```sql
    INSERT INTO users (id, name) VALUES (4, 'Eve');
    ```

> **Note**: If a column has a default value or is auto-incremented (like `id`), you may omit it from the `INSERT` statement.

---

## 3. UPDATE: Modifying Existing Data

The `UPDATE` statement allows you to modify existing data in a table. It specifies the table, the columns to update, and a condition to filter the rows to update.

### Basic UPDATE Syntax

```sql
UPDATE table_name
SET column1 = value1, column2 = value2, ...
WHERE condition;
```

- **Example**: Changing the name of the user with `id` 1 to "Alice Johnson".
    ```sql
    UPDATE users
    SET name = 'Alice Johnson'
    WHERE id = 1;
    ```

> **Important**: Always use the `WHERE` clause in `UPDATE` statements to avoid updating all rows in the table.

---

## 4. DELETE: Removing Data

The `DELETE` statement is used to remove rows from a table. Like `UPDATE`, it’s critical to use a `WHERE` clause to specify the rows to delete.

### Basic DELETE Syntax

```sql
DELETE FROM table_name
WHERE condition;
```

- **Example**: Deleting a user named "Charlie".
    ```sql
    DELETE FROM users
    WHERE name = 'Charlie';
    ```

- **Example**: Deleting all users (use with caution).
    ```sql
    DELETE FROM users;
    ```

> **Warning**: The above example will delete all rows in the `users` table. Always ensure your `DELETE` statements are precise.

---

## Putting It All Together

Here’s a quick scenario to illustrate the use of these commands.

1. **Insert Data**:
    ```sql
    INSERT INTO users (name) VALUES ('Alice'), ('Bob'), ('Charlie');
    ```

2. **Retrieve Data**:
    ```sql
    SELECT * FROM users;
    ```

3. **Update Data**:
    ```sql
    UPDATE users SET name = 'Charlie Brown' WHERE name = 'Charlie';
    ```

4. **Delete Data**:
    ```sql
    DELETE FROM users WHERE name = 'Bob';
    ```

After each command, you can run `SELECT * FROM users;` to see the changes applied in the `users` table.

---

## Additional Tips for Writing SQL

- **End Each Command with a Semicolon**: This tells the database that the command is complete and ready to be executed.
- **Use Descriptive Column Names**: When creating tables, use column names that clearly represent the data they store.
- **Test Commands Carefully**: Particularly with `UPDATE` and `DELETE` commands, always verify your `WHERE` clause to avoid unintended changes or deletions.

---

## Recap of Core SQL Commands

Here’s a summary of the four main SQL commands you’ve learned in this lesson:

Command      | Description                                           | Example
-------------|-------------------------------------------------------|--------------------------------
`SELECT`     | Retrieves data from a table                           | `SELECT * FROM users;`
`INSERT`     | Adds new data into a table                            | `INSERT INTO users (name) VALUES ('Alice');`
`UPDATE`     | Modifies existing data in a table                     | `UPDATE users SET name = 'Alice Johnson' WHERE id = 1;`
`DELETE`     | Removes data from a table                             | `DELETE FROM users WHERE name = 'Charlie';`
{:class="table table-striped"}

By mastering these fundamental commands, you’ve built a strong foundation for working with SQL databases. In the next lesson, we’ll explore how to structure your database using tables and data types.

---
