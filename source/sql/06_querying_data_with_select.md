---
title: Querying Data with SELECT
description: >-
    Retrieve data from tables using SELECT and basic filtering techniques
layout: lesson
---

In this lesson, we’ll cover the `SELECT` statement, which is the cornerstone of querying data in SQL. You’ll learn how to retrieve data from tables, apply basic filters, and sort your results.

---

## 1. The Basics of `SELECT`

The `SELECT` statement is used to retrieve specific data from one or more tables. You can use `SELECT` to display specific columns, filter rows, sort results, and perform calculations.

### Basic `SELECT` Syntax

```sql
SELECT column1, column2, ...
FROM table_name;
```

- **Example**: Display all columns from the `users` table.

    ```sql
    SELECT * FROM users;
    ```

- **Example**: Display only the `name` and `email` columns.

    ```sql
    SELECT name, email FROM users;
    ```

The `*` symbol selects all columns in the table, while specifying individual columns limits the output to just those columns.

---

## 2. Filtering Data with `WHERE`

The `WHERE` clause allows you to filter rows based on specified conditions. It’s used to retrieve only the data that meets certain criteria.

### Basic `WHERE` Syntax

```sql
SELECT column1, column2, ...
FROM table_name
WHERE condition;
```

- **Example**: Retrieve all users with the name "Alice".

    ```sql
    SELECT * FROM users
    WHERE name = 'Alice';
    ```

- **Example**: Retrieve users who are 30 years old or older.

    ```sql
    SELECT * FROM users
    WHERE age >= 30;
    ```

### Common Operators with `WHERE`

Operator  | Description                    | Example
----------|--------------------------------|--------------------------------
`=`       | Equal to                       | `WHERE age = 30`
`<>` or `!=` | Not equal to                | `WHERE name != 'Alice'`
`<`       | Less than                      | `WHERE age < 25`
`<=`      | Less than or equal to          | `WHERE age <= 30`
`>`       | Greater than                   | `WHERE age > 25`
`>=`      | Greater than or equal to       | `WHERE age >= 30`
`BETWEEN` | Within a range                 | `WHERE age BETWEEN 20 AND 30`
`LIKE`    | Pattern matching               | `WHERE name LIKE 'A%'`
`IN`      | Matches any value in a list    | `WHERE age IN (25, 30, 35)`
{:class="table table-striped"}

> **Tip**: Use `LIKE` with wildcards (`%` and `_`) for flexible pattern matching. `%` matches any sequence of characters, while `_` matches a single character.

---

## 3. Sorting Results with `ORDER BY`

The `ORDER BY` clause sorts the result set by one or more columns in ascending (`ASC`) or descending (`DESC`) order.

### Basic `ORDER BY` Syntax

```sql
SELECT column1, column2, ...
FROM table_name
ORDER BY column1 [ASC|DESC], column2 [ASC|DESC], ...;
```

- **Example**: List all users, sorted by age in ascending order.

    ```sql
    SELECT * FROM users
    ORDER BY age ASC;
    ```

- **Example**: List all users, sorted by age in descending order.

    ```sql
    SELECT * FROM users
    ORDER BY age DESC;
    ```

- **Example**: Sort by age, and then by name within each age group.

    ```sql
    SELECT * FROM users
    ORDER BY age ASC, name ASC;
    ```

---

## 4. Limiting Results with `LIMIT`

The `LIMIT` clause restricts the number of rows returned by the query. This is especially useful when testing queries or when only the top results are needed.

### Basic `LIMIT` Syntax

```sql
SELECT column1, column2, ...
FROM table_name
LIMIT number_of_rows;
```

- **Example**: Retrieve the first 5 users.

    ```sql
    SELECT * FROM users
    LIMIT 5;
    ```

### Combining `LIMIT` with `ORDER BY`

`LIMIT` can be used with `ORDER BY` to retrieve the top or bottom values.

- **Example**: Retrieve the 3 oldest users.

    ```sql
    SELECT * FROM users
    ORDER BY age DESC
    LIMIT 3;
    ```

---

## 5. Combining Conditions with `AND` and `OR`

You can combine multiple conditions in the `WHERE` clause using `AND` and `OR` operators to refine your queries.

- **`AND`**: All conditions must be true.
- **`OR`**: At least one condition must be true.

### Example: Using `AND`

- **Example**: Retrieve users who are over 25 years old and whose name starts with "A".

    ```sql
    SELECT * FROM users
    WHERE age > 25 AND name LIKE 'A%';
    ```

### Example: Using `OR`

- **Example**: Retrieve users whose name is "Alice" or who are 30 years old.

    ```sql
    SELECT * FROM users
    WHERE name = 'Alice' OR age = 30;
    ```

---

## 6. Using Aliases with `AS`

Aliases give temporary names to columns or tables for easier readability. They do not change the actual column or table names in the database.

### Example: Aliasing Columns

- **Example**: Display user names with the alias "User Name".

    ```sql
    SELECT name AS "User Name"
    FROM users;
    ```

### Example: Aliasing Tables

- **Example**: Use an alias for the `users` table to shorten query syntax.

    ```sql
    SELECT u.name, u.age
    FROM users AS u
    WHERE u.age > 25;
    ```

---

## Practice Exercise: Writing `SELECT` Queries

1. Retrieve all users from the `users` table, displaying only the `name` and `email` columns.

    ```sql
    SELECT name, email FROM users;
    ```

2. Retrieve all users whose age is between 20 and 30.

    ```sql
    SELECT * FROM users
    WHERE age BETWEEN 20 AND 30;
    ```

3. Retrieve the top 3 users by age, in descending order.

    ```sql
    SELECT * FROM users
    ORDER BY age DESC
    LIMIT 3;
    ```

4. Retrieve users whose name starts with "A" and display their names as "User".

    ```sql
    SELECT name AS "User" FROM users
    WHERE name LIKE 'A%';
    ```

---

## Summary of `SELECT` Query Techniques

Here’s a recap of the techniques covered in this lesson:

Command       | Description                                           | Example
--------------|-------------------------------------------------------|--------------------------------
`SELECT`      | Retrieves data from a table                           | `SELECT * FROM users;`
`WHERE`       | Filters results based on conditions                   | `WHERE age > 25`
`ORDER BY`    | Sorts results by specified columns                    | `ORDER BY age DESC`
`LIMIT`       | Limits the number of rows returned                    | `LIMIT 5`
`AND` / `OR`  | Combines multiple conditions                          | `WHERE age > 25 AND name LIKE 'A%'`
`AS`          | Provides an alias for columns or tables               | `SELECT name AS "User Name"`
{:class="table table-striped"}

With these querying techniques, you’re now equipped to retrieve and filter data from your tables effectively. In the next lesson, we’ll expand on filtering techniques and introduce conditional logic for more powerful queries.

---
