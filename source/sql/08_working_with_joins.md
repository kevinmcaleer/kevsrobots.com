---
title: Working with Joins
description: >-
    Learn how to combine data from multiple tables using SQL joins
layout: lesson
---

In this lesson, we’ll explore how to work with multiple tables in a relational database using **joins**. Joins are essential for combining data across tables, helping you retrieve meaningful insights by connecting related data.

---

## 1. Understanding Joins

In relational databases, data is often distributed across multiple tables. Joins allow us to retrieve data from two or more tables based on related columns, typically through primary and foreign keys.

### Types of Joins

- **INNER JOIN**: Returns records with matching values in both tables.
- **LEFT JOIN** (or **LEFT OUTER JOIN**): Returns all records from the left table and matching records from the right table. Unmatched rows from the right table return `NULL`.
- **RIGHT JOIN** (or **RIGHT OUTER JOIN**): Returns all records from the right table and matching records from the left table. Unmatched rows from the left table return `NULL`.
- **FULL JOIN** (or **FULL OUTER JOIN**): Returns all records when there is a match in either table. Non-matching rows from both tables return `NULL`.

---

## 2. Inner Join: Retrieving Matching Records

An **INNER JOIN** returns records that have matching values in both tables.

### Basic INNER JOIN Syntax

```sql
SELECT columns
FROM table1
INNER JOIN table2
ON table1.common_column = table2.common_column;
```

### Example: Inner Join with `users` and `orders` Tables

Suppose we have two tables:

**users**

```plaintext
| user_id | name    |
|---------|---------|
| 1       | Alice   |
| 2       | Bob     |
| 3       | Charlie |
```

**orders**

```plaintext
| order_id | user_id | item        |
|----------|---------|-------------|
| 101      | 1       | Laptop      |
| 102      | 2       | Smartphone  |
| 103      | 1       | Tablet      |
```

- **Example**: Retrieve each user and their orders.

    ```sql
    SELECT users.name, orders.item
    FROM users
    INNER JOIN orders ON users.user_id = orders.user_id;
    ```

**Result**:

```plaintext
| name   | item       |
|--------|------------|
| Alice  | Laptop     |
| Bob    | Smartphone |
| Alice  | Tablet     |
```

Only users with orders appear in the results.

---

## 3. Left Join: Retrieving All Records from the Left Table

A **LEFT JOIN** returns all records from the left table and the matched records from the right table. If there is no match, the result is `NULL` for the columns from the right table.

### Basic LEFT JOIN Syntax

```sql
SELECT columns
FROM table1
LEFT JOIN table2
ON table1.common_column = table2.common_column;
```

### Example: Left Join with `users` and `orders`

- **Example**: Retrieve all users and their orders, including users without orders.

    ```sql
    SELECT users.name, orders.item
    FROM users
    LEFT JOIN orders ON users.user_id = orders.user_id;
    ```

**Result**:

```plaintext
| name     | item       |
|----------|------------|
| Alice    | Laptop     |
| Bob      | Smartphone |
| Alice    | Tablet     |
| Charlie  | NULL       |
```

The `NULL` value indicates that Charlie has no orders.

---

## 4. Right Join: Retrieving All Records from the Right Table

A **RIGHT JOIN** is the opposite of a left join. It returns all records from the right table and the matched records from the left table, with `NULL` for unmatched rows from the left.

### Basic RIGHT JOIN Syntax

```sql
SELECT columns
FROM table1
RIGHT JOIN table2
ON table1.common_column = table2.common_column;
```

- **Example**: If using `RIGHT JOIN` with `users` and `orders`, we’d retrieve all orders, with `NULL` for users without matching entries. This syntax is more commonly supported in MySQL and PostgreSQL than SQLite.

> **Note**: Not all SQL databases support `RIGHT JOIN`. SQLite, for example, does not support `RIGHT JOIN` directly.

---

## 5. Full Join: Retrieving All Records with Matches from Either Table

A **FULL JOIN** returns all records when there’s a match in either table. Unmatched rows from each table show `NULL` for the other table’s columns.

### Basic FULL JOIN Syntax

```sql
SELECT columns
FROM table1
FULL JOIN table2
ON table1.common_column = table2.common_column;
```

- **Example**: Retrieve all users and orders, showing `NULL` where there’s no match in either table.

> **Note**: SQLite does not support `FULL JOIN` directly, but similar results can be achieved using `UNION` of `LEFT JOIN` and `RIGHT JOIN`.

---

## 6. Self Join: Joining a Table with Itself

A **self join** is when a table is joined with itself. This is useful for hierarchical data or when finding relationships within the same table.

### Example: Self Join on an `employees` Table

**employees**

```plaintext
| emp_id | name     | manager_id |
|--------|----------|------------|
| 1      | Alice    | NULL       |
| 2      | Bob      | 1          |
| 3      | Charlie  | 1          |
| 4      | Diana    | 2          |
```

- **Example**: Retrieve each employee and their manager’s name.

    ```sql
    SELECT e1.name AS employee, e2.name AS manager
    FROM employees AS e1
    LEFT JOIN employees AS e2 ON e1.manager_id = e2.emp_id;
    ```

**Result**:

```plaintext
| employee | manager |
|----------|---------|
| Alice    | NULL    |
| Bob      | Alice   |
| Charlie  | Alice   |
| Diana    | Bob     |
```

---

## Practice Exercise: Working with Joins

1. **Inner Join**: Retrieve all users and their orders using an inner join.

    ```sql
    SELECT users.name, orders.item
    FROM users
    INNER JOIN orders ON users.user_id = orders.user_id;
    ```

2. **Left Join**: Retrieve all users, including those without orders.

    ```sql
    SELECT users.name, orders.item
    FROM users
    LEFT JOIN orders ON users.user_id = orders.user_id;
    ```

3. **Self Join**: Retrieve each employee and their manager from the `employees` table.

    ```sql
    SELECT e1.name AS employee, e2.name AS manager
    FROM employees AS e1
    LEFT JOIN employees AS e2 ON e1.manager_id = e2.emp_id;
    ```

4. **Full Join (Advanced)**: If your database supports it, retrieve all users and orders using a full join.

---

## Summary of Join Types

Here’s a summary of the join types covered in this lesson:

Join Type     | Description                                                        | Example
--------------|--------------------------------------------------------------------|-------------------------------------
INNER JOIN    | Returns records with matching values in both tables                | `INNER JOIN orders ON users.user_id = orders.user_id`
LEFT JOIN     | Returns all records from the left table and matched records from the right | `LEFT JOIN orders ON users.user_id = orders.user_id`
RIGHT JOIN    | Returns all records from the right table and matched records from the left | `RIGHT JOIN orders ON users.user_id = orders.user_id`
FULL JOIN     | Returns all records when there’s a match in either table           | `FULL JOIN orders ON users.user_id = orders.user_id`
SELF JOIN     | Joins a table with itself to compare rows within the same table    | `FROM employees AS e1 LEFT JOIN employees AS e2`
{:class="table table-striped"}

By mastering joins, you can retrieve and analyze data from multiple tables effectively. In the next lesson, we’ll focus on grouping and aggregating data for even more insightful analyses.

---
