---
title: Data Insertion, Modification, and Deletion
description: >-
    How to perform CRUD operations (Create, Read, Update, Delete) in SQL
layout: lesson
---

In this lesson, we’ll explore how to insert, modify, and delete data in SQL tables. These operations are commonly referred to as **CRUD** operations: **Create**, **Read**, **Update**, and **Delete**.

---

## 1. Inserting Data with `INSERT INTO`

The `INSERT INTO` statement allows you to add new records (rows) to a table. It specifies the table to insert data into, as well as the values for each column.

### Basic `INSERT` Syntax

```sql
INSERT INTO table_name (column1, column2, ...)
VALUES (value1, value2, ...);
```

- **Example**: Adding a new user to the `users` table.

    ```sql
    INSERT INTO users (name, age, email)
    VALUES ('Alice', 30, 'alice@example.com');
    ```

- **Example**: Adding multiple users at once.

    ```sql
    INSERT INTO users (name, age, email)
    VALUES ('Bob', 25, 'bob@example.com'),
           ('Charlie', 28, 'charlie@example.com');
    ```

### Inserting Data without Specifying All Columns

If some columns have default values, you can omit them in the `INSERT` statement.

- **Example**: Inserting only `name` and `email`, letting `age` default to `NULL` or a specified default value.

    ```sql
    INSERT INTO users (name, email)
    VALUES ('David', 'david@example.com');
    ```

---

## 2. Reading Data with `SELECT`

To verify your data, you can use the `SELECT` statement to retrieve rows from a table.

- **Example**: View all users in the `users` table.

    ```sql
    SELECT * FROM users;
    ```

This command retrieves every column for each row in the table.

---

## 3. Modifying Data with `UPDATE`

The `UPDATE` statement is used to modify existing data in a table. It specifies the table, the columns to change, and conditions to filter the rows to update.

### Basic `UPDATE` Syntax

```sql
UPDATE table_name
SET column1 = value1, column2 = value2, ...
WHERE condition;
```

> **Note**: It’s essential to use the `WHERE` clause to target specific rows. Omitting it will update every row in the table.

### Example: Updating a User’s Age

- **Example**: Change the age of the user with `id` 1 to 35.

    ```sql
    UPDATE users
    SET age = 35
    WHERE id = 1;
    ```

### Example: Updating Multiple Columns

- **Example**: Change both the age and email of a specific user.

    ```sql
    UPDATE users
    SET age = 32, email = 'alice.johnson@example.com'
    WHERE name = 'Alice';
    ```

---

## 4. Deleting Data with `DELETE`

The `DELETE` statement is used to remove rows from a table. Similar to `UPDATE`, it’s critical to include a `WHERE` clause to specify which rows to delete.

### Basic `DELETE` Syntax

```sql
DELETE FROM table_name
WHERE condition;
```

> **Warning**: Omitting the `WHERE` clause deletes all rows in the table.

### Example: Deleting a Specific User

- **Example**: Delete the user with the email "bob@example.com".

    ```sql
    DELETE FROM users
    WHERE email = 'bob@example.com';
    ```

### Example: Deleting All Rows in a Table

- **Example**: Remove all data from the `users` table (use with caution).

    ```sql
    DELETE FROM users;
    ```

This command empties the table but retains its structure, so you can still insert new data later.

---

## 5. Using Transactions for Data Integrity

When performing multiple `INSERT`, `UPDATE`, or `DELETE` statements, it’s often beneficial to use **transactions**. A transaction groups multiple changes, which are only saved if all operations are successful.

### Starting and Committing a Transaction

1. **Start a Transaction**:
    ```sql
    BEGIN TRANSACTION;
    ```

2. **Execute Multiple Statements**:
    ```sql
    INSERT INTO users (name, age, email) VALUES ('Eve', 29, 'eve@example.com');
    UPDATE users SET age = 30 WHERE name = 'Eve';
    ```

3. **Commit the Transaction**:
    ```sql
    COMMIT;
    ```

If any part of the transaction fails, you can use `ROLLBACK` to undo all changes:

```sql
ROLLBACK;
```

Transactions help ensure data integrity, especially in cases where multiple related changes must either all succeed or fail together.

---

## Practice Exercise: Performing CRUD Operations

1. **Insert** three new records into the `users` table with the following data:

   - (John, 40, 'john@example.com')
   - (Diana, 35, 'diana@example.com')
   - (Sophia, 22, 'sophia@example.com')

   ```sql
   INSERT INTO users (name, age, email)
   VALUES ('John', 40, 'john@example.com'),
          ('Diana', 35, 'diana@example.com'),
          ('Sophia', 22, 'sophia@example.com');
   ```

2. **Update** John’s age to 41.

   ```sql
   UPDATE users
   SET age = 41
   WHERE name = 'John';
   ```

3. **Delete** the user "Sophia".

   ```sql
   DELETE FROM users
   WHERE name = 'Sophia';
   ```

4. Verify each step by using `SELECT * FROM users;` to check your results.

---

## Summary of CRUD Commands

Here’s a recap of the CRUD commands and their usage:

Command      | Description                                           | Example
-------------|-------------------------------------------------------|--------------------------------
`INSERT`     | Adds new data into a table                            | `INSERT INTO users (name) VALUES ('Alice');`
`SELECT`     | Retrieves data from a table                           | `SELECT * FROM users;`
`UPDATE`     | Modifies existing data in a table                     | `UPDATE users SET age = 35 WHERE id = 1;`
`DELETE`     | Removes data from a table                             | `DELETE FROM users WHERE name = 'Bob';`
{:class="table table-striped"}

With these CRUD operations, you now have the foundational skills to manage data within your SQL database. In the next lesson, we’ll dive deeper into querying data and using `SELECT` statements with filters.

---
