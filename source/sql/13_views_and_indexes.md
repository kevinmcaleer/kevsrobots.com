---
title: Views and Indexes
description: >-
    Creating and using views to simplify complex queries, and using indexes to optimize database performance
layout: lesson
---

In this lesson, we’ll explore **views** and **indexes**, two powerful tools that improve database usability and performance. Views allow you to simplify complex queries by saving them as virtual tables, while indexes optimize query speed by improving data retrieval efficiency.

---

## 1. What is a View?

A **view** is a virtual table in SQL that is defined by a query. It doesn’t store data itself but provides a simplified representation of data from one or more tables. Views are useful for creating reusable query results, simplifying complex queries, and improving data security by limiting access to specific data.

### Why Use Views?

- **Simplify Complex Queries**: Save complex queries as views and reuse them without writing the query each time.
- **Enhance Data Security**: Limit user access to specific columns or rows without altering the underlying tables.
- **Improve Maintainability**: Update the view query, and all users automatically get updated results.

---

## 2. Creating a View

You create a view using the `CREATE VIEW` statement. A view can be as simple or complex as necessary, depending on the query used to define it.

### Basic `CREATE VIEW` Syntax

```sql
CREATE VIEW view_name AS
SELECT column1, column2, ...
FROM table_name
WHERE condition;
```

### Example: Creating a View

Suppose we have an `employees` table with employee details:

**employees**

```plaintext
| emp_id | name     | salary | dept_id |
|--------|----------|--------|---------|
| 1      | Alice    | 50000  | 1       |
| 2      | Bob      | 60000  | 2       |
| 3      | Charlie  | 70000  | 1       |
```

- **Example**: Create a view called `high_earners` for employees earning over $60,000.

    ```sql
    CREATE VIEW high_earners AS
    SELECT name, salary
    FROM employees
    WHERE salary > 60000;
    ```

Now, you can retrieve high earners using a simple `SELECT` on the `high_earners` view:

```sql
SELECT * FROM high_earners;
```

---

## 3. Modifying a View

To update a view’s definition, use the `CREATE OR REPLACE VIEW` statement. This will redefine the view without requiring you to drop it first.

### Example: Modifying a View

- **Example**: Update the `high_earners` view to include only employees earning over $65,000.

    ```sql
    CREATE OR REPLACE VIEW high_earners AS
    SELECT name, salary
    FROM employees
    WHERE salary > 65000;
    ```

This command updates the `high_earners` view with the new salary threshold.

---

## 4. Dropping a View

To remove a view from the database, use the `DROP VIEW` statement.

### Basic `DROP VIEW` Syntax

```sql
DROP VIEW view_name;
```

- **Example**: Drop the `high_earners` view.

    ```sql
    DROP VIEW high_earners;
    ```

---

## 5. What is an Index?

An **index** is a database structure that improves the speed of data retrieval. It works like an index in a book, allowing the database to locate data more quickly. Indexes are particularly useful for large tables or columns frequently used in `WHERE`, `JOIN`, or `ORDER BY` clauses.

### Why Use Indexes?

- **Optimize Query Performance**: Indexes allow the database to retrieve data faster, especially in large tables.
- **Reduce Query Time**: By indexing frequently accessed columns, you reduce the time it takes to search for values.

> **Note**: While indexes improve read performance, they can slow down write operations (e.g., `INSERT`, `UPDATE`, `DELETE`) because the index needs updating whenever the data changes.

---

## 6. Creating an Index

To create an index, use the `CREATE INDEX` statement. You can create indexes on one or more columns to optimize queries on those columns.

### Basic `CREATE INDEX` Syntax

```sql
CREATE INDEX index_name ON table_name (column1, column2, ...);
```

### Example: Creating an Index

- **Example**: Create an index on the `salary` column of the `employees` table to speed up queries that filter by salary.

    ```sql
    CREATE INDEX idx_salary ON employees (salary);
    ```

This index helps optimize queries such as:

```sql
SELECT * FROM employees
WHERE salary > 60000;
```

### Composite Indexes

A **composite index** is an index on multiple columns. It’s useful when queries frequently use multiple columns in the `WHERE` clause.

- **Example**: Create a composite index on `dept_id` and `salary` in the `employees` table.

    ```sql
    CREATE INDEX idx_dept_salary ON employees (dept_id, salary);
    ```

This index will optimize queries that filter by both `dept_id` and `salary`.

---

## 7. Dropping an Index

To remove an index from the database, use the `DROP INDEX` statement.

### Basic `DROP INDEX` Syntax

```sql
DROP INDEX index_name;
```

- **Example**: Drop the `idx_salary` index.

    ```sql
    DROP INDEX idx_salary;
    ```

---

## 8. Best Practices for Using Views and Indexes

### When to Use Views

- Use views to simplify complex queries that you frequently use.
- Use views to limit user access to specific columns or rows for security.
- Avoid using views that rely on heavy or complex calculations if performance is a concern.

### When to Use Indexes

- Index columns frequently used in `WHERE`, `JOIN`, or `ORDER BY` clauses.
- Avoid indexing columns that are frequently updated, as indexes slow down write operations.
- Avoid excessive indexing; each index takes up storage space and may slow down updates.

---

## Practice Exercise: Creating and Using Views and Indexes

1. **Create a View**: Create a view called `engineering_staff` that shows employees in the Engineering department.

    ```sql
    CREATE VIEW engineering_staff AS
    SELECT name, salary
    FROM employees
    WHERE dept_id = 1;
    ```

2. **Query the View**: Retrieve all data from the `engineering_staff` view.

    ```sql
    SELECT * FROM engineering_staff;
    ```

3. **Create an Index**: Create an index on the `dept_id` column of the `employees` table.

    ```sql
    CREATE INDEX idx_dept_id ON employees (dept_id);
    ```

4. **Test the Index**: Run a query that filters by `dept_id` and observe the improved performance.

    ```sql
    SELECT * FROM employees
    WHERE dept_id = 1;
    ```

5. **Drop the View**: Drop the `engineering_staff` view.

    ```sql
    DROP VIEW engineering_staff;
    ```

---

## Summary of Views and Indexes

Here’s a quick summary of the key concepts and commands related to views and indexes:

Feature    | Purpose                                          | Example
-----------|--------------------------------------------------|------------------------------------------
View       | A virtual table created by a saved query         | `CREATE VIEW view_name AS SELECT ...`
Index      | A structure that improves data retrieval speed   | `CREATE INDEX index_name ON table (col)`
Composite Index | An index on multiple columns               | `CREATE INDEX index_name ON table (col1, col2)`
{:class="table table-striped"}

Views and indexes are valuable tools for optimizing and simplifying your SQL workflows. In the next lesson, we’ll dive into SQL functions and expressions, which will further enhance your ability to manipulate and analyze data.

---
