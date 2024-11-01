---
layout: lesson
title: Advanced Filtering and Conditional Logic
author: Kevin McAleer
type: page
cover: /learn/sql/assets/cover.jpg
date: 2024-11-01
previous: 06_querying_data_with_select.html
next: 08_working_with_joins.html
description: Use SQL's AND, OR, and CASE statements for more control over queries
percent: 48
duration: 6
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


In this lesson, we’ll explore advanced filtering techniques and conditional logic with SQL. You’ll learn how to refine your queries using multiple conditions and implement conditional expressions with the `CASE` statement.

---

## 1. Combining Conditions with `AND` and `OR`

In SQL, you can combine multiple conditions in a `WHERE` clause to narrow down results. The `AND` and `OR` operators are used to create more complex filters.

- **`AND`**: All conditions must be true for the row to be included in the results.
- **`OR`**: At least one condition must be true for the row to be included in the results.

### Example: Using `AND` to Refine Results

- **Example**: Retrieve users over the age of 25 who have an email ending with "example.com".

    ```sql
    SELECT * FROM users
    WHERE age > 25 AND email LIKE '%example.com';
    ```

### Example: Using `OR` to Broaden Results

- **Example**: Retrieve users who are either named "Alice" or have an age of 30.

    ```sql
    SELECT * FROM users
    WHERE name = 'Alice' OR age = 30;
    ```

### Combining `AND` and `OR`

You can combine `AND` and `OR` in the same query, but use parentheses to control the logic and ensure accurate results.

- **Example**: Retrieve users named "Alice" who are older than 25, or users whose age is exactly 30.

    ```sql
    SELECT * FROM users
    WHERE (name = 'Alice' AND age > 25) OR age = 30;
    ```

---

## 2. Using `IN` for List Filtering

The `IN` operator allows you to specify a list of values for SQL to search for in a column, simplifying queries with multiple OR conditions.

### Example: Using `IN`

- **Example**: Retrieve users aged 25, 30, or 35.

    ```sql
    SELECT * FROM users
    WHERE age IN (25, 30, 35);
    ```

This query is equivalent to using `OR` conditions but is more concise.

---

## 3. Filtering with `BETWEEN`

The `BETWEEN` operator is used to filter data within a specific range. It’s often used with numeric or date values.

### Example: Using `BETWEEN`

- **Example**: Retrieve users aged between 20 and 30 (inclusive).

    ```sql
    SELECT * FROM users
    WHERE age BETWEEN 20 AND 30;
    ```

> **Tip**: `BETWEEN` includes the boundary values, so this query retrieves users who are exactly 20 or 30 years old as well as those in between.

---

## 4. Pattern Matching with `LIKE`

The `LIKE` operator enables pattern matching with the `%` and `_` wildcards:

- `%`: Matches any sequence of characters.
- `_`: Matches a single character.

### Example: Using `LIKE` with Wildcards

- **Example**: Retrieve users whose names start with "A".

    ```sql
    SELECT * FROM users
    WHERE name LIKE 'A%';
    ```

- **Example**: Retrieve users with a 3-letter name where the second letter is "o".

    ```sql
    SELECT * FROM users
    WHERE name LIKE '_o_';
    ```

---

## 5. Conditional Logic with `CASE`

The `CASE` statement in SQL allows you to add conditional logic to your queries. It’s often used to create calculated fields based on certain conditions, similar to `IF` statements in other languages.

### Basic `CASE` Syntax

```sql
SELECT column1,
    CASE
        WHEN condition1 THEN result1
        WHEN condition2 THEN result2
        ELSE result
    END AS new_column
FROM table_name;
```

### Example: Categorizing Data with `CASE`

Let’s say you want to categorize users based on their age group.

- **Example**: Create an age category for each user.

    ```sql
    SELECT name, age,
        CASE
            WHEN age < 18 THEN 'Minor'
            WHEN age BETWEEN 18 AND 65 THEN 'Adult'
            ELSE 'Senior'
        END AS age_category
    FROM users;
    ```

In this example:

- Users under 18 are labeled "Minor".
- Users between 18 and 65 are labeled "Adult".
- Users over 65 are labeled "Senior".

### Example: Using `CASE` with Conditions in `WHERE`

The `CASE` statement can also be used to dynamically choose values based on conditions.

- **Example**: Mark users who don’t have an email with "No Email".

    ```sql
    SELECT name,
        CASE
            WHEN email IS NULL THEN 'No Email'
            ELSE email
        END AS contact_info
    FROM users;
    ```

In this example, users without an email will display "No Email" in the `contact_info` column.

---

## 6. Practice Exercise: Advanced Filtering and Conditional Logic

1. Retrieve all users whose age is either 20, 25, or 30 using the `IN` operator.

    ```sql
    SELECT * FROM users
    WHERE age IN (20, 25, 30);
    ```

2. Retrieve users with an age between 18 and 30 who also have a name starting with "B".

    ```sql
    SELECT * FROM users
    WHERE age BETWEEN 18 AND 30 AND name LIKE 'B%';
    ```

3. Categorize users into "Young Adult" if they are under 25, "Adult" if between 25 and 40, and "Middle-aged" if over 40 using `CASE`.

    ```sql
    SELECT name, age,
        CASE
            WHEN age < 25 THEN 'Young Adult'
            WHEN age BETWEEN 25 AND 40 THEN 'Adult'
            ELSE 'Middle-aged'
        END AS age_group
    FROM users;
    ```

4. Display all users, but show "Unknown Email" for any user who does not have an email address.

    ```sql
    SELECT name,
        CASE
            WHEN email IS NULL THEN 'Unknown Email'
            ELSE email
        END AS contact_info
    FROM users;
    ```

---

## Summary of Advanced Filtering and Conditional Logic

Here’s a recap of the advanced filtering techniques and conditional logic covered in this lesson:

Operator / Function  | Description                                         | Example
---------------------|-----------------------------------------------------|-------------------------------------
`AND` / `OR`         | Combines multiple conditions                        | `WHERE age > 25 AND name LIKE 'A%'`
`IN`                 | Filters for values within a specified list          | `WHERE age IN (20, 25, 30)`
`BETWEEN`            | Filters for values within a range                   | `WHERE age BETWEEN 20 AND 30`
`LIKE`               | Matches patterns with wildcards                     | `WHERE name LIKE 'A%'`
`CASE`               | Adds conditional logic in queries                   | `CASE WHEN age < 18 THEN 'Minor' END`
{:class="table table-striped"}

By mastering these advanced filtering techniques and conditional logic, you can create more precise and flexible SQL queries. In the next lesson, we’ll move on to working with joins to retrieve data from multiple tables.

---
