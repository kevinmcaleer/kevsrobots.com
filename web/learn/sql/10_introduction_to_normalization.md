---
layout: lesson
title: Introduction to Database Normalization
author: Kevin McAleer
type: page
cover: /learn/sql/assets/cover.jpg
date: 2024-11-01
previous: 09_grouping_and_aggregating_data.html
next: 11_boyce_codd_normal_form.html
description: Understanding database normalization and the first three normal forms
  (1NF, 2NF, 3NF)
percent: 66
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


In this lesson, we’ll discuss **database normalization**, a process that organizes data to reduce redundancy and improve data integrity. You’ll learn about the first three normal forms (1NF, 2NF, and 3NF), which are essential for creating efficient and well-structured databases.

---

## 1. What is Database Normalization?

**Normalization** is a technique used in database design to structure tables and their relationships, minimizing duplicate data and ensuring data integrity. The goal is to create a database that:

- Reduces data redundancy (i.e., avoids storing the same data in multiple places)
- Ensures data consistency and accuracy
- Makes it easier to update, insert, and delete data

Normalization organizes data into a series of tables and establishes relationships based on defined rules, known as **normal forms**.

---

## 2. The First Normal Form (1NF)

A table is in **First Normal Form (1NF)** if:

1. Each column contains only atomic (indivisible) values.
2. Each row is unique.

### Example: 1NF Violations and Corrections

Consider the following table that stores a customer’s order and items:

```plaintext
| order_id | customer_name | items          |
|----------|---------------|----------------|
| 1        | Alice         | Laptop, Tablet |
| 2        | Bob           | Smartphone     |
```

This table is not in 1NF because the `items` column contains multiple values (e.g., "Laptop, Tablet"). To bring it into 1NF, we need to split the items so that each row contains only one item per order:

```plaintext
| order_id | customer_name | item       |
|----------|---------------|------------|
| 1        | Alice         | Laptop     |
| 1        | Alice         | Tablet     |
| 2        | Bob           | Smartphone |
```

Now, each column contains only atomic values, meeting the criteria for 1NF.

---

## 3. The Second Normal Form (2NF)

A table is in **Second Normal Form (2NF)** if it:

1. Is already in 1NF.
2. Has no partial dependencies, meaning that non-key columns depend on the entire primary key, not just part of it.

### Example: 2NF Violations and Corrections

Consider an `orders` table that includes customer information:

```plaintext
| order_id | customer_id | customer_name | item       |
|----------|-------------|---------------|------------|
| 1        | 101         | Alice         | Laptop     |
| 1        | 101         | Alice         | Tablet     |
| 2        | 102         | Bob           | Smartphone |
```

This table is not in 2NF because `customer_name` depends only on `customer_id`, which is part of the primary key (`order_id, customer_id`). To normalize it, we can separate customer information into its own table.

**orders**:

```plaintext
| order_id | customer_id | item       |
|----------|-------------|------------|
| 1        | 101         | Laptop     |
| 1        | 101         | Tablet     |
| 2        | 102         | Smartphone |
```

**customers**:

```plaintext
| customer_id | customer_name |
|-------------|---------------|
| 101         | Alice         |
| 102         | Bob           |
```

Now, the `orders` table only contains data related to the order, and the `customers` table holds information specific to customers. This structure meets 2NF requirements.

---

## 4. The Third Normal Form (3NF)

A table is in **Third Normal Form (3NF)** if it:

1. Is already in 2NF.
2. Has no transitive dependencies, meaning that non-key columns are dependent only on the primary key, not on other non-key columns.

### Example: 3NF Violations and Corrections

Consider an `orders` table with additional customer location information:

```plaintext
| order_id | customer_id | customer_name | city     | item       |
|----------|-------------|---------------|----------|------------|
| 1        | 101         | Alice         | New York | Laptop     |
| 2        | 102         | Bob           | Chicago  | Smartphone |
```

This table is not in 3NF because `city` depends on `customer_id` through `customer_name`, rather than directly on the primary key (`order_id`). To bring it into 3NF, we should split `city` into the `customers` table.

**orders**:

```plaintext
| order_id | customer_id | item       |
|----------|-------------|------------|
| 1        | 101         | Laptop     |
| 2        | 102         | Smartphone |
```

**customers**:

```plaintext
| customer_id | customer_name | city     |
|-------------|---------------|----------|
| 101         | Alice         | New York |
| 102         | Bob           | Chicago  |
```

Now, each non-key column depends only on the primary key in each table, meeting 3NF requirements.

---

## 5. Benefits of Normalization

Normalization provides several advantages:

- **Data Integrity**: Reduces redundancy and ensures data consistency.
- **Efficient Storage**: Minimizes duplicate data, saving storage space.
- **Improved Maintenance**: Simplifies updates, deletions, and insertions by eliminating duplicated data.
- **Enhanced Query Performance**: Structured relationships improve query efficiency and make complex queries easier to manage.

---

## 6. Practice Exercise: Normalizing Data

1. **Convert to 1NF**: Consider the following table and make it 1NF compliant.

    ```plaintext
    | order_id | customer_name | items          |
    |----------|---------------|----------------|
    | 1        | Alice         | Laptop, Tablet |
    | 2        | Bob           | Smartphone     |
    ```

    **Solution**:

    ```plaintext
    | order_id | customer_name | item       |
    |----------|---------------|------------|
    | 1        | Alice         | Laptop     |
    | 1        | Alice         | Tablet     |
    | 2        | Bob           | Smartphone |
    ```

2. **Convert to 2NF**: Consider the following table and make it 2NF compliant.

    ```plaintext
    | order_id | customer_id | customer_name | item       |
    |----------|-------------|---------------|------------|
    | 1        | 101         | Alice         | Laptop     |
    | 1        | 101         | Alice         | Tablet     |
    | 2        | 102         | Bob           | Smartphone |
    ```

    **Solution**:
    Separate customer information into a new table:

    **orders**:

    ```plaintext
    | order_id | customer_id | item       |
    |----------|-------------|------------|
    | 1        | 101         | Laptop     |
    | 1        | 101         | Tablet     |
    | 2        | 102         | Smartphone |
    ```

    **customers**:

    ```plaintext
    | customer_id | customer_name |
    |-------------|---------------|
    | 101         | Alice         |
    | 102         | Bob           |
    ```

3. **Convert to 3NF**: Consider the following table and make it 3NF compliant.

    ```plaintext
    | order_id | customer_id | customer_name | city     | item       |
    |----------|-------------|---------------|----------|------------|
    | 1        | 101         | Alice         | New York | Laptop     |
    | 2        | 102         | Bob           | Chicago  | Smartphone |
    ```

    **Solution**:
    Move `city` to the `customers` table:

    **orders**:

    ```plaintext
    | order_id | customer_id | item       |
    |----------|-------------|------------|
    | 1        | 101         | Laptop     |
    | 2        | 102         | Smartphone |
    ```

    **customers**:

    ```plaintext
    | customer_id | customer_name | city     |
    |-------------|---------------|----------|
    | 101         | Alice         | New York |
    | 102         | Bob           | Chicago  |
    ```

---

## Summary of Normalization Concepts

Normal Form  | Description                                                                                      | Goal
-------------|--------------------------------------------------------------------------------------------------|---------------------------
1NF          | Each column contains only atomic values, and each row is unique                                  | Eliminate repeating groups
2NF          | Table is in 1NF, and non-key columns depend on the entire primary key                            | Remove partial dependencies
3NF          | Table is in 2NF, and non-key columns depend only on the primary key                              | Remove transitive dependencies
{:class="table table-striped"}

By following normalization principles, you can create efficient, scalable, and consistent databases. In the next lesson, we’ll go further into Boyce-Codd Normal Form (BCNF) for advanced normalization techniques.

---
