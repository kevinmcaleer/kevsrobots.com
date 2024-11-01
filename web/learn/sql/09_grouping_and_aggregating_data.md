---
layout: lesson
title: Grouping and Aggregating Data
author: Kevin McAleer
type: page
cover: /learn/sql/assets/cover.jpg
date: 2024-11-01
previous: 08_working_with_joins.html
next: 10_introduction_to_normalization.html
description: Aggregate data with GROUP BY, COUNT, SUM, AVG, and other functions
percent: 60
duration: 5
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


In this lesson, we’ll dive into grouping and aggregating data in SQL. These techniques allow us to summarize and analyze data across multiple rows, making it easier to extract valuable insights.

---

## 1. Introduction to Aggregate Functions

Aggregate functions are SQL functions that operate on multiple rows of data to return a single result. Common aggregate functions include:

- **COUNT**: Counts the number of rows
- **SUM**: Calculates the sum of a numeric column
- **AVG**: Calculates the average of a numeric column
- **MIN**: Finds the minimum value in a column
- **MAX**: Finds the maximum value in a column

### Example: Basic Aggregate Functions

Suppose we have a `sales` table with the following data:

**sales**

```plaintext
| sale_id | customer_id | amount |
|---------|-------------|--------|
| 1       | 1           | 50     |
| 2       | 2           | 30     |
| 3       | 1           | 20     |
| 4       | 3           | 40     |
```

- **Example**: Find the total amount of all sales.

    ```sql
    SELECT SUM(amount) AS total_sales FROM sales;
    ```

- **Example**: Find the average sale amount.

    ```sql
    SELECT AVG(amount) AS average_sale FROM sales;
    ```

- **Example**: Count the number of sales.

    ```sql
    SELECT COUNT(sale_id) AS sale_count FROM sales;
    ```

---

## 2. Grouping Data with `GROUP BY`

The `GROUP BY` clause groups rows that have the same values in specified columns, allowing us to perform aggregate functions on each group.

### Basic `GROUP BY` Syntax

```sql
SELECT column1, aggregate_function(column2)
FROM table_name
GROUP BY column1;
```

### Example: Grouping Sales by Customer

- **Example**: Find the total amount spent by each customer.

    ```sql
    SELECT customer_id, SUM(amount) AS total_spent
    FROM sales
    GROUP BY customer_id;
    ```

**Result**:

```plaintext
| customer_id | total_spent |
|-------------|-------------|
| 1           | 70          |
| 2           | 30          |
| 3           | 40          |
```

Each customer’s total spending is calculated separately.

---

## 3. Filtering Groups with `HAVING`

The `HAVING` clause allows you to filter groups based on aggregate values. Unlike `WHERE`, which filters rows, `HAVING` filters groups created by `GROUP BY`.

### Basic `HAVING` Syntax

```sql
SELECT column1, aggregate_function(column2)
FROM table_name
GROUP BY column1
HAVING condition;
```

### Example: Using `HAVING` to Filter Grouped Results

- **Example**: Find customers who spent more than $50.

    ```sql
    SELECT customer_id, SUM(amount) AS total_spent
    FROM sales
    GROUP BY customer_id
    HAVING total_spent > 50;
    ```

**Result**:

```plaintext
| customer_id | total_spent |
|-------------|-------------|
| 1           | 70          |
```

Only customers with a total spending of over $50 are included in the result.

---

## 4. Combining `GROUP BY` with Multiple Columns

You can group data by multiple columns to create more specific groupings.

### Example: Grouping by Multiple Columns

Suppose we add a `product` column to the `sales` table:

**sales**

```plaintext
| sale_id | customer_id | product | amount |
|---------|-------------|---------|--------|
| 1       | 1           | Laptop  | 50     |
| 2       | 2           | Phone   | 30     |
| 3       | 1           | Phone   | 20     |
| 4       | 3           | Laptop  | 40     |
```

- **Example**: Find the total amount spent by each customer on each product.

    ```sql
    SELECT customer_id, product, SUM(amount) AS total_spent
    FROM sales
    GROUP BY customer_id, product;
    ```

**Result**:

```plaintext
| customer_id | product | total_spent |
|-------------|---------|-------------|
| 1           | Laptop  | 50          |
| 1           | Phone   | 20          |
| 2           | Phone   | 30          |
| 3           | Laptop  | 40          |
```

This query calculates the total amount spent on each product by each customer.

---

## 5. Common Aggregate Functions Explained

Here’s a quick overview of some of the most useful aggregate functions:

Function | Description                                        | Example
---------|----------------------------------------------------|--------------------------------
`COUNT`  | Counts the number of rows in a result set          | `COUNT(sale_id)`
`SUM`    | Adds up the values in a numeric column             | `SUM(amount)`
`AVG`    | Calculates the average value of a numeric column   | `AVG(amount)`
`MIN`    | Finds the minimum value in a column                | `MIN(amount)`
`MAX`    | Finds the maximum value in a column                | `MAX(amount)`
{:class="table table-striped"}

---

## 6. Practice Exercise: Grouping and Aggregating Data

1. **Total Sales**: Find the total sales amount.

    ```sql
    SELECT SUM(amount) AS total_sales FROM sales;
    ```

2. **Total Sales by Customer**: Find the total amount each customer has spent.

    ```sql
    SELECT customer_id, SUM(amount) AS total_spent
    FROM sales
    GROUP BY customer_id;
    ```

3. **Filter Groups with HAVING**: Retrieve only customers who have spent more than $50.

    ```sql
    SELECT customer_id, SUM(amount) AS total_spent
    FROM sales
    GROUP BY customer_id
    HAVING total_spent > 50;
    ```

4. **Count of Sales per Product**: Count the number of sales transactions for each product.

    ```sql
    SELECT product, COUNT(sale_id) AS sale_count
    FROM sales
    GROUP BY product;
    ```

---

## Summary of Grouping and Aggregating Data

Here’s a summary of the key concepts and commands covered in this lesson:

Command/Function     | Description                                                      | Example
---------------------|------------------------------------------------------------------|---------------------------------------------
`GROUP BY`           | Groups rows by specified columns                                 | `GROUP BY customer_id`
`HAVING`             | Filters groups based on aggregate values                         | `HAVING SUM(amount) > 50`
`COUNT`              | Counts the number of rows in a result set                        | `COUNT(sale_id)`
`SUM`                | Adds up the values in a numeric column                           | `SUM(amount)`
`AVG`                | Calculates the average value of a numeric column                 | `AVG(amount)`
`MIN`                | Finds the minimum value in a column                              | `MIN(amount)`
`MAX`                | Finds the maximum value in a column                              | `MAX(amount)`
{:class="table table-striped"}

Using grouping and aggregation, you can generate meaningful summaries and insights from your data. In the next lesson, we’ll explore database normalization to help you organize your data effectively.

---
