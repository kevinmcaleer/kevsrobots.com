---
layout: lesson
title: Subqueries and Nested Queries
author: Kevin McAleer
type: page
cover: /learn/sql/assets/cover.jpg
date: 2024-11-01
previous: 11_boyce_codd_normal_form.html
next: 13_views_and_indexes.html
description: Using subqueries in SQL to perform complex queries within queries
percent: 78
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


In this lesson, we’ll explore **subqueries** (also known as nested queries), which allow you to perform queries within queries. Subqueries are powerful tools for retrieving data based on the results of other queries, enabling more advanced data analysis and manipulation.

---

## 1. What is a Subquery?

A **subquery** is a query nested within another SQL query. Subqueries are typically enclosed in parentheses and can appear in various parts of the main query, including the `SELECT`, `FROM`, and `WHERE` clauses.

### Why Use Subqueries?

Subqueries allow you to:
- Retrieve data based on the results of another query
- Simplify complex queries by breaking them into smaller parts
- Make queries more readable and maintainable

---

## 2. Types of Subqueries

Subqueries can be categorized based on how and where they’re used:

- **Single-row Subqueries**: Return a single value.
- **Multi-row Subqueries**: Return multiple values.
- **Correlated Subqueries**: Depend on values from the outer query and are evaluated row by row.
- **Non-correlated Subqueries**: Run independently of the outer query and are evaluated once.

---

## 3. Subqueries in the `WHERE` Clause

Subqueries are often used in the `WHERE` clause to filter results based on the output of another query.

### Example: Single-row Subquery in `WHERE`

Suppose we have two tables, `employees` and `departments`.

**employees**

```plaintext
| emp_id | name     | salary | dept_id |
|--------|----------|--------|---------|
| 1      | Alice    | 50000  | 1       |
| 2      | Bob      | 60000  | 2       |
| 3      | Charlie  | 70000  | 1       |
```

**departments**

```plaintext
| dept_id | dept_name     |
|---------|---------------|
| 1       | Engineering   |
| 2       | Sales         |
```

- **Example**: Find employees who earn more than the average salary in the `employees` table.

    ```sql
    SELECT name, salary
    FROM employees
    WHERE salary > (SELECT AVG(salary) FROM employees);
    ```

In this example, the subquery `(SELECT AVG(salary) FROM employees)` returns a single value (the average salary), which is then used as a condition in the `WHERE` clause.

---

## 4. Subqueries in the `SELECT` Clause

Subqueries can also be used in the `SELECT` clause to calculate values for each row in the result set.

### Example: Subquery in `SELECT`

- **Example**: List each employee’s salary and the average salary of their department.

    ```sql
    SELECT name, salary,
           (SELECT AVG(salary)
            FROM employees AS e
            WHERE e.dept_id = employees.dept_id) AS dept_avg_salary
    FROM employees;
    ```

In this example, the subquery calculates the average salary for the department of each employee and displays it as `dept_avg_salary`.

---

## 5. Subqueries in the `FROM` Clause

Subqueries can be used in the `FROM` clause to create a temporary table that the main query can then use.

### Example: Subquery in `FROM`

- **Example**: Find the highest salary by department.

    ```sql
    SELECT dept_name, MAX(salary) AS max_salary
    FROM (SELECT departments.dept_name, employees.salary
          FROM employees
          JOIN departments ON employees.dept_id = departments.dept_id) AS dept_salaries
    GROUP BY dept_name;
    ```

In this example, the subquery creates a temporary table `dept_salaries` that includes both department names and salaries. The main query then groups this temporary table by `dept_name` and finds the maximum salary for each department.

---

## 6. Correlated Subqueries

A **correlated subquery** depends on the outer query for its values and is executed once for each row processed by the outer query. These are useful for filtering or calculating data that depends on row-by-row conditions.

### Example: Correlated Subquery

- **Example**: Find employees whose salary is above the average salary in their department.

    ```sql
    SELECT name, salary
    FROM employees AS e1
    WHERE salary > (SELECT AVG(salary)
                    FROM employees AS e2
                    WHERE e1.dept_id = e2.dept_id);
    ```

In this example, the subquery calculates the average salary for each department. The outer query then retrieves employees whose salary is above the department average.

---

## 7. Using `IN` with Subqueries

The `IN` operator is commonly used with subqueries to filter results based on a list of values returned by the subquery.

### Example: Subquery with `IN`

- **Example**: Find employees who work in departments with an average salary greater than $60,000.

    ```sql
    SELECT name
    FROM employees
    WHERE dept_id IN (SELECT dept_id
                      FROM employees
                      GROUP BY dept_id
                      HAVING AVG(salary) > 60000);
    ```

In this example, the subquery returns `dept_id` values where the department’s average salary exceeds $60,000. The outer query then retrieves employees who work in these departments.

---

## Practice Exercise: Using Subqueries

1. **Filter with a Subquery**: Retrieve employees with a salary greater than the average salary.

    ```sql
    SELECT name, salary
    FROM employees
    WHERE salary > (SELECT AVG(salary) FROM employees);
    ```

2. **Correlated Subquery**: Find employees who earn more than the average salary of their department.

    ```sql
    SELECT name, salary
    FROM employees AS e1
    WHERE salary > (SELECT AVG(salary)
                    FROM employees AS e2
                    WHERE e1.dept_id = e2.dept_id);
    ```

3. **Subquery with `IN`**: Find employees working in departments with an average salary above $50,000.

    ```sql
    SELECT name
    FROM employees
    WHERE dept_id IN (SELECT dept_id
                      FROM employees
                      GROUP BY dept_id
                      HAVING AVG(salary) > 50000);
    ```

4. **Subquery in `SELECT`**: Display each employee’s name, salary, and the average salary of their department.

    ```sql
    SELECT name, salary,
           (SELECT AVG(salary)
            FROM employees AS e
            WHERE e.dept_id = employees.dept_id) AS dept_avg_salary
    FROM employees;
    ```

---

## Summary of Subquery Types

Here’s a summary of the various ways subqueries can be used in SQL:

Subquery Location   | Purpose                                        | Example
--------------------|------------------------------------------------|------------------------------------------------
`WHERE` Clause      | Filter results based on subquery output        | `WHERE salary > (SELECT AVG(salary) ...)`
`SELECT` Clause     | Calculate additional values for each row       | `SELECT name, (SELECT AVG(salary) ...) AS avg`
`FROM` Clause       | Create a temporary table for main query        | `FROM (SELECT dept_name, salary ...) AS temp`
Correlated Subquery | Row-by-row filtering based on outer query rows | `WHERE salary > (SELECT AVG(salary) ...)`
{:class="table table-striped"}

Subqueries add powerful functionality to SQL queries, allowing for complex filtering and data manipulation. In the next lesson, we’ll cover views and indexes, which enhance the performance and usability of your database.

---
