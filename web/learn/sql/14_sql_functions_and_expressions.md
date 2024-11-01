---
layout: lesson
title: SQL Functions and Expressions
author: Kevin McAleer
type: page
cover: /learn/sql/assets/cover.jpg
date: 2024-11-01
previous: 13_views_and_indexes.html
next: 15_wrap_up_and_final_project.html
description: Learn to use SQL functions and expressions to manipulate and analyze
  data in queries
percent: 90
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


In this lesson, we’ll explore **SQL functions and expressions** that enhance your ability to manipulate, analyze, and transform data. SQL provides a variety of functions, including string, numeric, date, and aggregate functions, that make it easier to handle different types of data efficiently.

---

## 1. What are SQL Functions?

SQL functions are built-in commands that perform specific operations on data and return a single result. Functions can be applied to columns in your database to manipulate or calculate values directly within your queries.

### Types of SQL Functions

- **Aggregate Functions**: SUM, COUNT, AVG, MIN, MAX
- **String Functions**: CONCAT, LENGTH, UPPER, LOWER
- **Numeric Functions**: ROUND, CEIL, FLOOR
- **Date Functions**: NOW, DATE, YEAR, MONTH, DAY

---

## 2. Aggregate Functions

Aggregate functions operate on a set of values and return a single result. They are commonly used in conjunction with the `GROUP BY` clause.

### Common Aggregate Functions

Function | Description                            | Example
---------|----------------------------------------|------------------------------
`COUNT`  | Counts the number of rows              | `SELECT COUNT(*) FROM users;`
`SUM`    | Calculates the sum of a numeric column | `SELECT SUM(salary) FROM employees;`
`AVG`    | Calculates the average value           | `SELECT AVG(salary) FROM employees;`
`MIN`    | Returns the minimum value              | `SELECT MIN(salary) FROM employees;`
`MAX`    | Returns the maximum value              | `SELECT MAX(salary) FROM employees;`
{:class="table table-striped"}

- **Example**: Find the total salary and average salary of employees.

    ```sql
    SELECT SUM(salary) AS total_salary, AVG(salary) AS average_salary
    FROM employees;
    ```

---

## 3. String Functions

String functions allow you to manipulate text data. These functions are useful for formatting, concatenating, and altering strings.

### Common String Functions

Function       | Description                               | Example
---------------|-------------------------------------------|---------------------------------------
`CONCAT`       | Concatenates two or more strings          | `SELECT CONCAT(first_name, ' ', last_name) FROM users;`
`LENGTH`       | Returns the length of a string            | `SELECT LENGTH(name) FROM users;`
`UPPER`        | Converts text to uppercase                | `SELECT UPPER(name) FROM users;`
`LOWER`        | Converts text to lowercase                | `SELECT LOWER(name) FROM users;`
`SUBSTRING`    | Extracts a substring from a string        | `SELECT SUBSTRING(name, 1, 3) FROM users;`
{:class="table table-striped"}

- **Example**: Retrieve employee names in uppercase and show the length of each name.

    ```sql
    SELECT UPPER(name) AS upper_name, LENGTH(name) AS name_length
    FROM employees;
    ```

---

## 4. Numeric Functions

Numeric functions are used to perform mathematical operations on numeric data.

### Common Numeric Functions

Function      | Description                                   | Example
--------------|-----------------------------------------------|-----------------------------------
`ROUND`       | Rounds a number to a specified number of decimals | `SELECT ROUND(salary, 2) FROM employees;`
`CEIL`        | Rounds a number up to the nearest integer         | `SELECT CEIL(salary) FROM employees;`
`FLOOR`       | Rounds a number down to the nearest integer       | `SELECT FLOOR(salary) FROM employees;`
`ABS`         | Returns the absolute value of a number            | `SELECT ABS(salary - 50000) FROM employees;`
{:class="table table-striped"}

- **Example**: Round each employee’s salary to the nearest hundred.

    ```sql
    SELECT name, ROUND(salary, -2) AS rounded_salary
    FROM employees;
    ```

---

## 5. Date Functions

Date functions allow you to manipulate and extract information from date and time data.

### Common Date Functions

Function      | Description                                         | Example
--------------|-----------------------------------------------------|-----------------------------------
`NOW`         | Returns the current date and time                   | `SELECT NOW();`
`CURDATE`     | Returns the current date                            | `SELECT CURDATE();`
`YEAR`        | Extracts the year from a date                       | `SELECT YEAR(birth_date) FROM users;`
`MONTH`       | Extracts the month from a date                      | `SELECT MONTH(birth_date) FROM users;`
`DAY`         | Extracts the day from a date                        | `SELECT DAY(birth_date) FROM users;`
`DATEDIFF`    | Returns the difference in days between two dates    | `SELECT DATEDIFF(NOW(), hire_date) FROM employees;`
{:class="table table-striped"}

- **Example**: Find the number of days each employee has been with the company, based on their hire date.

    ```sql
    SELECT name, DATEDIFF(NOW(), hire_date) AS days_with_company
    FROM employees;
    ```

---

## 6. Using Expressions for Calculations

Expressions in SQL allow you to perform calculations or manipulate data directly in queries.

### Example: Calculating a 10% Bonus

- **Example**: Calculate a 10% bonus for each employee based on their current salary.

    ```sql
    SELECT name, salary, salary * 0.10 AS bonus
    FROM employees;
    ```

### Example: Combining Columns with CONCAT

- **Example**: Combine `first_name` and `last_name` into a single `full_name` column.

    ```sql
    SELECT CONCAT(first_name, ' ', last_name) AS full_name
    FROM employees;
    ```

### Example: Date Arithmetic

- **Example**: Calculate each employee’s next anniversary by adding one year to their `hire_date`.

    ```sql
    SELECT name, hire_date, DATE_ADD(hire_date, INTERVAL 1 YEAR) AS next_anniversary
    FROM employees;
    ```

---

## Practice Exercise: Using SQL Functions and Expressions

1. **Calculate Total and Average Salary**: Find the total and average salary of all employees.

    ```sql
    SELECT SUM(salary) AS total_salary, AVG(salary) AS average_salary
    FROM employees;
    ```

2. **String Manipulation**: Retrieve the first three letters of each employee’s name and display it in uppercase.

    ```sql
    SELECT UPPER(SUBSTRING(name, 1, 3)) AS name_prefix
    FROM employees;
    ```

3. **Round Salaries**: Display each employee’s salary rounded to the nearest thousand.

    ```sql
    SELECT name, ROUND(salary, -3) AS rounded_salary
    FROM employees;
    ```

4. **Date Difference**: Calculate the number of days each employee has worked at the company based on their hire date.

    ```sql
    SELECT name, DATEDIFF(NOW(), hire_date) AS days_with_company
    FROM employees;
    ```

5. **Calculate Bonuses**: Calculate a 5% bonus for each employee based on their salary.

    ```sql
    SELECT name, salary, salary * 0.05 AS bonus
    FROM employees;
    ```

---

## Summary of SQL Functions and Expressions

Here’s a summary of the SQL functions and expressions covered in this lesson:

Function Category | Common Functions                            | Purpose
------------------|---------------------------------------------|-------------------------------------------------
Aggregate         | COUNT, SUM, AVG, MIN, MAX                  | Operate on sets of values to return a single result
String            | CONCAT, LENGTH, UPPER, LOWER, SUBSTRING    | Manipulate text data in queries
Numeric           | ROUND, CEIL, FLOOR, ABS                    | Perform mathematical operations on numeric data
Date              | NOW, CURDATE, YEAR, MONTH, DATEDIFF        | Work with date and time data in queries
Expressions       | Basic arithmetic operations (`+`, `-`, `*`, `/`) | Perform calculations directly in queries
{:class="table table-striped"}

With these SQL functions and expressions, you can perform a wide range of data manipulations directly within your queries. In the next lesson, we’ll wrap up the course with a final project to apply everything you’ve learned.

---
