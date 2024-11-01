---
layout: lesson
title: Wrap-up and Final Project
author: Kevin McAleer
type: page
cover: /learn/sql/assets/cover.jpg
date: 2024-11-01
previous: 14_sql_functions_and_expressions.html
description: Apply everything you've learned in this SQL course to a final project
percent: 100
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


In this lesson, we’ll review the key topics covered in this course and introduce a final project that lets you apply everything you’ve learned to a real-world scenario.

---

## 1. Course Recap

Over the past lessons, you’ve built a strong foundation in SQL, covering the following key concepts:

1. **SQL Basics**: Learned to create, read, update, and delete data using essential SQL commands (`SELECT`, `INSERT`, `UPDATE`, `DELETE`).
2. **Table Structure and Data Types**: Structured data into tables and chose appropriate data types.
3. **Filtering and Sorting**: Used `WHERE`, `ORDER BY`, and `LIMIT` to refine and organize query results.
4. **Advanced Filtering**: Combined conditions with `AND`, `OR`, and `CASE` statements for dynamic filtering.
5. **Working with Joins**: Combined data from multiple tables using various types of joins (INNER, LEFT, RIGHT, FULL).
6. **Grouping and Aggregating**: Used `GROUP BY` with aggregate functions (SUM, COUNT, AVG, MIN, MAX) to summarize data.
7. **Database Normalization**: Designed efficient databases using normalization up to Boyce-Codd Normal Form (BCNF).
8. **Subqueries and Nested Queries**: Wrote queries within queries to retrieve data based on complex conditions.
9. **Views and Indexes**: Created views for reusable queries and indexes to improve query performance.
10. **SQL Functions and Expressions**: Used SQL functions to manipulate and analyze data within queries.

---

## 2. Final Project: Building a Simple Sales Database

For this project, you’ll create a small database to manage sales data for a fictional company. This database will allow you to track customers, products, orders, and sales, applying the concepts and skills you’ve developed throughout the course.

### Project Requirements

Your database should include the following tables:

1. **customers**:
   - `customer_id`: Unique identifier for each customer (PRIMARY KEY)
   - `name`: Customer’s name
   - `email`: Customer’s email (ensure it’s unique)
   - `join_date`: Date the customer joined

2. **products**:
   - `product_id`: Unique identifier for each product (PRIMARY KEY)
   - `product_name`: Name of the product
   - `price`: Price of the product

3. **orders**:
   - `order_id`: Unique identifier for each order (PRIMARY KEY)
   - `customer_id`: ID of the customer who placed the order (FOREIGN KEY referencing `customers`)
   - `order_date`: Date the order was placed

4. **order_items**:
   - `order_item_id`: Unique identifier for each item in an order (PRIMARY KEY)
   - `order_id`: ID of the associated order (FOREIGN KEY referencing `orders`)
   - `product_id`: ID of the product ordered (FOREIGN KEY referencing `products`)
   - `quantity`: Number of items ordered
   - `total_price`: Calculated as `quantity * price`

### Step-by-Step Instructions

#### Step 1: Create the Tables and Set Up Relationships

1. Define each table according to the project requirements.
2. Set up primary keys and foreign keys to establish relationships.
3. Use appropriate data types for each column.

#### Step 2: Insert Sample Data

Populate each table with sample data to test your queries. Include at least:

- 3–5 customers
- 3–5 products
- 2–3 orders, each with multiple items in `order_items`

#### Step 3: Write Queries to Answer Business Questions

Use the following questions as prompts to write SQL queries and test your understanding:

1. **Retrieve all customer details for customers who joined in the last year**.
    ```sql
    SELECT * FROM customers
    WHERE join_date > DATE_SUB(NOW(), INTERVAL 1 YEAR);
    ```

2. **List all products with a price greater than $50**.
    ```sql
    SELECT * FROM products
    WHERE price > 50;
    ```

3. **Find total sales for each product**.
    ```sql
    SELECT p.product_name, SUM(oi.total_price) AS total_sales
    FROM order_items AS oi
    JOIN products AS p ON oi.product_id = p.product_id
    GROUP BY p.product_name;
    ```

4. **Identify customers who have placed more than one order**.
    ```sql
    SELECT c.name, COUNT(o.order_id) AS order_count
    FROM customers AS c
    JOIN orders AS o ON c.customer_id = o.customer_id
    GROUP BY c.customer_id
    HAVING order_count > 1;
    ```

5. **Retrieve the details of orders that include a specific product (e.g., “Laptop”)**.
    ```sql
    SELECT o.order_id, c.name, p.product_name, oi.quantity, oi.total_price
    FROM orders AS o
    JOIN order_items AS oi ON o.order_id = oi.order_id
    JOIN customers AS c ON o.customer_id = c.customer_id
    JOIN products AS p ON oi.product_id = p.product_id
    WHERE p.product_name = 'Laptop';
    ```

6. **Calculate the total revenue generated from all orders**.
    ```sql
    SELECT SUM(total_price) AS total_revenue
    FROM order_items;
    ```

#### Step 4: Create a View and an Index

1. **Create a View**: Define a view called `customer_orders` that shows each customer’s name, order ID, and order date.
    ```sql
    CREATE VIEW customer_orders AS
    SELECT c.name, o.order_id, o.order_date
    FROM customers AS c
    JOIN orders AS o ON c.customer_id = o.customer_id;
    ```

2. **Create an Index**: Create an index on the `order_date` column in the `orders` table to improve the performance of date-based queries.
    ```sql
    CREATE INDEX idx_order_date ON orders (order_date);
    ```

---

## 3. Additional Challenges (Optional)

If you’re ready for a challenge, try adding more complex requirements to your project:

1. **Implement a `discount` field** in the `order_items` table, then calculate `total_price` to reflect the discount.
2. **Write a report query** showing total revenue per month for the past year.
3. **Add a `categories` table** and link products to categories, then write queries to analyze sales by category.

---

## 4. Wrapping Up

Congratulations on completing the course! By now, you should have a solid foundation in SQL and relational database theory. You can:

- Create and manage databases and tables
- Retrieve, filter, and manipulate data effectively
- Design efficient database schemas using normalization
- Optimize performance with indexes and reusable views
- Write complex queries using joins, subqueries, and functions

This final project has helped you apply your skills to a practical scenario. Keep practicing, experiment with new datasets, and continue building on the concepts you’ve learned. SQL is a valuable skill that will serve you well across many projects and applications.

Thank you for participating in this course, and good luck with your SQL journey!

---
