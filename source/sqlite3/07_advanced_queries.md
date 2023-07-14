---
title: Advanced SQL Queries
description: Module 7 delves into advanced SQL queries in SQLite
layout: lesson
---

Module 7 delves into advanced SQL queries in SQLite. You will learn about aggregating data, using grouping and filtering conditions, and working with subqueries and nested queries.

### Aggregating Data with GROUP BY

The `GROUP BY` clause allows us to group rows based on specific columns and perform aggregate functions on those groups.

#### Grouping Data

```python
# Grouping data example
query = '''
    SELECT genre, COUNT(*) AS count
    FROM books
    GROUP BY genre
'''
result = connection.execute(query)
data = result.fetchall()
```

In this example, we group books based on their genre and use the `COUNT()` function to count the number of books in each genre.

### Filtering Grouped Data with HAVING

The `HAVING` clause allows us to filter groups based on specific conditions after the `GROUP BY` operation.

#### Filtering Grouped Data

```python
# Filtering grouped data example
query = '''
    SELECT genre, COUNT(*) AS count
    FROM books
    GROUP BY genre
    HAVING COUNT(*) > 5
'''
result = connection.execute(query)
data = result.fetchall()
```

In this example, we retrieve genres with more than 5 books by using the `HAVING` clause to filter the groups.

### Using Built-in Functions and Expressions

SQLite provides various built-in functions and expressions that allow us to perform calculations and transformations on data within queries.

#### Built-in Functions

```python
# Using built-in functions example
query = '''
    SELECT AVG(price) AS average_price, MAX(price) AS max_price, MIN(price) AS min_price
    FROM books
'''
result = connection.execute(query)
data = result.fetchall()
```

In this example, we calculate the average, maximum, and minimum prices of books using the `AVG()`, `MAX()`, and `MIN()` functions.

### Subqueries and Nested Queries

Subqueries, also known as nested queries, allow us to use the result of one query within another query.

#### Subqueries

```python
# Subquery example
query = '''
    SELECT title, author
    FROM books
    WHERE genre IN (
        SELECT genre
        FROM books
        WHERE year > 2000
    )
'''
result = connection.execute(query)
data = result.fetchall()
```

In this example, we retrieve the titles and authors of books that belong to genres found in a subquery. The subquery selects genres of books published after 2000.

### Advanced SQL Techniques

SQLite supports advanced SQL techniques, including views, common table expressions (CTEs), and window functions. These techniques can provide powerful ways to structure and manipulate data in queries.

#### Views

Views are virtual tables based on the result of a query. They allow us to simplify complex queries and reuse query logic.

#### Common Table Expressions (CTEs)

CTEs are named temporary result sets that we can reference within a query. They provide a convenient way to break down complex queries into smaller, more manageable parts.

#### Window Functions

Window functions perform calculations across a set of rows in a query result. They allow us to perform calculations such as ranking, row numbering, and cumulative sums.

By mastering advanced SQL queries and techniques, you will have the skills to perform complex data analysis and manipulation within SQLite databases.

---
