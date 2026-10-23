---
title: Advanced SQL Queries
description: Module 7 delves into advanced SQL queries in SQLite
layout: lesson
---

Module 7 delves into advanced SQL queries in SQLite. You will learn about aggregating data, using grouping and filtering conditions, and working with subqueries and nested queries.

The examples use the `library.db` tables from the previous module.

### Aggregating Data with GROUP BY

The `GROUP BY` clause allows us to group rows based on specific columns and perform aggregate functions on those groups.

#### Grouping Data

```python
# Grouping data example
query = '''
    SELECT genre_id, COUNT(*) AS count
    FROM books
    GROUP BY genre_id
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
    SELECT genre_id, COUNT(*) AS count
    FROM books
    GROUP BY genre_id
    HAVING COUNT(*) > 2
'''
result = connection.execute(query)
data = result.fetchall()
```

In this example, we retrieve genres with more than 2 books by using the `HAVING` clause to filter the groups. `WHERE` filters rows *before* grouping; `HAVING` filters the groups *after*.

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
    SELECT title, year
    FROM books
    WHERE price > (
        SELECT AVG(price)
        FROM books
    )
'''
result = connection.execute(query)
data = result.fetchall()
```

In this example, the subquery works out the average price, and the outer query returns the books that cost more than that.

### Advanced SQL Techniques

SQLite supports advanced SQL techniques, including views, common table expressions (CTEs), and window functions. These techniques can provide powerful ways to structure and manipulate data in queries.

#### Views

Views are virtual tables based on the result of a query. They allow us to simplify complex queries and reuse query logic.

```python
with connection:
    connection.execute('''
        CREATE VIEW IF NOT EXISTS book_details AS
        SELECT b.title, a.name AS author, b.year
        FROM books AS b LEFT JOIN authors AS a ON b.author_id = a.id
    ''')

print(connection.execute("SELECT * FROM book_details WHERE year > 2000").fetchall())
```

#### Common Table Expressions (CTEs)

CTEs are named temporary result sets that we can reference within a query. They provide a convenient way to break down complex queries into smaller, more manageable parts.

```python
query = '''
    WITH recent AS (
        SELECT * FROM books WHERE year > 2000
    )
    SELECT COUNT(*), AVG(price) FROM recent
'''
print(connection.execute(query).fetchone())
```

#### Window Functions

Window functions perform calculations across a set of rows in a query result. They allow us to perform calculations such as ranking, row numbering, and cumulative sums.

```python
query = '''
    SELECT title, price,
           RANK() OVER (ORDER BY price DESC) AS price_rank
    FROM books
'''
for row in connection.execute(query):
    print(row)
```

Unlike `GROUP BY`, a window function keeps every row and adds the calculated value alongside it.

By mastering advanced SQL queries and techniques, you will have the skills to perform complex data analysis and manipulation within SQLite databases.

---

{% quiz %}
? What's the difference between `WHERE` and `HAVING`?
- [x] `WHERE` filters rows before grouping, `HAVING` filters groups after
- [ ] They're the same
- [ ] `HAVING` only works with text columns
> Use `HAVING` for conditions on aggregates like `COUNT(*)`.

? Which of these returns one row per book with a ranking alongside it?
- [ ] `GROUP BY price`
- [x] `RANK() OVER (ORDER BY price DESC)`
- [ ] `MAX(price)`
> Window functions keep every row and add the calculated value.
{% endquiz %}

---
