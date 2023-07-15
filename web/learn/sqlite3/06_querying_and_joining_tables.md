---
layout: lesson
title: Querying and Joining Tables
author: Kevin McAleer
type: page
cover: /learn/sqlite3/assets/database.png
previous: 05_updating_and_deleting_data.html
next: 07_advanced_queries.html
description: Module 6 focuses on more advanced querying techniques in SQLite, including
  working with multiple tables and performing joins
percent: 35
duration: 3
navigation:
- name: Creating Databases with Python and SQLite3 for Beginners
- content:
  - section: Overview
    content:
    - name: Course Overview
      link: 00_intro.html
  - section: Getting Started
    content:
    - name: Introduction to Databases
      link: 01_introduction_to_databases.html
    - name: Getting Started with Python and SQLite3
      link: 02_getting_started_with_sqlite3.html
    - name: Creating Tables and Schema Design
      link: 03_creating_tables_and_schema.html
    - name: Inserting and Retrieving Data
      link: 04.inserting_and_retrieving_data.html
    - name: Updating and Deleting Data
      link: 05_updating_and_deleting_data.html
  - section: Advanced topics
    content:
    - name: Querying and Joining Tables
      link: 06_querying_and_joining_tables.html
    - name: Advanced SQL Queries
      link: 07_advanced_queries.html
    - name: Database Transactions and Error Handling
      link: 08_transations_and_error_handling.html
    - name: Working with Indexes and Performance Optimization
      link: 09_indexes_and_performance.html
    - name: Securing and Backing Up SQLite Databases
      link: 10_securing_and_backing_up.html
  - section: Summary and Conclusion
    content:
    - name: Course Summary
      link: 11_summary.html
  - section: bonus
    content:
    - name: First Normal Form (1NF)
      link: 13_1nf.html
    - name: Second Normal Form (2NF)
      link: 14_2nf.html
    - name: Third Normal Form (3NF)
      link: 15_3nf.html
    - name: Boyce-Codd Normal Form (BCNF)
      link: 12_boyce_codd_normal_form.html
    - name: Demo program
      link: 16_demo_program.html
    - name: Tools
      link: 17_tools.html
---


Module 6 focuses on more advanced querying techniques in SQLite, including working with multiple tables and performing joins. You will learn how to retrieve data from multiple tables, establish relationships, and use join operations to combine data.

### Working with Multiple Tables

Real-world databases often involve multiple tables that are related to each other. To retrieve meaningful data, we need to understand how to work with these tables collectively.

#### Relationships Between Tables

Tables can be related to each other through common attributes or columns. Common types of relationships include:

- **One-to-One**: Each record in one table is associated with exactly one record in another table.
- **One-to-Many**: Each record in one table is associated with one or more records in another table.
- **Many-to-Many**: Multiple records in one table are associated with multiple records in another table, creating a junction table to represent the relationship.

Understanding these relationships allows us to retrieve related data effectively.

![Relationships diagram](assets/relationships.png){:class="img-fluid w-100"}

### Joining Tables

To combine data from multiple tables, we use SQL join operations. SQLite supports several types of joins, including inner joins, outer joins, left joins, and right joins.

#### Inner Join

The inner join returns only the records that have matching values in both tables being joined.

```python
# Inner join example
query = '''
    SELECT books.title, authors.name
    FROM books
    INNER JOIN authors ON books.author_id = authors.id
'''
result = connection.execute(query)
data = result.fetchall()
```

In this example, we join the "books" and "authors" tables based on the common column "author_id" and retrieve the book titles along with the corresponding author names.

#### Outer Join

Outer joins return all records from one table and the matching records from the other table. If no match is found, NULL values are filled in for the missing data.

```python
# Left outer join example
query = '''
    SELECT books.title, authors.name
    FROM books
    LEFT OUTER JOIN authors ON books.author_id = authors.id
'''
result = connection.execute(query)
data = result.fetchall()
```

In this example, we perform a left outer join between the "books" and "authors" tables, retrieving all books and the corresponding author names. If there is no matching author, NULL is returned for the author name.

### Joining Multiple Tables

We can join multiple tables together to retrieve data across related tables.

```python
# Joining multiple tables example
query = '''
    SELECT books.title, authors.name, genres.genre_name
    FROM books
    INNER JOIN authors ON books.author_id = authors.id
    INNER JOIN genres ON books.genre_id = genres.id
'''
result = connection.execute(query)
data = result.fetchall()
```

In this example, we join the "books," "authors," and "genres" tables, retrieving book titles along with author names and genre names.

### Aliasing Tables

To make SQL queries more readable, we can use table aliases to provide shorthand names for tables.

```python
# Using table aliases example
query = '''
    SELECT b.title, a.name
    FROM books AS b
    INNER JOIN authors AS a ON b.author_id = a.id
'''
result = connection.execute(query)
data = result.fetchall()
```

In this example, we use the aliases "b" and "a" for the "books" and "authors" tables, respectively, to simplify the query.

By understanding how to work with multiple tables and perform joins, you will be able to retrieve and combine data from various sources, providing more comprehensive insights from your databases.

---
