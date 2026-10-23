---
layout: lesson
title: Working with Indexes and Performance Optimization
author: Kevin McAleer
type: page
cover: /learn/sqlite3/assets/database.png
date: 2023-07-15
previous: 08_transations_and_error_handling.html
next: 10_securing_and_backing_up.html
description: Module 9 focuses on optimizing the performance of SQLite databases
percent: 50
duration: 4
quiz_total: 24
navigation:
- name: Create Databases with Python and SQLite3
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


Module 9 focuses on optimizing the performance of SQLite databases by utilizing indexes and employing various techniques to improve query execution speed.

### Understanding Indexes

Indexes are data structures that improve the speed of data retrieval operations in a database. They provide a way to quickly locate and access specific data within a table.

#### Primary Key Index

Every SQLite table is stored in order of its row id, and an `INTEGER PRIMARY KEY` column *is* the row id. So looking a row up by its primary key is already fast, without any extra index. For other kinds of primary key, and for `UNIQUE` columns, SQLite creates an index automatically.

#### Creating Additional Indexes

To improve the performance of queries that involve filtering, sorting, or joining, we can create additional indexes on specific columns.

```python
# Create an index on a column
connection.execute("CREATE INDEX IF NOT EXISTS idx_books_author ON books(author)")
```

In this example, we create an index named "idx_books_author" on the "author" column of the "books" table. This index enhances query performance when filtering or sorting based on the "author" column.

### Analyzing Query Performance

SQLite provides the `EXPLAIN QUERY PLAN` command, which helps analyze the execution plan of a query and identify areas for optimization.

```python
# Analyze query performance
query = "SELECT * FROM books WHERE author = ?"
result = connection.execute("EXPLAIN QUERY PLAN " + query, ("F. Scott Fitzgerald",))
data = result.fetchall()
```

By examining the output of the `EXPLAIN QUERY PLAN` command, we can identify potential performance bottlenecks and make informed decisions on index creation or query optimization.

Print `data` and look at the last item of each row:

- `SCAN books` means SQLite reads every row to find the matches. Fine for small tables, slow for big ones
- `SEARCH books USING INDEX idx_books_author (author=?)` means it jumps straight to the matching rows using the index

Try it before and after creating the index to see the difference.

### Optimizing Query Execution

There are various techniques to optimize query execution and improve overall database performance.

#### Filtering Rows

To improve query performance, it's important to limit the number of rows returned by a query. Use the `WHERE` clause to filter rows based on specific conditions and retrieve only the necessary data.

#### Limiting Result Sets

If you only need a subset of records, use the `LIMIT` clause to restrict the number of rows returned by a query. This helps reduce the data transferred and improves query response time.

#### Using Joins Efficiently

Joins can impact query performance, especially when dealing with large tables. Be selective when joining tables and use appropriate indexes on join columns.

#### Avoiding Unnecessary Data Retrieval

Retrieve only the columns that are required for a particular query. Avoid selecting unnecessary columns to reduce the amount of data transferred and improve query performance.

### Monitoring and Tuning

Regular monitoring and tuning of your SQLite database can help identify performance issues and optimize query execution.

#### Monitoring Tools

Running `ANALYZE` makes SQLite gather statistics about your tables and indexes (stored in the `sqlite_stat1` table), which helps it choose the best plan for each query. The `sqlite3_analyzer` command-line utility from the SQLite website reports how space is used in a database file.

#### Query Optimization Techniques

Optimizing queries involves techniques such as using appropriate indexes, rewriting complex queries, and denormalizing data in certain cases.

### Performance Testing and Benchmarking

To measure the performance improvements achieved through optimizations, it's important to conduct performance testing and benchmarking on representative workloads.

### Continuous Improvement

Database performance is an ongoing concern. Regularly monitor performance, analyze query execution plans, and fine-tune indexes and queries to ensure optimal performance over time.

By understanding indexes and employing performance optimization techniques, you can significantly enhance the speed and efficiency of your SQLite databases.

---

> ## Indexes aren't free
>
> Each index takes up space and has to be updated on every insert, update and delete. Add indexes for columns you search, sort or join on often, not for every column.

---

{% quiz %}
? `EXPLAIN QUERY PLAN` shows `SCAN books`. What does that mean?
- [x] SQLite is reading every row of the table
- [ ] The query is using an index
- [ ] The table is corrupt
> A scan checks every row. `SEARCH ... USING INDEX` is what you want for big tables.

? Why not put an index on every column?
- [ ] SQLite only allows one index per table
- [x] Indexes use space and slow down inserts, updates and deletes
- [ ] Indexes make SELECT queries slower
> Each index must be kept up to date whenever the data changes.
{% endquiz %}

---
