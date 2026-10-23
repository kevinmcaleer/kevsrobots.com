---
title: Working with Indexes and Performance Optimization
description: >-
    Module 9 focuses on optimizing the performance of SQLite databases
layout: lesson
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
