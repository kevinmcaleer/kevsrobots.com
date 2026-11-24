---
title: Course Summary
description: >-
    Throughout this course, you have gained a solid foundation in working with databases using Python and SQLite3
layout: lesson
type: page
cover: /learn/sqlite3/assets/banners/11_summary.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to recall the main ideas of the course and choose a project to practise them on.

Congratulations! You have completed the `Create Databases with Python and SQLite3` course. Throughout this course, you have gained a solid foundation in working with databases using Python and SQLite3. Let's summarize what we have covered:

### Module 1: Introduction to Databases

- Overview of databases and their importance
- Introduction to SQLite as a lightweight database engine
- Understanding basic database concepts: tables, records, columns, and primary keys

### Module 2: Getting Started with Python and SQLite3

- Setting up Python and SQLite3
- Establishing a connection to a database
- Executing SQL statements: creating tables, inserting data, querying data

### Module 3: Creating Tables and Schema Design

- Understanding table structure: rows (records) and columns
- Creating tables with data types and constraints
- Defining primary keys and relationships between tables

### Module 4: Inserting and Retrieving Data

- Inserting data into tables
- Querying data using SELECT statements
- Filtering, sorting, and limiting data retrieval

### Module 5: Updating and Deleting Data

- Updating existing records in tables
- Deleting data from tables
- Handling cascading deletes and referential integrity

### Module 6: Querying and Joining Tables

- Working with multiple tables
- Performing join operations: inner join, outer join
- Using table aliases and advanced SQL techniques

### Module 7: Advanced SQL Queries

- Aggregating data with GROUP BY and HAVING
- Using built-in functions and expressions
- Working with subqueries and nested queries

### Module 8: Database Transactions and Error Handling

- Understanding database transactions and ACID properties
- Working with transactions: committing, rolling back and the `with` block
- Catching and handling SQLite errors

### Module 9: Working with Indexes and Performance Optimization

- Understanding indexes and their importance
- Creating indexes for improved performance
- Analyzing query performance and optimizing execution

### Module 10: Securing and Backing Up SQLite Databases

- Securing databases with file permissions, access controls and encryption options
- Implementing backup strategies: file copy, the backup API, SQL dump, database snapshots
- Testing and restoring backups, disaster recovery planning

## Course Conclusion

By completing this course, you have gained essential knowledge and practical skills in creating databases with Python and SQLite3. You are now capable of:

- Setting up Python and SQLite3 to work with databases
- Designing and creating tables with proper schema design
- Inserting, querying, updating, and deleting data in SQLite tables
- Joining and querying data from multiple tables
- Optimizing query performance and working with indexes
- Securing databases and implementing backup strategies

Databases are a fundamental part of modern applications, and with the skills you have acquired, you are well-equipped to develop robust and efficient database-driven applications using Python and SQLite3.

Remember, practice is key to mastering these concepts. Continue exploring and experimenting with databases to reinforce your understanding and further enhance your skills.

Best of luck with your future endeavors in database development!

---

## What's next?

- The bonus lessons that follow cover database design (normal forms), a complete demo program and desktop tools
- [Getting Started with SQL](/learn/sql/) - more practice with the SQL language
- [Using Redis with Python](/learn/redis/) - a super-fast in-memory data store for caching and messaging
- [Data Manipulation with Pandas and Numpy](/learn/pandas_and_numpy/) - analyse your data. `pandas.read_sql()` can load a query straight into a DataFrame

---

## Try it Yourself

Pick one project and build it with everything you've learned:

1. A parts inventory for your robot box: a `parts` table with constraints, a script to add parts with `?` placeholders, and a query for parts running low.
2. A sensor log: a table with a timestamp and a reading, a function that inserts a reading, and a query for the average of the last 10 readings.
3. A library with authors, genres and books, and a view that joins them.
4. Challenge: add an index, time a query before and after, and write a `backup()` script that you run each time you finish.

## Common Issues

- **Problem**: Your project works once, then fails the second time you run it.
- **Solution**: Use `CREATE TABLE IF NOT EXISTS`, and think about what should happen to rows that are already there (`INSERT OR IGNORE`, or a `UNIQUE` column).
- **Why**: A database file keeps its data between runs, unlike the variables in your program.

---

## Check your understanding

{% quiz %}
? Which two habits protect you from lost changes and from SQL injection?
- [x] Committing (or using `with connection:`) and passing values with `?` placeholders
- [ ] Closing the connection and using f-strings
- [ ] Adding an index and running `ANALYZE`
> Placeholders keep data out of the SQL text, and committing saves the changes.

? When would you reach for `connection.backup()`?
- [x] To copy a database safely while a program is using it
- [ ] To speed up a query
- [ ] To encrypt the file
> It takes a consistent copy even while the database is in use.

? Which course would you take to practise SQL further?
- [ ] Getting Started with Python
- [x] Getting Started with SQL
> The next steps below link to it.
{% endquiz %}

---
