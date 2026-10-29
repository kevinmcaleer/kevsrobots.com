---
layout: lesson
title: Course Overview
author: Kevin McAleer
type: page
cover: /learn/sqlite3/assets/overview01.jpg
date: 2023-07-15
next: 01_introduction_to_databases.html
description: What you'll learn
percent: 5
duration: 3
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


![banner]({{page.cover}}){:class="cover"}

## Watch the video

{% include youtubeplayer.html id="8BCsBgt-5zY" %}

---

## Overview

Welcome to `Create Databases with Python and SQLite3`! Almost every useful program needs to remember things: high scores, sensor readings, a list of parts, the books on your shelf. In this course you'll learn how to store that data properly in a database, using Python and SQLite, which is built into Python so there's nothing extra to install.

---

## Course Content

Module | Title                                      | What you'll learn
-------|--------------------------------------------|------------------------------------------------------------------------------
1      | Introduction to Databases                  | What databases are, why we use them, and the basics: tables, rows, columns and keys
2      | Getting Started with Python and SQLite3    | Connecting to a database, creating a table, and saving and reading data
3      | Creating Tables and Schema Design          | Data types, primary keys, constraints and changing tables with `ALTER TABLE`
4      | Inserting and Retrieving Data              | `INSERT`, `SELECT`, filtering with `WHERE`, sorting, `LIMIT` and `OFFSET`
5      | Updating and Deleting Data                 | `UPDATE`, `DELETE`, batch changes and foreign keys
6      | Querying and Joining Tables                | Relationships between tables and the different kinds of `JOIN`
7      | Advanced SQL Queries                       | `GROUP BY`, `HAVING`, functions, subqueries, views, CTEs and window functions
8      | Transactions and Error Handling            | Committing, rolling back and catching errors
9      | Indexes and Performance                    | Making queries faster with indexes and `EXPLAIN QUERY PLAN`
10     | Securing and Backing Up                    | Keeping your data safe, and backing it up properly
Bonus  | Normal forms, a demo program and tools     | How to design tables well, a complete example, and desktop apps for SQLite
{:class="table table-striped"}

---

## Key Results

After completing this course, you will be able to:

* Create a SQLite database and tables from Python
* Insert, query, update and delete data safely
* Join related tables and summarise data with SQL
* Use transactions so your data is never left half-saved
* Speed up queries with indexes and back up your databases

---

## What you'll need

* A computer with Python 3 installed (Windows, macOS, Linux or a Raspberry Pi)
* A code editor such as [Thonny](https://thonny.org) or [VS Code](https://code.visualstudio.com)

---

## Prerequisites

You should be comfortable writing simple Python programs. If you're new to Python, take [Python for beginners](/learn/python/) first.

---

## How the course works

Each module explains a topic and shows the SQL and Python you need, with examples you can run yourself. Most modules end with a quick quiz, and your progress is saved in your browser and shown in the sidebar.

Let's dive in!

---
