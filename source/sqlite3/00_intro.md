---
title: Course Overview
description: >-
    What you'll learn
layout: lesson
type: page
cover: /learn/sqlite3/assets/banners/00_intro.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

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
{:class="table table-single"}

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
* Nothing else to buy: the `sqlite3` module is part of Python, and the bonus tools lesson suggests free desktop apps for looking inside your databases

---

## Prerequisites

You should be comfortable writing simple Python programs. If you're new to Python, take [Python for beginners](/learn/python/) first. When you finish, [Getting Started with SQL](/learn/sql/) goes deeper into the SQL language itself.

> **Teachers**: This course suits learners aged about 13 and up who can already write short Python programs (variables, loops, functions). Allow 30 to 45 minutes per lesson; the ten core lessons fit about six sessions, and the bonus normal-form lessons are a good stand-alone session on design. Each learner needs a computer with Python 3 and nothing else. There is no hardware. The demo program lesson has a run box that works in the browser, and the normal-form lessons are pencil-and-paper design, so a class with no installs can still do those. Have learners use made-up data, and each keep their own `.db` file on a local disk.

---

## How the course works

Each module explains a topic and shows the SQL and Python you need, with examples you can run yourself. Most modules end with a quick quiz, and your progress is saved in your browser and shown in the sidebar.

Let's dive in!

---
