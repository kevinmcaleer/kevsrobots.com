---
layout: course_overview
title: Getting Started with SQL
author: Kevin McAleer
cover: /learn/sql/assets/cover.jpg
date: 2024-11-01
description: Learn the fundamentals of SQL, relational database theory, and essential
  skills for database management, by building a Robot Workshop database in SQLite.
level: beginner
course_duration: 1h 37m
lesson_count: 16
first_lesson: 00_intro.html
quiz_total: 46
outcomes:
- Explain what tables, rows, primary keys and foreign keys are, and why data is split
  across linked tables
- Create a SQLite database and tables with sensible data types and constraints
- Insert, update and delete rows safely, and undo mistakes with transactions
- Filter, sort and combine data with SELECT, WHERE, CASE and JOIN
- Summarise data with GROUP BY and aggregate functions, and answer questions with
  subqueries
- Design tables with normal forms, and speed up queries with views and indexes
prerequisites:
- None. Being comfortable typing commands in a terminal helps, for example Introduction
  to the Linux Command Line
you_need:
- A computer running Windows, macOS or Linux, or a Raspberry Pi
- The sqlite3 command-line tool, or Python 3 (which includes SQLite)
- A text editor for saving your SQL files
navigation:
- name: Getting Started with SQL
- content:
  - section: Introduction
    content:
    - name: Course Overview
      link: 00_intro.html
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
