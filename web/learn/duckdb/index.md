---
layout: course_overview
title: DuckDB - Fast, free analytics
author: Kevin McAleer
cover: /learn/duckdb/assets/cover.jpg
date: 2025-09-04
description: Learn how to use DuckDB, the fast, free, and embeddable database designed
  for analytics. Query robot sensor logs, parts lists and print histories with SQL,
  from the shell and from Python, then save them as Parquet and tune the speed.
level: beginner
course_duration: 1h 32m
lesson_count: 14
first_lesson: 00_intro.html
quiz_total: 39
outcomes:
- Query CSV and Parquet files with SQL, with no database server
- Summarise, filter and join robot sensor logs, parts lists and print histories
- Save results in a .duckdb file and share them as Parquet
- Run DuckDB from Python with Pandas, Polars and safe query parameters
- Query a folder of partitioned files and show that DuckDB skips the ones it does
  not need
- Time and profile a query, and choose between CSV, Parquet, views and tables
prerequisites:
- Basic comfort with a terminal and with running a Python script
- Helpful but not required, the SQL course and the Python course
you_need:
- A Windows, macOS or Linux computer, or a 64-bit Raspberry Pi 4 or 5
- Python 3.11 or newer, with pip
- A text editor and a terminal
- Optional, for the last lesson's data logger, a Raspberry Pi Pico and Thonny
navigation:
- name: DuckDB - Fast, free analytics
- content:
  - section: Introduction
    content:
    - name: Introduction
      link: 00_intro.html
    - name: Quickstart
      link: quickstart.html
    - name: Troubleshooting DuckDB
      link: troubleshooting.html
  - section: Getting Started with DuckDB
    content:
    - name: Installing DuckDB
      link: 01_installing.html
    - name: Basic Queries
      link: 02_basic_queries.html
    - name: Advanced Features
      link: 03_advanced_features.html
  - section: Integrating DuckDB with Python
    content:
    - name: Python Integration
      link: 04_python_integration.html
    - name: Practical Data Analysis
      link: 05_data_analysis.html
  - section: Datalakes, Parquet, and Performance
    content:
    - name: Data lakes - many files, one query
      link: 06_datalakes.html
    - name: 'Parquet: fast columnar files'
      link: 07_parquet.html
    - name: Performance tuning (practical)
      link: 08_performance_tuning.html
  - section: Conclusion and Next Steps
    content:
    - name: Additional Resources
      link: 10_additional_resources.html
    - name: 'Mini capstone: the workshop report'
      link: 11_mini_capstone.html
    - name: Conclusion and Next Steps
      link: 09_conclusion.html
---
