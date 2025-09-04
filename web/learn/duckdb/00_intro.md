---
layout: lesson
title: Introduction
author: Kevin McAleer
type: page
cover: assets/cover.jpg
date: 2025-09-04
next: quickstart.html
description: Start here to learn what DuckDB is, why it shines for analytics, and
  how this course is structured.
percent: 7
duration: 3
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
    - name: Querying data lakes (HTTP/S3)
      link: 06_datalakes.html
    - name: 'Parquet: fast columnar files'
      link: 07_parquet.html
    - name: Performance tuning (practical)
      link: 08_performance_tuning.html
  - section: Conclusion and Next Steps
    content:
    - name: Conclusion and Next Steps
      link: 09_conclusion.html
    - name: Additional Resources
      link: 10_additional_resources.html
    - name: 'Mini capstone: local analytics project'
      link: 11_mini_capstone.html
---


![]({{page.cover}}){:class="cover"}

## DuckDB for Analytics — Introduction

DuckDB is an in-process, [columnar](/resources/glossary.html#columnar) [SQL](/resources/glossary.html#sql) database that runs anywhere you can run your app. It excels at fast analytical queries on local files like [Parquet](/resources/glossary.html#parquet) and CSV, without needing a server, a cluster, or cloud credits.

> New to terms like Parquet, CTE, or materialize? See the [Beginner glossary](09_conclusion#beginner-glossary-the-language-of-data).

This course is part of the [Databases learning pathway](/learn/learning_pathways/databases.html), focused on practical analytics: querying files, creating small analytical datasets, and integrating with Python.

## What you’ll learn
- Install and run DuckDB (CLI and Python)
- Query CSV/Parquet directly with SQL
- Create and persist a local analytics database
- Work with DuckDB from Python and Pandas/Polars
- Read data from data lakes (local, HTTP/S3) and tune performance
- Use extensions and best practices for analytical workloads

## Who this is for
- Makers, analysts, and developers who want fast local analytics
- Beginners to databases and data lakes who prefer practical examples

## Prerequisites
- Basic command line and Python familiarity
- macOS, Linux, or Windows
- Optional: Python 3.10+ and VS Code for notebook-style work

## Why DuckDB
- No server: embed it in your scripts and apps
- Fast analytical SQL: vectorized, columnar execution
- File-first: query Parquet/CSV directly, no ETL required
- Great with Python: query DataFrames and return results as DataFrames
- Portable: a single .duckdb file you can version and ship

When to choose it:
- You need quick, local analytics (OLAP) on files
- You want to prototype or build repeatable data workflows without infra
- You want SQL over DataFrames, or to complement Pandas/Polars

## Quick start (macOS)
- CLI: `brew install duckdb`
- Python: `python -m pip install duckdb`

Try the CLI:

```bash
duckdb
```

Then run a tiny analytics query directly on a CSV from the web:

```sql
-- Create a table from a remote CSV
CREATE TABLE tips AS
SELECT * FROM read_csv_auto('https://raw.githubusercontent.com/mwaskom/seaborn-data/master/tips.csv');

-- Simple aggregate
SELECT day, ROUND(SUM(total_bill), 2) AS revenue, COUNT(*) AS orders
FROM tips
GROUP BY day
ORDER BY revenue DESC;
```

Exit with `.quit`.

Or from Python:

```python
import duckdb, pandas as pd

df = pd.read_csv('https://raw.githubusercontent.com/mwaskom/seaborn-data/master/tips.csv')

# Query a DataFrame directly
result = duckdb.query("""
  SELECT day, AVG(total_bill) AS avg_bill
  FROM df
  GROUP BY day
  ORDER BY avg_bill DESC
""").df()

print(result)
```

## What we’ll build in this course
- A small, local analytics workspace using DuckDB
- Reproducible queries over CSV/Parquet (local and remote)
- A persisted .duckdb database and lightweight “data mart”
- Python integrations for analysis and notebooks (like Jupyter notebooks)
- Practical performance tips (statistics, Parquet predicate pushdown, and more)

## Course structure
- **00 intro**: *what DuckDB is and why use it*
- **01 installing**: *CLI, Python, VS Code setup*
- **02 basic queries**: *selecting, filtering, aggregations, joins*
- **03 advanced features**: *views, CTEs, extensions, persistence*
- **04 python integration**: *DataFrames, results, notebooks*
- **05 data analysis**: *end-to-end mini analysis*
- **06 data lakes**: *reading from local/remote object storage*
- **07 parquet**: *formats, partitioning, predicates*
- **08 performance tuning**: *tips for speed and memory*
- **09 conclusion**: *recap and patterns*
- **10 additional resources**: *docs, tools, datasets*

## Next up
If you want a one-page setup and first queries, see the [Quickstart](quickstart). Otherwise, head to [01 installing](01_installing) to set up DuckDB and the Python environment.

---
