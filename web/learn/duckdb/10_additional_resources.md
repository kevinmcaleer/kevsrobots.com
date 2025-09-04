---
layout: lesson
title: Additional Resources
author: Kevin McAleer
type: page
cover: /learn/duckdb/assets/cover.jpg
date: 2025-09-04
previous: 09_conclusion.html
next: 11_mini_capstone.html
description: Handy links, datasets, and tools to go deeper with DuckDB and analytics.
percent: 91
duration: 2
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


## Additional Resources

Curated references to continue your DuckDB journey.

## Official

- DuckDB Documentation: https://duckdb.org/docs/
- SQL Functions and Operators: https://duckdb.org/docs/sql/functions
- Python API: https://duckdb.org/docs/api/python
- Extensions: https://duckdb.org/docs/extensions/overview

---

## Tutorials and datasets

- Seaborn sample datasets (small, friendly): https://github.com/mwaskom/seaborn-data
- FiveThirtyEight datasets (story-focused): https://github.com/fivethirtyeight/data
- UCI Machine Learning Repository (varied sizes): https://archive.ics.uci.edu/
- data.gov.uk, data.gov (public sector): curated open datasets by country
- NYC TLC trips (large): https://www.nyc.gov/site/tlc/about/tlc-trip-record-data.page
- Awesome DuckDB list: https://github.com/duckdb/awesome-duckdb

### Starter query ideas
- Tidy a CSV into typed columns, export to Parquet
- Compute daily or monthly aggregates with `DATE_TRUNC`
- Compare categories with `GROUP BY` and `ORDER BY`
- Add a small dimension table (e.g., labels) and `JOIN`
- Try partitioned export and query one slice

---

## Tools that pair well

- Parquet command line: `parquet-tools`
- Pandas or Polars for DataFrames
- VS Code + Jupyter for notebooks

---

## Example repo structure

```
project/
  data/         # raw inputs (csv/parquet)
  exports/      # query outputs (parquet/png)
  analytics.duckdb
  queries/      # sql scripts
  notebooks/    # analysis notebooks
  README.md
```

---

## Troubleshooting

- If remote reads fail, ensure `httpfs` is installed and loaded.
- For large scans, add predicates and select fewer columns.
- If memory errors occur, lower data size or set `PRAGMA memory_limit`.

Keep experimenting—small, composable queries add up to powerful analytics.

---
