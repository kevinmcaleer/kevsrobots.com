---
layout: lesson
title: Troubleshooting DuckDB
author: Kevin McAleer
type: page
cover: /learn/duckdb/assets/cover.jpg
date: 2025-09-04
previous: quickstart.html
next: 01_installing.html
description: Common issues and quick fixes for CLI, Python, httpfs, exports, and performance.
percent: 21
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


## Export and file paths
- Error: `No such file or directory` when using `COPY ... TO 'exports/...'`
  - Fix: Create the folder first (e.g., `exports/`). DuckDB won’t auto-create parent folders.
- Relative paths
  - The path is relative to the current working directory (CLI or script). Use absolute paths if in doubt.

## SSL / HTTPS on macOS
- Error: `certificate verify failed`
  - Option A (system Python): Run the installer once: `bash "/Applications/Python 3.13/Install Certificates.command"`
  - Option B (venv): `pip install certifi` then set `SSL_CERT_FILE=$(python -c 'import certifi; print(certifi.where())')`
  - Option C: Use DuckDB’s `httpfs` so DuckDB fetches the file directly:
    ```sql
    INSTALL httpfs; LOAD httpfs;
    SELECT COUNT(*) FROM read_csv_auto('https://example.com/data.csv');
    ```

## S3 access
- Permission denied or timeouts
  - Ensure correct AWS credentials in environment or config file
  - Region mismatches can cause slowdowns; set `s3_region` if needed

## Schema mismatch when combining files
- Error: column not found or incompatible types
  - Align schemas by selecting matching columns and explicit casts
  - Example:
    ```sql
    SELECT a, b, CAST(NULL AS DOUBLE) AS c FROM read_parquet('old/*.parquet')
    UNION ALL
    SELECT a, b, c FROM read_parquet('new/*.parquet');
    ```

## Performance quick wins
- Prefer Parquet over CSV for repeated reads
- Filter early, select only needed columns
- Partition on low-cardinality keys you filter by
- Set PRAGMAs for parallelism/memory:
  ```sql
  PRAGMA threads = 8; PRAGMA memory_limit = '2GB';
  ```
- Collect statistics after bulk loads:
  ```sql
  ANALYZE;
  ```

## Python integration
- `ModuleNotFoundError: polars` → `pip install polars`
- Connection locked by another process → close other connections/restart kernel
- Use parameter binding to avoid string formatting bugs:
  ```python
  duckdb.query("SELECT * FROM tips WHERE total_bill > ?", [20]).df()
  ```

## Notebook hiccups
- If you change queries a lot, restart the kernel to clear state
- Materialize expensive views into tables if cells depend on them often

## Where to get help
- DuckDB docs: https://duckdb.org/docs/
- GitHub issues: https://github.com/duckdb/duckdb
- Course Quickstart: quick setup and common patterns
