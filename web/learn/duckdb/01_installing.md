---
layout: lesson
title: Installing DuckDB
author: Kevin McAleer
type: page
cover: /learn/duckdb/assets/cover.jpg
date: 2025-09-04
previous: troubleshooting.html
next: 02_basic_queries.html
description: Install the DuckDB CLI and Python package, verify the setup, and create
  your first local analytics database.
percent: 28
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


## Installing DuckDB

DuckDB runs as a single, embedded binary. Use the CLI for interactive SQL and the Python package for notebooks and scripts.

## Install options

macOS (Homebrew):

```bash
brew install duckdb
```

Python (any OS):

```bash
python -m pip install --upgrade pip
python -m pip install duckdb
```

Windows (CLI):

- Winget: `winget install DuckDB.cli`
- Chocolatey: `choco install duckdb`

Linux (CLI):

- Debian/Ubuntu: `apt install duckdb` (or download binaries from duckdb.org if not available)

## Verify your install

CLI:

```bash
duckdb --version
```

Python:

```python
import duckdb
print(duckdb.__version__)
```

## First steps in the CLI

Start the shell and run a simple query:

```bash
duckdb
```

---

```sql
-- in the duckdb shell
SELECT 1 AS ok;
```

Create a persistent database file and a table:

```sql
-- creates analytics.duckdb in the current folder
.open analytics.duckdb
CREATE TABLE events(id INTEGER, ts TIMESTAMP, type TEXT);
INSERT INTO events VALUES (1, now(), 'start');
SELECT COUNT(*) AS rows FROM events;
```

Exit with `.quit`.

> You try it (2 min)
> - Start the CLI, run `SELECT 42 AS answer;`
> - Create `analytics.duckdb`, then `CREATE TABLE numbers AS SELECT * FROM range(5) AS x;`
> - `SELECT COUNT(*) FROM numbers;` and exit with `.quit`

## First steps in Python

```python
import duckdb

# Connect to an on-disk database (creates file if it doesn't exist)
con = duckdb.connect("analytics.duckdb")
con.execute("CREATE TABLE IF NOT EXISTS numbers AS SELECT * FROM range(10) AS x;")
rows = con.execute("SELECT COUNT(*) FROM numbers").fetchone()[0]
print({"rows": rows})

con.close()
```

> You try it (2 min)
> - Change `range(10)` to `range(7)` and print the new count
> - Create a new table `evens` with only even values from `numbers`

## Tips

- Keep a dedicated project folder with a single `.duckdb` file you can version control.
- Prefer the Python package for notebooks and the CLI for quick ad‑hoc exploration.
- You can use both in the same project; they are fully compatible.

> Troubleshooting? See [Troubleshooting DuckDB](troubleshooting) for common fixes (exports folder, SSL, Python, S3).

---
