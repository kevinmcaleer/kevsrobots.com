---
layout: lesson
title: Python Integration
author: Kevin McAleer
type: page
cover: /learn/duckdb/assets/cover.jpg
date: 2025-09-04
previous: 03_advanced_features.html
next: 05_data_analysis.html
description: Use DuckDB from Python to query DataFrames, return results as DataFrames,
  and power notebooks.
percent: 49
duration: 5
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


DuckDB integrates tightly with Pandas and Polars. You can query DataFrames directly using SQL and get results back as DataFrames.

> Seeing new terms (DataFrame registration, persistence, Parquet)? See the [Beginner glossary](09_conclusion#beginner-glossary-the-language-of-data).

---

## Mental model (2 minutes)
- DuckDB is a fast SQL engine you embed in your Python process.
- A "connection" is your gateway to run multiple statements and keep state (tables, views).
- DataFrames (Pandas/Polars) can be treated like temporary tables in SQL.
- Results can come back as DataFrames, so you can plot/model immediately.

Why this matters:
- You keep the best of both worlds: Python’s ecosystem and SQL’s expressive queries.
- No servers to manage; it’s reproducible and fast on your laptop or CI.

---

## Pattern A: Quick one‑off query on a DataFrame

Useful for small scripts and notebooks when you don’t need persistence.

```python
import duckdb, pandas as pd

# Load a DataFrame
url = 'https://raw.githubusercontent.com/mwaskom/seaborn-data/master/tips.csv'
df = pd.read_csv(url)

# Query a DataFrame directly (DuckDB auto-registers the variable name df)
res = duckdb.query("""
  SELECT day, time, ROUND(AVG(total_bill), 2) AS avg_bill
  FROM df
  GROUP BY day, time
  ORDER BY avg_bill DESC
""").df()

print(res.head())
```

> You try it (2–3 min)
> - Select only `day, time, tip` and compute `tip_pct` in SQL
> - Filter to `time = 'Dinner'` and sort by `tip_pct` desc

Notes:
- `duckdb.query(..)` can see variables in the current scope (like `df`).
- `.df()` converts the result to a Pandas DataFrame.
- Good for quick exploration; use a connection if you want to create tables.

---

## Pattern B: Connection for persistence and multiple statements

Use a connection when you want to create tables, run several queries, or reuse state across cells.

```python
import duckdb
con = duckdb.connect("analytics.duckdb")  # creates the file if missing

# Create or reuse a table from a remote CSV
con.execute("""
  CREATE TABLE IF NOT EXISTS tips AS
  SELECT * FROM read_csv_auto('https://raw.githubusercontent.com/mwaskom/seaborn-data/master/tips.csv')
""")

# Run more queries using the same state
out = con.execute("SELECT day, COUNT(*) AS orders FROM tips GROUP BY day ORDER BY orders DESC").df()
print(out)

con.close()
```

> You try it (2–3 min)
> - Add a view `v_tips_enriched` with `tip_pct`
> - Create a table `tip_summary(day, time, revenue, orders)` from that view

Notes:
- A single `.duckdb` file makes your analysis portable and versionable.
- Use `.execute()` to run SQL; call `.df()` to get results into Pandas.

---

## Pattern C: Register DataFrames explicitly

When you have an in-memory DataFrame and want to join it with tables.

```python
import duckdb, pandas as pd

local_df = pd.DataFrame({
    "day": ["Thur", "Fri", "Sat", "Sun"],
    "is_weekend": [False, False, True, True]
})

con = duckdb.connect("analytics.duckdb")
con.register("days", local_df)  # now usable as table `days`

res = con.execute("""
  SELECT t.day, d.is_weekend, COUNT(*) AS orders
  FROM tips t
  JOIN days d ON d.day = t.day
  GROUP BY t.day, d.is_weekend
  ORDER BY orders DESC
""").df()

print(res)
con.close()
```

> You try it (2–3 min)
> - Add `avg_tip_pct` using `tip / NULLIF(total_bill,0) * 100`
> - Sort by `avg_tip_pct` and compare weekend vs weekday

Notes:
- `register` exposes a DataFrame as a temporary table for this connection.
- Handy for small lookup tables or feature lists you build in Python.

---

## Returning results: Pandas or Polars

- `.df()` returns Pandas.
- `.pl()` returns Polars (requires `polars`).

```python
# Optional: Polars
# pip install polars
import duckdb, polars as pl

q = duckdb.query("SELECT 42 AS answer")
res_pl: pl.DataFrame = q.pl()
print(res_pl)
```

---

See also: [Troubleshooting DuckDB](troubleshooting) for common Python/SSL/export issues.


## Parameter binding (avoid string formatting bugs)

Use `?` placeholders and pass values as a list/tuple.

```python
threshold = 20
rows = duckdb.query("SELECT * FROM tips WHERE total_bill > ?", [threshold]).df()
```

> You try it (1–2 min)
> - Bind a `min_tip_pct` variable and filter using `tip / NULLIF(total_bill,0) * 100 > ?`

---

## Notebooks tips
- Keep each query small and readable; give views/tables clear names.
- Persist important intermediate results to tables for speed and reuse.
- Prefer Parquet for large datasets; it’s fast and preserves schema.

---

## Troubleshooting
- "No such file or directory" when exporting: create the folder first (e.g., `exports/`).
- "Module not found" for polars: `pip install polars`.
- Connection locked by another process: close other connections or restart the kernel.

> SSL on macOS (certificate verify failed):
> - Run the Python certificate script once: `bash "/Applications/Python 3.13/Install Certificates.command"`
> - Or in a venv: `pip install certifi` then `export SSL_CERT_FILE="$(python -c 'import certifi; print(certifi.where())')"`
> - Alternative: let DuckDB fetch via httpfs:
>   ```python
>   import duckdb
>   con = duckdb.connect(); con.execute("INSTALL httpfs; LOAD httpfs;")
>   df = con.execute("SELECT * FROM read_csv_auto('https://…/tips.csv')").df()
>   ```

---

## Recap
- Query DataFrames with SQL and get DataFrames back.
- Use a connection when you need persistence and multi-step workflows.
- Mix Python’s libraries with SQL for a productive analytics setup.

---
