---
title: Quickstart
description: "One-page guide: install DuckDB, run first queries, export data, use Python."
type: page
layout: lesson
---

## TL;DR
- Install DuckDB (CLI) and Python package.
- Create a table from CSV, aggregate, and export results.
- Optional: enable httpfs for HTTPS/S3 reads.

---

## Install
- macOS CLI: `brew install duckdb`
- Python: `python -m pip install duckdb pandas`

Create a workspace folder and an `exports/` subfolder so COPY/export won’t fail.

---

## 90 seconds in the CLI

```sql
-- Start the shell: duckdb

-- Create a small table from a public CSV
CREATE OR REPLACE TABLE tips AS
SELECT * FROM read_csv_auto('https://raw.githubusercontent.com/mwaskom/seaborn-data/master/tips.csv');

-- Simple analytics
SELECT day, time, ROUND(AVG(total_bill), 2) AS avg_bill, COUNT(*) AS orders
FROM tips
GROUP BY day, time
ORDER BY avg_bill DESC;

-- Export results
COPY (SELECT * FROM tips LIMIT 100) TO 'exports/tips_sample.csv' (HEADER, DELIMITER ',');
COPY (SELECT * FROM tips) TO 'exports/tips.parquet' (FORMAT PARQUET);
```

Exit with `.quit` when done.

---

## 90 seconds in Python

```python
import duckdb, pandas as pd

# Read CSV to a DataFrame
url = 'https://raw.githubusercontent.com/mwaskom/seaborn-data/master/tips.csv'
df = pd.read_csv(url)

# Query the DataFrame with SQL and get a DataFrame back
res = duckdb.query('''
  SELECT day, time,
         ROUND(AVG(total_bill), 2) AS avg_bill,
         ROUND(AVG(tip / NULLIF(total_bill,0) * 100), 2) AS avg_tip_pct,
         COUNT(*) AS orders
  FROM df
  GROUP BY day, time
  ORDER BY avg_bill DESC
''').df()
print(res)

# Persist to a local .duckdb file for reuse
con = duckdb.connect('analytics.duckdb')
con.execute("CREATE TABLE IF NOT EXISTS tips AS SELECT * FROM df")
con.close()
```

---

## Read from HTTPS/S3 (httpfs)

```sql
INSTALL httpfs;
LOAD httpfs;
SELECT COUNT(*) FROM read_parquet('https://duckdb-public-datasets.s3.us-east-1.amazonaws.com/tpch/1/parquet/lineitem/part-00000-*.parquet');
```

Use folder globs (`*`) to read many files at once.

---

## Minimal Parquet export

Create Parquet from a CSV or table in one step. Parquet is faster to read and keeps types.

```sql
-- From an existing table
COPY (SELECT * FROM tips) TO 'exports/tips.parquet' (FORMAT PARQUET);

-- Or, directly from CSV without creating a table
COPY (
  SELECT * FROM read_csv_auto('https://raw.githubusercontent.com/mwaskom/seaborn-data/master/tips.csv')
) TO 'exports/tips.parquet' (FORMAT PARQUET);
```

Notes:
- Ensure the `exports/` folder exists first.
- Use a `.parquet` extension; DuckDB infers Parquet from `FORMAT PARQUET`.

### Python: minimal Parquet export

```python
import duckdb, pandas as pd
df = pd.read_csv('https://raw.githubusercontent.com/mwaskom/seaborn-data/master/tips.csv')
with duckdb.connect() as con:
  con.execute("COPY (SELECT * FROM df) TO 'exports/tips.parquet' (FORMAT PARQUET)")
```

---

## Offline sample data
See `source/duckdb/data/README.md` to create tiny local CSV/Parquet samples (e.g., `tips.csv`, `tips.parquet`).

Then swap the path in examples to point at those local files.

---

## Quick troubleshooting
- Exports fail: create the target folder, e.g., `exports/`.
- SSL on macOS: either run the Python certificate installer or use DuckDB `httpfs` to fetch data.
- Too slow? Try: `PRAGMA threads = 8; PRAGMA memory_limit = '2GB';` and prefer Parquet over CSV.

---
