---
title: "Mini capstone: local analytics project"
description: "Plan and build a tiny analytics project end-to-end with DuckDB, Parquet, and Python."
type: page
layout: lesson
---

## Goal
Build a small, repeatable analytics workflow on your machine:
- Ingest CSV → curate tables → export Parquet summaries
- Use a `.duckdb` file for persistence
- Optional: a simple chart via Python

## Dataset
Pick one:
- Local: `source/duckdb/data/tips.csv` (create from README if missing)
- Remote: Seaborn tips.csv URL (works with httpfs)

## Steps
1) Create project folders
- `exports/` for outputs
- `analytics.duckdb` as your DB file

2) Ingest
```sql
CREATE OR REPLACE TABLE tips AS
SELECT * FROM read_csv_auto('source/duckdb/data/tips.csv');
```
(Or use the remote URL.)

3) Enrich
```sql
CREATE OR REPLACE VIEW v_tips AS
SELECT *, CASE WHEN total_bill=0 THEN NULL ELSE tip/total_bill END AS tip_pct
FROM tips;
```

4) Aggregate
```sql
CREATE OR REPLACE TABLE tip_daily AS
SELECT day, time,
       ROUND(AVG(tip_pct)*100,2) AS avg_tip_pct,
       SUM(total_bill) AS revenue,
       COUNT(*) AS orders
FROM v_tips
GROUP BY day, time;
```

5) Export
```sql
COPY (SELECT * FROM tip_daily)
  TO 'exports/tip_daily.parquet' (FORMAT PARQUET);
```

6) Optional chart (Python)
```python
import duckdb
con = duckdb.connect()
df = con.execute("SELECT day, time, avg_tip_pct FROM tip_daily").df()
ax = df.pivot(index='day', columns='time', values='avg_tip_pct').plot(kind='bar', figsize=(6,3))
ax.set_ylabel('Avg tip %')
```

## Checkpoints
- Does `exports/tip_daily.parquet` exist and open in DuckDB?
- How many rows? Which `day,time` is top by `revenue`?
- Try partitioned export:
```sql
COPY (SELECT *, strftime(current_timestamp, '%Y') AS year,
             strftime(current_timestamp, '%m') AS month
      FROM tip_daily)
TO 'exports/tip_daily_by_year_month' (FORMAT PARQUET, PARTITION_BY (year, month));
```

## Stretch goals
- Add a small `day_labels(day, weekend)` table and join.
- Profile one query (enable JSON profiling, inspect slowest operator).
- Swap CSV ingest for Parquet ingest and compare query time.

---
