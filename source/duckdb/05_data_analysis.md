---
title: Practical Data Analysis
description: "Do an end-to-end mini analysis with DuckDB over CSV/Parquet, producing tidy outputs for reuse."
type: page
layout: lesson
---

## Practical Data Analysis

Let’s run a compact analysis workflow you can adapt to your own data. We’ll ingest CSV, transform to tidy tables, and export Parquet.

> New to terms like view, table, materialize, or Parquet? See the [Beginner glossary](09_conclusion#beginner-glossary-the-language-of-data).

> Note: ensure an `exports/` folder exists in your project (or change the path) before running `COPY` commands below.

---

## Mental model (2 minutes)
- Treat your project folder like a tiny data workspace.
- Raw files (CSV/Parquet) come in, queries transform them, tidy outputs come out.
- Use views for readability, then persist final results as tables/Parquet for reuse.

Why this matters:

- **Reproducible**: you (and others) can rerun the same steps and get the same results.
- **Performant**: DuckDB is fast on local files; Parquet keeps things snappy.
- **Composable**: each step is small and easy to reason about.

---

## Ingest data

Goal: get the source data into a table for stable, fast queries.

```sql
.open analytics.duckdb

CREATE OR REPLACE TABLE tips AS
SELECT * FROM read_csv_auto('https://raw.githubusercontent.com/mwaskom/seaborn-data/master/tips.csv');
```

Notes:
- `.open` creates/opens a local database file next to your project.
- `read_csv_auto` infers schema automatically; great for quick starts.
- Persisting as a table speeds up repeated queries and avoids re‑downloading.

Quality check:
```sql
SELECT COUNT(*) AS rows, MIN(total_bill), MAX(total_bill) FROM tips;
SELECT * FROM tips LIMIT 5;
```

> You try it
> - Verify row count: `SELECT COUNT(*) FROM tips;`
> - Quick peek: `SELECT * FROM tips LIMIT 5;`

---

## Clean and enrich

Goal: create helpful metrics without changing the raw table.

```sql
CREATE OR REPLACE VIEW v_tips_enriched AS
SELECT *, ROUND(tip / NULLIF(total_bill,0) * 100, 2) AS tip_pct
FROM tips;
```

Notes:
- Views are named queries; they stay up-to-date with the source table.
- `NULLIF(total_bill,0)` prevents divide-by-zero.
- Keep derived metrics (like `tip_pct`) in views for clarity.

Quality check:
```sql
SELECT * FROM v_tips_enriched LIMIT 5;
SELECT AVG(tip_pct) FROM v_tips_enriched;
```

> You try it
> - Round `tip_pct` to 2 decimals as `tip_pct_2dp`
> - Filter out rows where `tip_pct` is NULL

---

## Analysis queries

Goal: answer questions by summarizing and comparing groups.

```sql
-- Day/time revenue and tip behavior
WITH day_time AS (
  SELECT day, time, SUM(total_bill) AS revenue, AVG(tip_pct) AS avg_tip_pct
  FROM v_tips_enriched
  GROUP BY day, time
)
SELECT * FROM day_time ORDER BY revenue DESC;
```

Notes:
- Use `GROUP BY` to make buckets (e.g., by day and time).
- Measures like `SUM(total_bill)` and `AVG(tip_pct)` describe each bucket.
- Sorting helps find the biggest/most interesting segments quickly.

Try: filter to a specific day or time to focus the analysis.

---

## Persist important results

Goal: save results you’ll reuse or share.

```sql
CREATE OR REPLACE TABLE tip_summary AS
SELECT day, time, ROUND(SUM(total_bill),2) AS revenue,
       ROUND(AVG(tip_pct),2) AS avg_tip_pct,
       COUNT(*) AS orders
FROM v_tips_enriched
GROUP BY day, time;
```

Notes:
- Tables are great for handoff to notebooks, dashboards, or exports.
- Keep table names descriptive (what’s inside, not how it was made).

Quality check:
```sql
SELECT * FROM tip_summary ORDER BY revenue DESC LIMIT 5;
```

---

## Export to Parquet

Goal: produce a fast, portable file for downstream tools.

```sql
COPY tip_summary TO 'exports/tip_summary.parquet' (FORMAT 'parquet');
```

Notes:
- Parquet is columnar and compressed; ideal for analytics.
- You can also write partitioned outputs for larger datasets.

---

## Optional: Visualize with Python

```python
import duckdb, pandas as pd

con = duckdb.connect("analytics.duckdb")
df = con.execute("SELECT * FROM tip_summary ORDER BY revenue DESC").df()

ax = df.plot.bar(x='day', y='revenue', title='Revenue by Day')
fig = ax.get_figure()
fig.tight_layout()
fig.savefig('exports/revenue_by_day.png', dpi=150)
```

Tip:
- Prefer small, focused charts that answer one question at a time.

> You try it
> - Change chart type to line or stacked bar
> - Save the figure as `exports/tips_chart.png`

---

## Try it yourself
- Add a `weekend` flag (via a tiny lookup table) and compare `avg_tip_pct` for weekend vs weekday.
- Create a `high_tip` indicator (e.g., `tip_pct >= 18`) and count orders by day.
- Export both results to Parquet and load them in a notebook to visualize.

---

## Recap
- Views for readability, tables for persisted artifacts.
- Export Parquet for fast re-use and downstream tools.
- Keep analyses small, composable, and reproducible.

---
