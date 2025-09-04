---
title: Advanced Features
description: Use CTEs, views, persistence, COPY, and extensions to build a tidy local analytics workflow.
type: page
layout: lesson
---

## Advanced Features

Level up your workflow with CTEs, views, persistence, COPY, and extensions.

> Unfamiliar terms like CTE, view, persistence, or PRAGMA? See the [Beginner glossary](09_conclusion#beginner-glossary-the-language-of-data).

## CTEs (WITH clauses)

CTE stands for Common Table Expression. Think of a CTE as a short‑lived, named subquery you define at the top of your SQL with the `WITH` keyword. It helps you break a complex query into readable steps.

Why it matters:
- **Improves readability**: name meaningful steps instead of nesting subqueries.
- **Reuse**: reference the same CTE multiple times in a single query.
- **Debuggability**: test each step independently by running it as a normal SELECT.

---

Step by step:

1. Define a CTE with `WITH name AS ( …select… )`.
2. Use that name in the main query as if it were a temporary table.
3. Chain multiple CTEs by separating them with commas.

---

Example 1 — Single CTE to add a metric and summarize:

```sql
WITH enriched AS (
  SELECT *, ROUND(tip / NULLIF(total_bill,0) * 100, 2) AS tip_pct
  FROM tips
)
SELECT day, ROUND(AVG(tip_pct), 2) AS avg_tip_pct
FROM enriched
GROUP BY day
ORDER BY avg_tip_pct DESC;
```

Notes:
- `NULLIF(total_bill,0)` avoids divide‑by‑zero.
- The CTE `enriched` exists only for this query; it’s not persisted.

> You try it (5 min)
> - Add a `party_size_band` CTE that buckets sizes (1–2 small, 3–4 medium, 5+ large)
> - Join it in the final SELECT and summarize by `day, party_size_band`

---

Example 2 — Multiple CTE pipeline (clean → enrich → summarize):

```sql
WITH cleaned AS (
  SELECT * FROM tips WHERE total_bill > 0
),

enriched AS (
  SELECT *, ROUND(tip / total_bill * 100, 2) AS tip_pct
  FROM cleaned
),

summarized AS (
  SELECT day, time,
         ROUND(SUM(total_bill), 2) AS revenue,
         ROUND(AVG(tip_pct), 2) AS avg_tip_pct,
         COUNT(*) AS orders
  FROM enriched
  GROUP BY day, time
)
SELECT * FROM summarized ORDER BY revenue DESC;
```

When to use CTEs vs Views vs Tables:

- **CTEs**: for one‑off, readable query steps; no persistence.
- **Views**: save a query definition you’ll reuse; always reflects latest data.
- **Tables**: persist results for speed, stability, or downstream use.

---

## Views

```sql
CREATE OR REPLACE VIEW v_tip_stats AS
SELECT day, time, ROUND(SUM(total_bill), 2) AS revenue, COUNT(*) AS orders
FROM tips
GROUP BY day, time;

SELECT * FROM v_tip_stats ORDER BY revenue DESC;
```

Views store queries, not data. They always reflect the latest underlying data.

---

## Persistence: database files

```sql
-- In the CLI
.open analytics.duckdb

-- Save results as a physical table
CREATE TABLE IF NOT EXISTS tips AS SELECT * FROM read_csv_auto('https://raw.githubusercontent.com/mwaskom/seaborn-data/master/tips.csv');
```

Now your data is stored in `analytics.duckdb` for fast, repeatable queries.

---

## COPY data in and out

Note: ensure the `exports/` folder exists (or change the path) before running COPY.

```sql
-- Export a query to CSV or Parquet
COPY (
  SELECT day, time, SUM(total_bill) AS revenue
  FROM tips
  GROUP BY day, time
) TO 'exports/tip_revenue.parquet' (FORMAT 'parquet');
```

```sql
-- Import local CSV/Parquet
CREATE TABLE sales AS SELECT * FROM read_parquet('data/sales/*.parquet');
```

> You try it (3–5 min)
> - Export `v_tip_stats` to `exports/tip_stats.parquet`
> - Re-import it as `tip_stats_imported` and compare `COUNT(*)`

---

## Extensions

Some features ship as extensions. Popular ones:

```sql
INSTALL httpfs;  -- http, https, s3
LOAD httpfs;

INSTALL json;
LOAD json;
```

With `httpfs` you can query remote files over HTTP/S3. With `json` you can query JSON via `read_json()`.

---

## Pragmas and settings

```sql
PRAGMA version;
PRAGMA threads;        -- show default threads
SET threads = 4;       -- adjust parallelism
PRAGMA memory_limit='2GB';
```

---

## Recap

- Use CTEs and views for readable analytics pipelines.
- Persist to a `.duckdb` file for speed and portability.
- COPY moves data between DuckDB and files; extensions add superpowers.

---
