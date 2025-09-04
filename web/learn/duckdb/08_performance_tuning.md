---
layout: lesson
title: Performance tuning (practical)
author: Kevin McAleer
type: page
cover: /learn/duckdb/assets/cover.jpg
date: 2025-09-04
previous: 07_parquet.html
next: 09_conclusion.html
description: 'Quick wins to keep queries fast: schema, files, PRAGMAs, stats, EXPLAIN,
  profiling.'
percent: 77
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


This lesson is a checklist of practical steps you can apply to most DuckDB analytics workloads.

> Seeing new terms (threads, memory limit, statistics, EXPLAIN)? See the [Beginner glossary](09_conclusion#beginner-glossary-the-language-of-data).

---

## PRAGMAs: threads and memory

```sql
PRAGMA threads = 8;          -- use more CPU cores
PRAGMA memory_limit = '2GB'; -- cap memory usage
```

> You try it
> - Set threads to the number of cores you have; re-run a heavy aggregation and time it

---

## Create the right objects

- Views for logic reuse; Tables for persistence and faster reuse.
- Materialize expensive subqueries to tables when reused in multiple steps.

```sql
CREATE OR REPLACE VIEW v_orders_clean AS
SELECT * FROM orders WHERE order_status = 'COMPLETE';

CREATE OR REPLACE TABLE orders_daily AS
SELECT order_date, SUM(amount) AS revenue
FROM v_orders_clean
GROUP BY order_date;
```

> You try it
> - Convert one reused view in your project into a materialized table and compare timings

---

## Statistics and ANALYZE

DuckDB can collect column stats to guide the optimizer.

```sql
ANALYZE;            -- whole database
-- or
ANALYZE my_table;   -- a specific table
```

> You try it
> - Run `ANALYZE` after bulk loads; compare query plans before/after with `EXPLAIN`

---

## EXPLAIN basics (read the plan)

```sql
EXPLAIN
SELECT c.category, SUM(o.amount) AS revenue
FROM orders o
JOIN customers c USING (customer_id)
WHERE o.order_date >= DATE '2024-01-01'
GROUP BY c.category;
```

How to read:
- Look for Scan nodes: are they filtered early? (predicate pushdown)
- Join order: small/selective inputs should be on the build side for hash joins.
- Aggregation: ideally after filters and joins, not earlier.

> You try it
> - Add an index-like hint via materialization: create a filtered table `orders_2024` and re-check `EXPLAIN`

---

## Profiling walkthrough (operator timings)

Enable profiling per session, run a query, then inspect timings.

```sql
PRAGMA enable_profiling = json;                 -- or query_tree / query_profile
PRAGMA profiling_output = 'exports/profile.json';

SELECT c.category, SUM(o.amount) AS revenue
FROM orders o
JOIN customers c USING (customer_id)
WHERE o.order_date >= DATE '2024-01-01'
GROUP BY c.category;
```

Then open `exports/profile.json` in your editor. Look for:
- The slowest operator by time (e.g., `Hash Join`, `Aggregate Hash`)
- Bytes read vs returned (column pruning effective?)
- Whether filters reduced rows early (good) or late (optimize)

> You try it
> - Turn profiling off: `PRAGMA enable_profiling = none;`
> - Change the filter to a tighter range and compare operator timings

---

## File/layout tips

- Parquet: prefer bigger files (tens-hundreds MB) over thousands of tiny ones.
- Partition by low-cardinality keys you filter on (year, month, region).
- Avoid wide rows with many rarely-used columns; split into slim tables when helpful.

> You try it
> - Rewrite a CSV dataset to Parquet and compare read time for a simple aggregation

---

## Recap
- Control parallelism and memory with PRAGMAs.
- Use ANALYZE and EXPLAIN to understand and improve plans.
- Profile occasionally to find real bottlenecks; fix the biggest one first.

See also: [Troubleshooting DuckDB](troubleshooting) for quick fixes that impact performance (file formats, partitions, stats).

---
