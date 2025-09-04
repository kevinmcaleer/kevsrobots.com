---
title: Querying data lakes (HTTP/S3)
description: "Read CSV/Parquet from cloud/object storage with DuckDB’s httpfs, and use partition pruning."
type: page
layout: lesson
---

DuckDB can read remote files over HTTPS and S3 without running a server.

> Seeing new terms (predicate pushdown, partition pruning, manifest)? See the [Beginner glossary](09_conclusion#beginner-glossary-the-language-of-data).

---

## Enable httpfs

```sql
INSTALL httpfs;
LOAD httpfs;
```

> You try it
> - Run the commands above once per database; verify with `SELECT 1;`

---

## Read remote CSV

```sql
SELECT COUNT(*)
FROM read_csv_auto('https://raw.githubusercontent.com/mwaskom/seaborn-data/master/tips.csv');
```

> You try it
> - Preview first rows with `LIMIT 5`

---

## Read remote Parquet

```sql
SELECT COUNT(*)
FROM read_parquet('https://duckdb-public-datasets.s3.us-east-1.amazonaws.com/tpch/1/parquet/lineitem/part-00000-*.parquet');
```

> You try it
> - Add a filter `WHERE l_shipdate >= '1993-01-01'`

---

## Partitioned folders and pruning

If your Parquet lives in folders like `…/year=2021/month=01/…`, DuckDB can prune folders based on filters.

```sql
SELECT COUNT(*)
FROM read_parquet('s3://my-bucket/sales/year=*/month=*/data.parquet')
WHERE year = 2022 AND month IN (1,2,3);
```

> You try it
> - Replace `s3://my-bucket/...` with your own layout (or a local `sales/year=.../month=...` test)
> - Time the query with and without the `WHERE` to see fewer files scanned

---

## CREATE VIEW over lake data

```sql
CREATE OR REPLACE VIEW v_sales AS
SELECT * FROM read_parquet('s3://my-bucket/sales/year=*/month=*/data.parquet');
```

> You try it
> - Query `v_sales` with a date filter and check that it’s fast

---

## Manifests for stable snapshots

```sql
CALL parquet_metadata('s3://my-bucket/sales/year=*/month=*/data.parquet', 'exports/sales_manifest.json');
SELECT * FROM read_parquet('exports/sales_manifest.json');
```

> You try it
> - Regenerate the manifest after adding data and observe row counts change

---

## Troubleshooting
- Permission denied on S3: configure credentials in environment or DuckDB settings.
- Slow scans: ensure filters match partition columns (`year`, `month`), not computed expressions.
- SSL issues on macOS: see lesson 04 troubleshooting block.

See also: [Troubleshooting DuckDB](troubleshooting) for SSL, S3, and manifest tips.

---

## Recap
- `httpfs` lets DuckDB read HTTPS/S3.
- Partitioned folders + pruning keep queries fast and cheap.
- Manifests provide reproducible snapshots over lake data.

---
