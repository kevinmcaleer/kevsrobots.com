This folder is reserved for tiny sample datasets used by the DuckDB course.

We keep files small so they load fast and work offline. Prefer Parquet where possible.

Suggested structure:
- tips.csv — small CSV used in lessons 04–05
- tips.parquet — same data in Parquet for lesson 07
- sales/ — optional partitioned example like sales/year=2024/month=06/data.parquet

You can create these locally using DuckDB:

```sql
CREATE OR REPLACE TABLE tips AS
SELECT * FROM read_csv_auto('https://raw.githubusercontent.com/mwaskom/seaborn-data/master/tips.csv');

COPY (SELECT * FROM tips) TO 'source/duckdb/data/tips.csv' (HEADER, DELIMITER ',');
COPY (SELECT * FROM tips) TO 'source/duckdb/data/tips.parquet' (FORMAT PARQUET);

-- Optional partitioned sample
CREATE OR REPLACE TABLE sales AS
SELECT 2024 AS year, 6 AS month, * FROM tips LIMIT 50;
COPY (SELECT * FROM sales) TO 'source/duckdb/data/sales' (FORMAT PARQUET, PARTITION_BY (year, month));
```

If you use the above, adjust lesson queries to point at `source/duckdb/data/…` when offline.
