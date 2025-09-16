import re
import sqlite3
from pathlib import Path
import duckdb

LOG_FILE = Path('search-logs.log')
SQLITE_DB = Path('exports/search_logs.db')
PARQUET_FILE = Path('exports/search_logs_duckdb.parquet')

LINE_REGEX = re.compile(r"INFO:root:(?P<ts>[^ ]+) - IP: (?P<ip>[^ ]+) - Query: (?P<query>.*)$")


def ensure_exports_dir():
    SQLITE_DB.parent.mkdir(parents=True, exist_ok=True)


def parse_log():
    if not LOG_FILE.exists():
        raise SystemExit(f"Log file not found: {LOG_FILE}")
    with LOG_FILE.open('r', encoding='utf-8', errors='ignore') as f:
        for line_no, line in enumerate(f, start=1):
            line = line.rstrip('\n')
            m = LINE_REGEX.match(line)
            if not m:
                continue
            g = m.groupdict()
            yield (line_no, g['ts'], g['ip'], g['query'].strip())


def write_sqlite(rows):
    if SQLITE_DB.exists():
        SQLITE_DB.unlink()
    con = sqlite3.connect(SQLITE_DB)
    cur = con.cursor()
    cur.execute(
        """
        CREATE TABLE search_logs (
            line_no INTEGER PRIMARY KEY,
            timestamp TEXT,
            ip TEXT,
            query TEXT
        )
        """
    )
    cur.executemany(
        "INSERT INTO search_logs (line_no, timestamp, ip, query) VALUES (?, ?, ?, ?)",
        rows
    )
    con.commit()
    count = cur.execute("SELECT COUNT(*) FROM search_logs").fetchone()[0]
    con.close()
    return count


def export_with_duckdb():
    con = duckdb.connect()
    # Attach the sqlite db via virtual table using the sqlite extension if available
    # Simpler: load using read_parquet but we have sqlite, so use sqlite_scan if compiled.
    try:
        con.execute("INSTALL sqlite; LOAD sqlite;")
        df = con.execute(f"SELECT * FROM sqlite_scan('{SQLITE_DB}', 'search_logs')").df()
    except Exception:
        # Fallback: use python sqlite to fetch and register as relation
        sq = sqlite3.connect(SQLITE_DB)
        rows = sq.execute("SELECT * FROM search_logs").fetchall()
        df = duckdb.from_df_rows(rows, ['line_no', 'timestamp', 'ip', 'query']).df()
        sq.close()
    # Cast timestamp to TIMESTAMP if possible
    try:
        con.register('logs_df', df)
        con.execute("CREATE OR REPLACE TABLE logs AS SELECT line_no, TRY_CAST(timestamp AS TIMESTAMP) AS ts, ip, query FROM logs_df")
        con.execute(f"COPY logs TO '{PARQUET_FILE}' (FORMAT PARQUET);")
    finally:
        con.close()


def main():
    ensure_exports_dir()
    rows = list(parse_log())
    if not rows:
        print("No rows parsed from log; exiting")
        return
    count = write_sqlite(rows)
    print(f"Wrote {count} rows into {SQLITE_DB}")
    export_with_duckdb()
    print(f"Parquet written to {PARQUET_FILE}")


if __name__ == '__main__':
    main()
