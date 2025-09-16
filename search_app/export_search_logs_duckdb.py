import re
import sqlite3
from pathlib import Path
import duckdb
import tempfile
import shutil
import sys

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
    try:
        try:
            con.execute("INSTALL sqlite; LOAD sqlite;")
            con.execute(f"CREATE OR REPLACE TABLE logs AS SELECT line_no, TRY_CAST(timestamp AS TIMESTAMP) AS ts, ip, query FROM sqlite_scan('{SQLITE_DB}', 'search_logs')")
        except Exception as e:
            # Fallback: pull via sqlite3 and create a values clause
            sq = sqlite3.connect(SQLITE_DB)
            rows = sq.execute("SELECT line_no, timestamp, ip, query FROM search_logs").fetchall()
            sq.close()
            if not rows:
                print("No rows fetched from SQLite fallback", file=sys.stderr)
                return
            # Create a DuckDB relation from Python list of tuples
            con.register('py_rows', rows)
            # However register doesn't auto-rel, so build using VALUES
            values_clause = ",".join(
                [
                    f"({r[0]}, '{r[1].replace("'","''")}', '{r[2].replace("'","''")}', '{r[3].replace("'","''")}')" for r in rows
                ]
            )
            con.execute("CREATE OR REPLACE TABLE logs AS SELECT * FROM (VALUES " + values_clause + ") AS t(line_no, timestamp, ip, query)")
            con.execute("CREATE OR REPLACE TABLE logs AS SELECT line_no, TRY_CAST(timestamp AS TIMESTAMP) AS ts, ip, query FROM logs")
        try:
            con.execute(f"COPY logs TO '{PARQUET_FILE}' (FORMAT PARQUET);")
        except Exception as e:
            if "File locks are not supported" in str(e):
                tmp_dir = Path(tempfile.gettempdir())
                tmp_file = tmp_dir / PARQUET_FILE.name
                con.execute(f"COPY logs TO '{tmp_file}' (FORMAT PARQUET);")
                PARQUET_FILE.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(tmp_file, PARQUET_FILE)
                print(f"DuckDB wrote to temp {tmp_file}; copied to {PARQUET_FILE}")
            else:
                raise
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
