import re
from pathlib import Path

import pandas as pd

LOG_FILE = Path('search-logs.log')
OUTPUT_DIR = Path('exports')
PARQUET_FILE = OUTPUT_DIR / 'search_logs_simple.parquet'

LINE_REGEX = re.compile(r"INFO:root:(?P<ts>[^ ]+) - IP: (?P<ip>[^ ]+) - Query: (?P<query>.*)$")


def parse():
    if not LOG_FILE.exists():
        raise SystemExit(f"Log file not found: {LOG_FILE}")
    with LOG_FILE.open('r', encoding='utf-8', errors='ignore') as f:
        for line_no, line in enumerate(f, start=1):
            line = line.rstrip('\n')
            m = LINE_REGEX.match(line)
            if not m:
                continue
            g = m.groupdict()
            yield {
                'line_no': line_no,
                'timestamp': g['ts'],
                'ip': g['ip'],
                'query': g['query'].strip()
            }


def main():
    rows = list(parse())
    if not rows:
        print('No rows parsed')
        return
    OUTPUT_DIR.mkdir(exist_ok=True, parents=True)
    df = pd.DataFrame(rows)
    try:
        df['timestamp'] = pd.to_datetime(df['timestamp'], errors='coerce')
    except Exception:
        pass
    df.to_parquet(PARQUET_FILE, index=False)
    print(f"Wrote {len(df)} rows -> {PARQUET_FILE}")


if __name__ == '__main__':
    main()
