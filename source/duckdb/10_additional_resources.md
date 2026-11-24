---
title: Additional Resources
description: Log real sensor data from a Raspberry Pi Pico, then find the docs, datasets and tools to take your DuckDB analytics further.
layout: lesson
type: page
cover: /learn/duckdb/assets/banners/10_additional_resources.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to log real sensor readings from a Raspberry Pi Pico to a CSV file, query them with DuckDB, and know where to look when you get stuck.

Everything so far used data we generated. The best data is your own, so let's grab some from real hardware and then collect the places to learn more.

## Log your own data from a Pico

A Raspberry Pi Pico has a temperature sensor built into its chip. This MicroPython program (run it on the Pico, from Thonny) writes ten readings to a CSV file stored on the board:

```python
import time
from machine import ADC

sensor = ADC(4)  # channel 4 is the Pico's internal temperature sensor

with open("log.csv", "w") as f:
    f.write("seconds,temp_c\n")
    for _ in range(10):
        volts = sensor.read_u16() * 3.3 / 65535
        temp_c = 27 - (volts - 0.706) / 0.001721  # formula from the RP2040 datasheet
        f.write(f"{time.ticks_ms() // 1000},{temp_c:.2f}\n")
        time.sleep(1)

print("Wrote log.csv")
```

Notes on this program:

- `read_u16()` returns a number from 0 to 65535, which the program scales to volts.
- The Pico has no battery-backed clock, so we log **seconds since the program started** rather than a date and time. If you set the time with Thonny or `ntptime`, you could log a real timestamp instead.
- The internal sensor reads the chip's own temperature, which runs a little warm, so treat the numbers as a demo rather than room temperature.

Copy `log.csv` from the Pico to your computer (Thonny's Files view shows both the Pico and your computer, and lets you copy files between them), then query it:

```sql
SELECT COUNT(*) AS readings, ROUND(AVG(temp_c), 2) AS avg_temp, MAX(seconds) AS seconds_logged
FROM 'log.csv';
```

With ten readings the first column is `10`, the second is the average of your temperatures, and `seconds_logged` is the Pico's uptime in seconds at the last reading. For a longer log, raise the `range(10)` count and add `time.sleep(60)` for one reading a minute. Your own logs can use exactly the queries from lessons 2 to 5.

```mermaid
flowchart LR
    A[Pico sensor] -->|MicroPython| B[log.csv on the board]
    B -->|Thonny copy| C[log.csv on your computer]
    C -->|DuckDB| D[Answers and charts]
```

## Official documentation

- DuckDB documentation: [duckdb.org/docs/stable](https://duckdb.org/docs/stable/)
- Concurrency (why only one program can write a database file): [duckdb.org/docs/stable/connect/concurrency](https://duckdb.org/docs/stable/connect/concurrency)
- SQL functions, Python client and extension pages are all linked from the documentation home page
- Release downloads for the shell: [github.com/duckdb/duckdb/releases](https://github.com/duckdb/duckdb/releases)
- A curated list of tools and articles: [github.com/duckdb/awesome-duckdb](https://github.com/duckdb/awesome-duckdb)

## Datasets to practise on

- [data.gov.uk](https://www.data.gov.uk/) and similar national open data sites offer thousands of free CSV files.
- The [NYC taxi trip records](https://www.nyc.gov/site/tlc/about/tlc-trip-record-data.page) are famous large Parquet files, handy for the performance lesson.
- Your own logs beat any download: a Pico, a Raspberry Pi weather station, a 3D printer's job history, your parts spreadsheet exported as CSV.

## Tools that pair well

- Pandas and Polars for DataFrames, and matplotlib for charts
- VS Code or Jupyter for notebook-style exploration
- Thonny for talking to your Pico

## Starter ideas for your next query

- Tidy a CSV into typed columns and export it to Parquet.
- Compute daily and monthly averages with `date_trunc`.
- Compare groups with `GROUP BY` and rank them with `ORDER BY`.
- Join a small lookup table, such as parts costs, onto a log.
- Partition an export by month and query a single slice.

## Try it Yourself

1. Run the Pico logger and query your own `log.csv` with the shell.
2. Change the program to log every five seconds for a minute, then compute the minimum and maximum temperature.
3. Find a CSV file from an open data site, load it with `read_csv`, and ask three questions of it.
4. Challenge: log two sessions to `log1.csv` and `log2.csv` and read both at once with `read_csv('log*.csv')`.

## Common Issues

- **Problem**: `log.csv` is empty, or missing from the Pico.
- **Solution**: Wait for `Wrote log.csv` to print before you stop the program or unplug the board.
- **Why**: The file is only complete once the program has finished and the `with` block has closed it.

- **Problem**: The downloaded CSV has strange columns or the first row is read as data.
- **Solution**: Tell DuckDB what to expect, for example `read_csv('file.csv', header = true, delim = ';')`.
- **Why**: Detecting the layout is a guess, and files with unusual separators or headers can fool it.

- **Problem**: A link in this list no longer works.
- **Solution**: Search the DuckDB documentation home page for the topic name.
- **Why**: Documentation sites reorganise their pages from time to time.

## Check your understanding

{% quiz %}
? Why does the Pico logger write seconds since start instead of a date and time?
- [ ] CSV files cannot store dates
- [x] The Pico has no battery-backed clock, so it does not know the real date after power-up
- [ ] Seconds are more accurate
> The clock only holds the right time if Thonny or `ntptime` set it during that session.

? Which function reads a CSV and lets you set options such as the delimiter?
- [ ] `open_csv`
- [x] `read_csv`
- [ ] `load_file`
> `read_csv('file.csv', delim = ';')` and similar options control how the file is parsed.

? Why are your own logs better practice material than a download?
- [x] You know the real story behind the numbers, so you can tell if an answer makes sense
- [ ] They are always bigger
- [ ] They need no cleaning
> Knowing the context makes it easy to spot wrong results.
{% endquiz %}
