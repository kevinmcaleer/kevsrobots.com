---
layout: lesson
title: Logging Data
author: Kevin McAleer
type: page
cover: /learn/snakie_instruments/assets/banners/08_logging_data.jpg
date: 2026-11-20
previous: 07_instruments_from_blocks.html
next: 09_project_light_scanner.html
description: Record a run with Snakie's Data Logger, tear off a CSV file, write your
  own log file on the board and open it as a table
percent: 81
duration: 5
quiz_total: 30
date_updated: 2026-11-24
navigation:
- name: Snakie Instruments - Oscilloscope, Multimeter and Plotter
- content:
  - section: Overview
    content:
    - name: Snakie Instruments - Oscilloscope, Multimeter and Plotter
      link: 00_intro.html
    - name: The Instrument Dock
      link: 01_the_instrument_dock.html
    - name: Telemetry and LIVE
      link: 02_telemetry_and_live.html
  - section: The Instruments
    content:
    - name: The Plotter
      link: 03_the_plotter.html
    - name: The Multimeter
      link: 04_the_multimeter.html
    - name: The Oscilloscope
      link: 05_the_oscilloscope.html
    - name: Watch and Update
      link: 06_watch_and_update.html
  - section: Blocks and Logging
    content:
    - name: Instruments from Blocks
      link: 07_instruments_from_blocks.html
    - name: Logging Data
      link: 08_logging_data.html
  - section: Project
    content:
    - name: Project - Light Scanner
      link: 09_project_light_scanner.html
    - name: Summary and What's Next
      link: 10_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to record a run with the Data Logger, save it as a CSV file and write your own log file on the board.

Instruments show you *now*. Sometimes you need *then*: a record you can study after the run, compare with yesterday's, or put in a report.

---

## The Data Logger

Open the **Data Logger** from the dock. It looks like a little dot-matrix printer, and it behaves like one.

Run the two-thermometer program from lesson 3 again, then press **● REC** on the logger.

![The Data Logger recording a run](/learn/snakie_instruments/assets/logger_rec.jpg){:class="img-fluid w-100"}

Paper feeds out of the printer. Along the top is a strip chart, and under it one row per moment with every stream's value: `outside=15.9  inside=19.0` and so on.

The logger records **everything** your program is sending, whichever instrument it was meant for: plotted values, meter readings and scope channels all land on the same paper. You don't change your code at all.

| Button | What it does |
|---|---|
| **● REC** | Start recording |
| **PAUSE** | Hold the recording without losing it |
| **✂ TEAR OFF** | Save what's on the paper as a CSV file |
{:class="table table-single"}

## Tear off a CSV

Press **TEAR OFF** and Snakie saves a file with a name like `snakie-log-2026-11-20.csv`.

CSV means comma-separated values. It's the plainest kind of spreadsheet: one row per line, commas between the columns. The first column is `time_s`, the seconds since recording began, followed by a column per stream, such as `plot:inside` and `plot:outside`.

Open it in any spreadsheet program and you can chart it, average it, or compare two runs side by side.

## Logging on the board itself

The Data Logger needs Snakie to be listening. A robot roaming the garden on batteries has nobody listening. For that, the program writes its own file on the board.

```python
log = open("scan.csv", "w")
log.write("angle,volts\n")

for angle in range(0, 181, 5):
    volts = 1.5                      # your reading goes here
    log.write("%d,%.3f\n" % (angle, volts))

log.close()
```

Line by line:

- `open("scan.csv", "w")` creates the file for **w**riting. If it already exists, it's replaced
- The first `write` is the header row: the column names
- `"%d,%.3f\n"` is a template. `%d` is a whole number, `%.3f` is a number with three decimal places, and `\n` ends the line
- `log.close()` finishes the file. **Don't skip it.** Until the file is closed, some of your data may not be saved

You'll run this for real in the project in the next lesson.

## Reading the file back

When the program finishes, the file appears under **DEVICE FILES** on the left of the Code workspace.

![scan.csv in the Device Files panel](/learn/snakie_instruments/assets/device_files.jpg){:class="img-fluid w-100"}

Double-click it. Snakie doesn't show you raw text. It opens a **Data View**: a proper table.

![A CSV file opened as a table](/learn/snakie_instruments/assets/csv_view.jpg){:class="img-fluid w-100"}

The top line tells you the size (*37 rows · 2 cols*). Each column header shows its type and, for numbers, the lowest and highest value and the mean. **Filter** narrows the rows, and **Columns** hides the ones you don't need.

Those column summaries answer a lot of questions before you've opened a spreadsheet. What was the brightest reading? Look at the top of the `volts` column.

## Which kind of log?

| | Data Logger | A file on the board |
|---|---|---|
| Code needed | None | A few lines |
| Works without a computer attached | No | Yes |
| What's recorded | Every telemetry stream | Exactly what you write |
| Where it's saved | Your computer | The board |
{:class="table table-single"}

> **Teachers**: The Data Logger saves its CSV file to the computer you are using, so on shared machines ask learners to move or rename their files before they log off. A file written on the board stays on the board, so the next group using that board will find it under Device Files. Name files clearly, or delete old ones, when boards are shared.

---

## Try it Yourself

1. Record ten seconds of the lesson 3 program and tear off the CSV. How many columns does it have, and why are there more than two?
2. Press **PAUSE** part-way through a run, then carry on. What does the paper show?
3. Change `%.3f` to `%.1f` in the file example. What would the rows look like?

## Common Issues

**Problem**: My CSV has columns I didn't send, like `plot:temp`.
**Solution**: Ignore or delete them. They're the simulator's demo streams.
**Why**: The logger records every stream it hears, and the simulated device has some of its own.

**Problem**: My file on the board is empty, or the last rows are missing.
**Solution**: Make sure `log.close()` runs, even if the loop ends early.
**Why**: Writes are held in memory and only fully saved when the file is closed.

**Problem**: Each run wipes out the last file.
**Solution**: Open with `"a"` to **a**ppend, or use a new file name each time.
**Why**: `"w"` always starts a fresh file.

## Check your understanding

{% quiz %}
? What does the Data Logger record?
- [ ] Only plotted values
- [x] Every telemetry stream your program sends
- [ ] The Python code
> Plot, meter and scope readings all end up on the same paper.

? Why would you write a log file on the board instead?
- [ ] It's faster
- [ ] The Data Logger can't make CSV files
- [x] It works when no computer is connected
> A roaming robot has nobody listening to its telemetry, so it has to keep its own notes.

? What happens if you forget log.close()?
- [x] Some data may never be saved
- [ ] The file is deleted
- [ ] Nothing at all
> Closing the file writes out anything still waiting in memory.
{% endquiz %}
