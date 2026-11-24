---
layout: lesson
title: Practical Examples and Case Studies
type: page
description: Apply your skills with Pandas and NumPy to real-world scenarios through practical examples and detailed case studies.
cover: /learn/pandas_and_numpy/assets/pandas10.png
date_updated: 2026-11-24
---

![Cover photo]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to combine cleaning, analysis and charting in one worked example, and choose a project of your own.

## Overview

Time to put it all together. We'll work through one complete example, from generating data to a chart, then give you some projects to try on your own.

---

## Worked example: robot battery log

Imagine your robot logs its battery level every hour for a week. We'll use NumPy to make some realistic-looking data, then clean it, analyse it and chart it with Pandas.

{% capture snakie_code %}
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# 1. Make a week of hourly readings
rng = np.random.default_rng(seed=42)
times = pd.date_range('2024-01-01', periods=24 * 7, freq='h')
battery = 100 - (np.arange(len(times)) % 24) * 3 + rng.normal(0, 2, len(times))

log = pd.DataFrame({'time': times, 'battery': battery.round(1)})

# 2. Knock out a few readings to simulate a flaky sensor
log.loc[rng.choice(len(log), 10, replace=False), 'battery'] = np.nan

# 3. Clean: fill gaps by interpolating between neighbouring readings
print("Missing before:", log['battery'].isna().sum())
log['battery'] = log['battery'].interpolate()
print("Missing after:", log['battery'].isna().sum())

# 4. Analyse: lowest battery each day, and a 6-hour rolling average
daily_min = log.resample('D', on='time')['battery'].min()
print(daily_min)

log['smooth'] = log['battery'].rolling(window=6).mean()

# 5. Visualise
log.plot(x='time', y=['battery', 'smooth'], title='Robot battery level')
plt.ylabel('Battery %')
plt.show()
{% endcapture %}
{% include snakie.html code=snakie_code name="case_studies.py" engine="pyodide" packages="numpy,pandas,matplotlib" %}

### How it works

1. `pd.date_range()` makes 168 hourly timestamps. NumPy builds a battery level that drains through each day (the robot is recharged every 24 hours) plus a little random noise. The `seed` means you'll get the same numbers every time
2. We set 10 random readings to `NaN` to pretend the sensor dropped out
3. `interpolate()` fills each gap with a value between its neighbours, which suits a smoothly changing reading like this
4. `resample('D', on='time')` groups the readings by day, and `rolling(window=6).mean()` averages each reading with the five before it, smoothing out the noise
5. `plot()` draws the raw and smoothed lines together

---

## Try it Yourself

1. Change `window=6` to `window=12` and look at the chart. Is the smoothed line smoother or more delayed?
2. Print the time of the lowest battery reading with `log.loc[log['battery'].idxmin()]`.
3. Count how many hours the battery was below 40 with `(log['battery'] < 40).sum()`.
4. Challenge: change the `seed` and the number of dropped-out readings, then check the clean-up still works.

---

## Common Issues

- **Problem**: The first few `smooth` values are `NaN`.
- **Solution**: This is expected. If you want a value from the first reading, use `rolling(window=6, min_periods=1)`.
- **Why**: A rolling average needs a full window of readings before it can give an answer.

- **Problem**: A different `seed` gives different numbers from the ones in this lesson.
- **Solution**: Use the same `seed=42` to match, or leave the seed out if you do not need repeatable results.
- **Why**: The seed starts the random number generator at a known place, so the same seed gives the same numbers.

> **Teachers**: the worked example makes its own data, so there is nothing to download, and it runs in a fraction of a second. For the projects, real datasets are a good chance to talk about size. A school machine may struggle to open a file with millions of rows, so give learners a trimmed sample (a few thousand rows) and remind them to check where a dataset came from and whether they are allowed to use it.

---

## Projects to try

### Financial data

Download historical prices for a stock or cryptocurrency as a CSV file. Load it with `read_csv()`, parse the dates, and plot the closing price with 20-day and 50-day rolling averages.

### Survey data

Make a small CSV of survey answers with some blanks. Count missing values, fill or drop them, turn text answers like "Agree" into numbers with `replace()`, and use `value_counts()` to summarise each question.

### Sales forecasting

Group a year of sales data by month with `resample('MS')`, plot the trend, and use a rolling average to estimate next month's sales.

### Your own sensors

Log readings from a real sensor, such as a Raspberry Pi Pico's temperature sensor, to a CSV file. Then load and analyse it with everything you've learned.

---

## Check your understanding

{% quiz %}
? Why does the worked example use `rolling(window=6).mean()`?
- [ ] To fill in missing values
- [x] To smooth out noise by averaging each reading with the ones before it
- [ ] To sort the readings
> A rolling average shows the overall trend without the jitter of individual readings.

? Why does the example set a `seed` for the random number generator?
- [ ] To make the chart colourful
- [x] So you get the same numbers every time you run it
- [ ] To speed up the code
> A fixed seed makes the random data repeatable, so your results match the lesson.

? What does `interpolate()` do to the dropped-out readings?
- [ ] Deletes the rows
- [x] Fills each gap with a value between its neighbours
- [ ] Sets them to zero
> The battery changes smoothly, so estimating from the readings either side is reasonable.
{% endquiz %}

---

## Summary

This lesson worked through a complete analysis, from raw data to a chart, and suggested real-world projects to practise on. Real data is the best teacher, so pick a project and dive in!

---
