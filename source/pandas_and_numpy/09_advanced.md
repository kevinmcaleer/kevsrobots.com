---
layout: lesson
title: Advanced Data Manipulation Techniques
type: page
description: Dive into advanced data manipulation techniques in Pandas, including merging datasets, joining, and handling time series data for sophisticated data analysis.
cover: /learn/pandas_and_numpy/assets/pandas08.png
date_updated: 2026-11-24
---

![Cover photo]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to combine tables with `concat()`, `merge()` and `join()`, and total readings by week with `resample()`.

## Overview

This lesson focuses on Advanced Data Manipulation Techniques with Pandas. Building on the basics, we now venture into more sophisticated techniques like merging datasets, joining, and handling time series data, which are crucial for complex data analysis tasks.

---

## Merging Datasets

### Concatenation

Concatenate Pandas objects along a particular axis:

```python
import pandas as pd

monday = pd.DataFrame({'robot': ['SMARS', 'Quad'], 'distance_m': [12, 8]})
tuesday = pd.DataFrame({'robot': ['SMARS', 'BurgerBot'], 'distance_m': [15, 20]})

# Stack the rows of both tables
all_days = pd.concat([monday, tuesday], ignore_index=True)
```

`ignore_index=True` gives the result a fresh index of 0, 1, 2, 3 instead of repeating 0, 1, 0, 1.

---

### Merge

Merge two datasets that share a column, like a database join:

```python
specs = pd.DataFrame({'robot': ['SMARS', 'Quad', 'BurgerBot'], 'wheels': [4, 0, 2]})

merged = pd.merge(all_days, specs, on='robot')
```

Each row of `all_days` gets the matching `wheels` value from `specs`. Use `how='left'` to keep rows that have no match.

---

## Joining

`join()` matches a column in one table to the **index** of another:

```python
specs_by_robot = specs.set_index('robot')

joined = all_days.join(specs_by_robot, on='robot')
```

This gives the same result as the merge above. `merge()` is usually easier when both tables have the key as a normal column.

---

## Handling Time Series Data

### Date and Time in Pandas

Pandas is robust in handling time series data:

```python
log = pd.DataFrame({
    'date': ['2024-01-01', '2024-01-03', '2024-01-09', '2024-01-12'],
    'distance_m': [12, 15, 9, 20],
})

# Turn the text into real dates
log['date'] = pd.to_datetime(log['date'])
```

---

### Time Series Functions

Utilize functions specific to time series:

```python
# Total distance per week
weekly = log.resample('W', on='date').sum()
print(weekly)
```

```shell
            distance_m
date
2024-01-07          27
2024-01-14          29
```

Each week is labelled with its last day, a Sunday. The 1st and 3rd of January (12 and 15) add up to 27, and the 9th and 12th (9 and 20) add up to 29.

`resample()` groups rows into time periods, here weeks (`'W'`). Use `'D'` for days or `'MS'` for months. If the dates are the index, you can leave out `on=`.

---

## Try it Yourself

1. Run the `merge` example again with `how='left'`, and add a robot to `all_days` that is missing from `specs`. What shows up in the `wheels` column?
2. Use `pd.concat()` without `ignore_index=True`, and print the index. What do you notice?
3. Change `resample('W', ...)` to `resample('MS', ...)` to get a monthly total.
4. Challenge: merge `all_days` with `specs`, then use `groupby('robot')` to find the total distance for each robot.

---

## Common Issues

- **Problem**: `TypeError: Only valid with DatetimeIndex, TimedeltaIndex or PeriodIndex` when you call `resample()`.
- **Solution**: Convert the dates first with `log['date'] = pd.to_datetime(log['date'])`.
- **Why**: Dates read from a file or typed as strings are plain text. `resample()` only works on real dates.

- **Problem**: After `merge()` some rows have `NaN` or have disappeared.
- **Solution**: Check the key values match exactly (spelling, spaces, case), and choose `how='left'` or `how='outer'` to decide which unmatched rows you keep.
- **Why**: The default is an inner merge, which keeps only the rows whose key appears in both tables.

- **Problem**: The key column has different names in the two tables.
- **Solution**: Use `pd.merge(a, b, left_on='name', right_on='robot')`.
- **Why**: `on=` expects the same name in both tables.

---

## Top 10 tips for Data Scientists

Here are 10 essential tips for using pandas in data science:

1. **Use Vectorized Operations**: Leverage pandas' vectorized operations for efficient data manipulation, rather than iterating over DataFrame rows.

2. **Master Indexing and Selecting Data**: Understand how to use `loc[]`, `iloc[]`, and conditional selection to effectively extract and filter data.

3. **Handling Missing Data**: Familiarize yourself with methods like `fillna()`, `dropna()`, and `interpolate()` to handle missing data appropriately.

4. **Use `groupby()` for Aggregation**: Grouping data and performing aggregate functions is key in data analysis; master `groupby()` for these tasks.

5. **Efficiently Merge and Concatenate Data**: Learn to use `merge()`, `join()`, and `concat()` for combining multiple datasets effectively.

6. **Data Type Conversion**: Understand how to change column data types using `astype()` for optimal memory usage and correct data representation.

7. **Use `apply()` for Custom Functions**: When built-in functions don’t suffice, use `apply()` to apply a custom function to DataFrame columns or rows.

8. **Date and Time Handling**: Get comfortable with pandas' time series tools, especially if you're dealing with time-stamped data.

9. **Pivot Tables and Crosstabs**: Learn to reshape data and perform grouped summaries using `pivot_table()` and `crosstab()`.

10. **Optimize Performance and Memory Usage**: Use the `category` data type for columns with only a few different values, and try `query()` for readable filtering.

These tips can significantly enhance your efficiency and effectiveness in data manipulation and analysis using pandas.

---

## Check your understanding

{% quiz %}
? Which function stacks the rows of two Data Frames on top of each other?
- [x] `pd.concat()`
- [ ] `pd.merge()`
- [ ] `df.join()`
> `concat()` stacks; `merge()` and `join()` match rows up using a key.

? What does `log.resample('W', on='date').sum()` do?
- [ ] Sorts the log by date
- [x] Adds up the values for each week
- [ ] Removes weekends
> `resample()` groups rows into time periods, then `sum()` totals each one.

? By default, what does `pd.merge(a, b, on='robot')` keep?
- [ ] Every row from both tables
- [x] Only rows whose `robot` value appears in both tables
- [ ] Only rows from the first table
> The default is an inner merge. Add `how='left'` or `how='outer'` to keep unmatched rows.
{% endquiz %}

---

## Summary

In this lesson, we have covered advanced data manipulation techniques in Pandas, including merging, joining, and handling time series data. These techniques are invaluable when dealing with complex datasets and analyses.

---
