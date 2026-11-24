---
layout: lesson
title: Basic Operations
type: page
description: Master the fundamental operations with Data Frames in Pandas, including data selection, filtering, sorting, and basic computations.
cover: /learn/pandas_and_numpy/assets/pandas04.png
date_updated: 2026-11-24
---

![Cover photo]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to select, filter and sort the rows of a Data Frame and calculate simple statistics.

## Overview

Data Frames are central to data manipulation in Pandas, and knowing how to select, filter, sort and summarise them is key to unlocking their power. In this lesson we'll use a small table of robot sensor readings, which we'll keep using for the next few lessons.

{% capture snakie_code %}
import pandas as pd

df = pd.DataFrame({
    'robot':   ['BurgerBot', 'SMARS', 'BurgerBot', 'SMARS', 'Quad', 'Quad'],
    'sensor':  ['distance', 'distance', 'line', 'distance', 'imu', 'distance'],
    'reading': [23.5, 41.0, 1.0, 12.2, None, 30.1],
    'battery': [87, 64, 85, 60, 92, 90],
})
print(df)
{% endcapture %}
{% include snakie.html code=snakie_code name="basics.py" engine="pyodide" packages="numpy,pandas" %}

---

## Data Selection

### Selecting Columns

To select a single column, use the column name. You get a Series back:

```python
batteries = df['battery']
```

To select multiple columns, use a list of column names. You get a Data Frame back:

```python
subset = df[['robot', 'battery']]
```

---

### Selecting Rows

Rows can be selected by their position using `iloc`, or by their index label using `loc`:

```python
# By position: the first three rows
first_three = df.iloc[0:3]

# By label: the row whose index is 2
row = df.loc[2]
```

Our index is just the numbers 0 to 5, so `loc` and `iloc` look similar here. They behave differently once the index has other labels, such as names or dates.

---

## Filtering Data

Put a condition inside the square brackets to keep only the rows where it's true:

```python
# Robots with more than 80% battery
charged = df[df['battery'] > 80]

# Combine conditions with & (and) and | (or), with brackets around each one
smars_distance = df[(df['robot'] == 'SMARS') & (df['sensor'] == 'distance')]
```

---

## Sorting Data

```python
# Lowest battery first
by_battery = df.sort_values(by='battery')

# Highest battery first
by_battery_desc = df.sort_values(by='battery', ascending=False)
```

---

## Basic Computations

```python
print(df['battery'].mean())   # average battery level
print(df['battery'].sum())
print(df['battery'].max())
```

Pandas skips missing values in calculations like `mean()`, so the empty reading for the Quad doesn't cause an error.

---

## Dropping rows and columns

```python
# Drop rows by index label
fewer_rows = df.drop([0, 1])

# Drop a column
no_sensor = df.drop(columns=['sensor'])
```

`drop()` returns a new Data Frame and leaves `df` unchanged. Assign the result back (`df = df.drop(...)`) to keep the change.

> ## Axis
>
> You'll also see `df.drop(['sensor'], axis=1)`. `axis=1` means "columns" and `axis=0` means "rows". Using `columns=` is easier to read.

---

## Try it Yourself

1. Find all the readings from the `distance` sensor, sorted from nearest to furthest.
2. Show only the `robot` and `battery` columns for robots with a battery below 70.
3. Print the average battery of the rows where the sensor is `distance`.
4. Challenge: find the robot with the lowest battery in one line, using `sort_values()` and `iloc[0]`.

---

## Common Issues

- **Problem**: `ValueError: The truth value of a Series is ambiguous` when you filter.
- **Solution**: Use `&` for and, `|` for or, and put brackets round each condition, like `df[(df['battery'] > 80) & (df['sensor'] == 'distance')]`.
- **Why**: Python's `and` and `or` only work on single True or False values. `&` and `|` compare each row.

- **Problem**: You changed a filtered table, but the original `df` did not change (older versions of Pandas may show a `SettingWithCopyWarning`).
- **Solution**: Change the original with `loc`, for example `df.loc[df['battery'] > 80, 'battery'] = 100`.
- **Why**: A filtered table is a copy. Current Pandas never writes changes on a copy back to `df`, so use `loc` on `df` itself.

- **Problem**: You called `df.drop(...)` but the column is still there.
- **Solution**: Assign the result back, `df = df.drop(columns=['sensor'])`.
- **Why**: `drop()` returns a new Data Frame and leaves the original alone.

## Check your understanding

{% quiz %}
? What does `df[df['battery'] > 80]` return?
- [ ] `True` or `False`
- [x] Only the rows where battery is above 80
- [ ] The battery column with values over 80 set to 80
> A condition inside the brackets filters the rows.

? What's the difference between `iloc` and `loc`?
- [x] `iloc` uses positions, `loc` uses index labels
- [ ] `iloc` selects columns, `loc` selects rows
- [ ] There is no difference
> `iloc[0]` is always the first row, while `loc[0]` is the row labelled 0.

? Which operator combines two conditions in a filter?
- [ ] `and`
- [x] `&`, with brackets round each condition
- [ ] `+`
> `and` fails on a whole column. `&` compares row by row.
{% endquiz %}

---

## Summary

In this lesson, we have covered data selection, filtering, sorting, and basic computations in Pandas. These operations are fundamental to data manipulation and analysis.

---
