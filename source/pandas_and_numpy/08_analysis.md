---
layout: lesson
title: Data Analysis and Aggregation
type: page
description: Explore the powerful data analysis and aggregation capabilities of Pandas, including summarizing data, grouping, and performing aggregate computations.
cover: /learn/pandas_and_numpy/assets/pandas07.png
---

![Cover photo]({{page.cover}}){:class="cover"}

## Overview

Effective data analysis often involves summarising data, grouping it, and performing calculations on each group. We'll keep using the robot sensor data.

```python
import pandas as pd

df = pd.DataFrame({
    'robot':   ['BurgerBot', 'SMARS', 'BurgerBot', 'SMARS', 'Quad', 'Quad'],
    'sensor':  ['distance', 'distance', 'line', 'distance', 'imu', 'distance'],
    'reading': [23.5, 41.0, 1.0, 12.2, None, 30.1],
    'battery': [87, 64, 85, 60, 92, 90],
})
print(df)
```

---

## Summarizing Data

### Descriptive Statistics

`describe()` gives you the key statistics for every numeric column in one go:

```python
print(df.describe())
```

```shell
         reading    battery
count   5.000000   6.000000
mean   21.560000  79.666667
std    15.531677  13.952300
min     1.000000  60.000000
25%    12.200000  69.250000
50%    23.500000  86.000000
75%    30.100000  89.250000
max    41.000000  92.000000
```

- `count` is how many values are present. `reading` has 5 because one is missing
- `mean` is the average and `std` the standard deviation (how spread out the values are)
- `25%`, `50%` and `75%` are percentiles. The `50%` value is the median

---

### Unique Values and Value Counts

```python
print(df['robot'].unique())         # each different robot, once
print(df['robot'].value_counts())   # how many rows each robot has
```

---

## Grouping Data

### Group By

`groupby()` splits the data into groups that share a value:

```python
grouped = df.groupby('robot')
```

---

### Aggregating Data

Then you run a calculation on each group:

```python
# Average battery for each robot
print(grouped['battery'].mean())

# Several calculations at once
print(grouped['battery'].agg(['min', 'max', 'mean']))
```

```shell
           min  max  mean
robot
BurgerBot   85   87  86.0
Quad        90   92  91.0
SMARS       60   64  62.0
```

> ## Older examples
>
> You may see `grouped.aggregate(np.sum)` in older tutorials. Recent versions of Pandas prefer the name as a string, like `.agg('sum')`, or simply `.sum()`.

---

## Pivot Tables

A pivot table summarises one column, broken down by two others:

```python
pivot = df.pivot_table(values='reading', index='robot', columns='sensor', aggfunc='mean')
print(pivot)
```

Each robot becomes a row, each sensor becomes a column, and each cell holds the average reading. Combinations with no data show `NaN`.

---

{% quiz %}
? In the output of `describe()`, what does the `50%` row show?
- [ ] The average
- [x] The median, the middle value
- [ ] Half of the maximum
> The 50th percentile is the median. The average is the `mean` row.

? What does `df.groupby('robot')['battery'].mean()` calculate?
- [ ] The average battery across all robots
- [x] The average battery for each robot separately
- [ ] The number of robots
> `groupby()` splits the rows by robot, then `mean()` runs on each group.
{% endquiz %}

---

## Summary

In this lesson, we've explored summarising, grouping and aggregating data with Pandas, and building pivot tables. These are the tools you'll reach for to turn raw data into answers.

---
