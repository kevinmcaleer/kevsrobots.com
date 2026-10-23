---
layout: lesson
title: Basic Operations
type: page
description: Master the fundamental operations with Data Frames in Pandas, including data selection, filtering, sorting, and basic computations.
cover: /learn/pandas_and_numpy/assets/pandas04.png
---

![Cover photo]({{page.cover}}){:class="cover"}

## Overview

Data Frames are central to data manipulation in Pandas, and knowing how to select, filter, sort and summarise them is key to unlocking their power. In this lesson we'll use a small table of robot sensor readings, which we'll keep using for the next few lessons.

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

## Try it yourself

Find all the readings from the `distance` sensor, sorted from nearest to furthest.

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
{% endquiz %}

---

## Summary

In this lesson, we have covered data selection, filtering, sorting, and basic computations in Pandas. These operations are fundamental to data manipulation and analysis.

---
