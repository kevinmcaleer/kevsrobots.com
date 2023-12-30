---
layout: lesson
title: Data Analysis and Aggregation
type: page
description: Explore the powerful data analysis and aggregation capabilities of Pandas, including summarizing data, grouping, and performing aggregate computations.
cover: /learn/pandas_and_numpy/assets/pandas07.png
---

![Cover photo]({{page.cover}}){:class="cover"}

## Overview

This lesson delves into Data Analysis and Aggregation with Pandas. Effective data analysis often involves summarizing data, grouping it based on certain criteria, and performing aggregate computations. We will explore these powerful capabilities in Pandas to derive meaningful insights from data.

---

## Summarizing Data

### Descriptive Statistics

Pandas provides convenient methods to get descriptive statistics:

```python
# Descriptive statistics
summary = df.describe()
```

The `describe()` method returns a Data Frame with descriptive statistics for each column in the original Data Frame, like this:

```python
    Column1    Column2
count  5.000000  5.000000
mean   0.000000  0.000000
std    0.707107  0.707107
min   -1.000000 -1.000000
25%   -1.000000 -1.000000
50%    0.000000  0.000000
75%    1.000000  1.000000
max    1.000000  1.000000
```

---

### Unique Values, Value Counts, and Membership

You can also explore unique values and counts:

```python
# Unique values and counts
unique_values = df['ColumnName'].unique()
value_counts = df['ColumnName'].value_counts()
```

---

## Grouping Data

### Group By

Group data by one or more columns:

```python
# Grouping data
grouped = df.groupby('ColumnName')
```

---

### Aggregating Data

Perform aggregate computations on groups:

```python
# Aggregating data
aggregated_data = grouped.aggregate(np.sum)
```

---

## Pivot Tables

Create pivot tables for multidimensional data analysis:

```python
# Pivot tables
pivot_table = df.pivot_table(values='ValueColumn', index='RowColumn', columns='ColumnColumn')
```

---

## Summary

In this lesson, we've explored essential aspects of data analysis and aggregation using Pandas. Understanding how to summarize, group, and aggregate data is crucial for effective data analysis and gaining insights.

---
