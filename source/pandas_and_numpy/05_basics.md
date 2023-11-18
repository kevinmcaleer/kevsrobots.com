---
layout: lesson
title: Basic Operations
type: page
description: Master the fundamental operations with Data Frames in Pandas, including data selection, filtering, sorting, and basic computations.
cover: assets/pandas04.png
---

![Cover photo]({{page.cover}}){:class="cover"}

## Overview

Welcome to the lesson on Basic Operations with Data Frames in Pandas. Data Frames are central to data manipulation in Pandas, and knowing how to effectively perform various operations on them is key to unlocking their full potential. This lesson covers essential techniques like selection, filtering, sorting, and basic computations.

---

## Data Selection

### Selecting Columns

To select a single column, use the column label:

```python
# Selecting a single column
selected_column = df['ColumnName']
```

To select multiple columns, use a list of column labels:

```python
# Selecting multiple columns
selected_columns = df[['Column1', 'Column2']]
```

---

### Selecting Rows

Rows can be selected by their position using `iloc` or by index using `loc`:

```python
# Selecting rows by position
rows_by_position = df.iloc[0:5]  # First five rows

# Selecting rows by index
rows_by_index = df.loc['IndexLabel']
```

---

## Filtering Data

You can filter data based on conditions:

```python
# Filtering data
filtered_data = df[df['ColumnName'] > value]
```

---

## Sorting Data

Data in a Data Frame can be sorted by the values of one or more columns:

```python
# Sorting data
sorted_data = df.sort_values(by='ColumnName')
```

---

## Basic Computations

Pandas allows for basic statistical computations:

```python
# Basic computations
mean_value = df['ColumnName'].mean()
sum_value = df['ColumnName'].sum()
```

---

## Dropping rows and columns

You can drop rows or columns from a Data Frame:

```python
# Dropping rows
df_dropped_rows = df.drop([0, 1, 2])

# Dropping columns
df_dropped_columns = df.drop(['Column1', 'Column2'], axis=1)
```

> ## Axis
>
> Note that `axis=1` indicates that the operation should be performed on columns, whereas `axis=0` indicates
> that it should be performed on rows.

---

## Summary

In this lesson, we have covered the basics of data selection, filtering, sorting, and performing basic computations in Pandas. These operations are fundamental to data manipulation and analysis.

---
