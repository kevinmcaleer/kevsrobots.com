---
layout: lesson
title: Basic Operations
author: Kevin McAleer
type: page
cover: assets/pandas04.png
previous: 04_dataframes.html
next: 06_import_export.html
description: Master the fundamental operations with Data Frames in Pandas, including
  data selection, filtering, sorting, and basic computations.
percent: 40
duration: 2
navigation:
- name: Data Manipulation with Pandas and Numpy
- content:
  - section: Overview
    content:
    - name: About this course
      link: 01_overview.html
  - section: Introduction to Pandas
    content:
    - name: Pandas and NumPy
      link: 02_intro.html
    - name: Installing Pandas and NumPy
      link: 03_installing.html
  - section: Pandas
    content:
    - name: Understanding Data Frames in Pandas
      link: 04_dataframes.html
    - name: Basic Operations
      link: 05_basics.html
    - name: Importing and Exporting Data
      link: 06_import_export.html
    - name: Data Cleaning and Preparation
      link: 07_cleansing.html
    - name: Data Analysis and Aggregation
      link: 08_analysis.html
    - name: Advanced Data Manipulation Techniques
      link: 09_advanced.html
  - section: MatPlotLib
    content:
    - name: Visualization with Pandas and Matplotlib
      link: 10_visualisation.html
    - name: Practical Examples and Case Studies
      link: 11_case_studies.html
  - section: Conclusion
    content:
    - name: Recap and Review
      link: 12_recap.html
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
