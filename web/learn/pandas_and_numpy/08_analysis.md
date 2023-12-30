---
layout: lesson
title: Data Analysis and Aggregation
author: Kevin McAleer
type: page
cover: /learn/pandas_and_numpy/assets/pandas07.png
date: 2023-11-14
previous: 07_cleansing.html
next: 09_advanced.html
description: Explore the powerful data analysis and aggregation capabilities of Pandas,
  including summarizing data, grouping, and performing aggregate computations.
percent: 64
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
