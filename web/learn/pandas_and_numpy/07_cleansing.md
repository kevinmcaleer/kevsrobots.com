---
layout: lesson
title: Data Cleaning and Preparation
author: Kevin McAleer
type: page
cover: /learn/pandas_and_numpy/assets/pandas06.png
date: 2023-11-14
previous: 06_import_export.html
next: 08_analysis.html
description: Learn the essentials of data cleaning and preparation in Pandas, including
  handling missing data, data transformation, and data filtering for effective data
  analysis.
percent: 56
duration: 3
quiz_total: 17
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

Real data is messy: values go missing, rows get duplicated and text comes in different shapes. In this lesson we'll clean up the robot sensor data from the last lesson.

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

## Handling Missing Data

### Identifying Missing Data

Missing values show up as `NaN` (Not a Number). To find them:

```python
print(df.isna())          # True wherever a value is missing
print(df.isna().sum())    # how many are missing in each column
```

`isnull()` does exactly the same as `isna()`.

---

### Filling Missing Data

You can fill missing data with a fixed value, a calculated value, or by interpolating between neighbours:

```python
# Fill with zero
df_zero = df.fillna({'reading': 0})

# Fill with the column average
df_mean = df.fillna({'reading': df['reading'].mean()})

# Estimate from the values either side
df['reading_estimated'] = df['reading'].interpolate()
```

Interpolating works best when rows are in a meaningful order, such as readings over time.

---

### Dropping Missing Data

Alternatively, you can drop rows or columns with missing values:

```python
# Drop rows with any missing data
df_dropped_rows = df.dropna()

# Drop columns with any missing data
df_dropped_columns = df.dropna(axis=1)
```

---

## Removing duplicates

To remove rows that are exact copies of another row, use `drop_duplicates()`:

```python
df = df.drop_duplicates()
```

Pass `subset=['robot', 'sensor']` to treat rows as duplicates when only those columns match.

---

## Data Transformation

### Applying Functions

`apply()` runs a function on every value in a column:

```python
# Convert distance readings from cm to inches
df['inches'] = df['reading'].apply(lambda cm: cm / 2.54)
```

For simple maths like this, operating on the whole column is shorter and much faster, and gives the same result:

```python
df['inches'] = df['reading'] / 2.54
```

---

### Replacing Values

Replace specific values in a column:

```python
df['sensor'] = df['sensor'].replace('imu', 'motion')
```

---

### Tidying text

The `.str` accessor applies string methods to a whole column:

```python
df['robot'] = df['robot'].str.strip().str.lower()
```

This removes stray spaces and makes the names lower case, so `'SMARS '` and `'smars'` count as the same robot.

---

### Changing data types

Columns sometimes load with the wrong type, such as numbers stored as text. Use `astype()` to convert them:

```python
df['battery'] = df['battery'].astype(float)
print(df.dtypes)
```

---

## Try it yourself

Fill the Quad's missing reading with the average of the other `distance` readings, then check `df.isna().sum()` shows no missing values.

{% quiz %}
? What does `df.isna().sum()` tell you?
- [x] How many missing values each column has
- [ ] The total of every column
- [ ] Which rows are duplicates
> `isna()` gives True for each missing value, and `sum()` counts the Trues in each column.

? Which is usually faster for simple maths on a whole column?
- [ ] `df['reading'].apply(lambda cm: cm / 2.54)`
- [x] `df['reading'] / 2.54`
> Vectorised column operations run in fast compiled code instead of calling a Python function for every value.
{% endquiz %}

---

## Summary

This lesson covered essential techniques in data cleaning and preparation using Pandas: finding and handling missing data, removing duplicates, transforming values and fixing data types.

---
