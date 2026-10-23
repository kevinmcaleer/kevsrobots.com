---
layout: lesson
title: Understanding Data Frames in Pandas
type: page
description: Dive into the world of Data Frames in Pandas, learn how to create, manipulate, and extract information from these powerful data structures.
cover: /learn/pandas_and_numpy/assets/pandas03.png
---

![Cover photo]({{page.cover}}){:class="cover"}

## Overview

Welcome to the exciting lesson on Data Frames in Pandas. Data Frames are one of the most important and widely used data structures in Pandas. They allow you to store and manipulate tabular data efficiently. In this lesson, we'll explore creating Data Frames, performing basic operations, and how they can be used in data analysis.

---

## What is a Data Frame?

A Data Frame in Pandas is a two-dimensional, size-mutable[^1], and potentially heterogeneous[^2] tabular data structure with labeled axes (rows and columns). It's akin to a spreadsheet or SQL table and is the most commonly used Pandas object.

**A Pandas Data Frame**

![A Dataframe image](assets/dataframe.png){:class="img-fluid w-50"}

---

**Data Frame Axis**

A Data Frame has two axes. **Axis 0** runs down the rows, and its labels are called the `index`. **Axis 1** runs across the columns, and its labels are the column names. You'll see `axis=0` and `axis=1` in many Pandas methods.

![A Dataframe image](assets/axis.png){:class="img-fluid w-50"}

---

**Data Frame Series**

Data Frame Series are one-dimensional labeled arrays capable of holding data of any type (integer, string, float, etc.). They are the building blocks of Data Frames.

![A Dataframe image](assets/series.png){:class="img-fluid w-50"}

---

**Data Frame Rows**

Data Frame Rows are the horizontal lines that contain the data. Each row is assigned a unique index value.

![A Dataframe image](assets/rows.png){:class="img-fluid w-50"}

---

[^1]: Size-mutable means that the size of a Data Frame can be changed after creation. 

[^2]: Heterogeneous means that the data in a Data Frame can be of different types (e.g., integer, float, string, etc.).

---

## Why use Data Frames?

DataFrames, a fundamental feature of pandas in Python, are widely used in data analysis for several reasons:

1. **Structured Data Representation**: DataFrames provide a tabular structure, which is intuitive and aligns well with how data is often organized (similar to spreadsheets).

2. **Efficient Data Manipulation**: They allow for efficient, easy manipulation of data, including filtering, replacing, and aggregating values.

3. **Handling Large Datasets**: DataFrames are optimized for performance, enabling the handling of large datasets effectively.

4. **Data Analysis**: They offer numerous built-in methods for data analysis, making it easier to perform complex statistical analysis, groupings, and pivots.

5. **Integration with Other Tools**: DataFrames seamlessly integrate with a variety of data sources and can be easily exported to different file formats.

6. **Visualization Support**: They are compatible with various data visualization libraries, simplifying the creation of charts and graphs from the data.

7. **Ease of Use**: Pandas DataFrames have a user-friendly syntax, making data manipulation and analysis more accessible.

In summary, DataFrames simplify data manipulation and analysis, making them a preferred choice for data scientists and analysts.

---

## Creating a Data Frame

You can create a Data Frame from various sources like lists, dictionaries, or external data sources (CSV, Excel files). Here's an example of creating a Data Frame from a dictionary:

```python
import pandas as pd

data = {
    'Name': ['Alice', 'Bob', 'Charlie'],
    'Age': [25, 30, 35],
    'City': ['New York', 'Paris', 'London']
}

df = pd.DataFrame(data)
print(df)
```

---

## Data Frame Shape

You can check the shape of a Data Frame using `df.shape`:

```python
print(df.shape)
```

This will produce the output:

```shell
(3, 3)
```

This means the Data Frame has 3 rows and 3 columns.

---

## Basic Operations

### Viewing Data

To view the top and bottom rows of the frame, use `df.head()` and `df.tail()`:

```python
print(df.head()) # first five rows
print(df.tail()) # last five rows
```

Our Data Frame only has three rows, so both show all of it. Pass a number to see fewer, like `df.head(2)`.

---

### Data Selection

You can select a specific column or row from a Data Frame:

```python
# Selecting a column
print(df['Name'])

# Selecting a row by position (rows count from 0, so this is Bob)
print(df.iloc[1])
```

---

### Adding and Deleting Columns

You can easily add new columns or remove existing ones:

```python
# Adding a new column
df['Salary'] = [70000, 80000, 90000]

# Deleting a column
del df['Age']
```
---

### Replacing data in Columns

You can easily replace data in a column:

```python
# Replacing data in a column
df['Salary'] = [75000, 85000, 95000]
```

---

If you want to replace one value wherever it appears, you can use `replace()`:

```python
# Replace every 75000 with 76000
df['Salary'] = df['Salary'].replace(75000, 76000)
```

To change one particular cell, use `loc` with the row label and column name:

```python
df.loc[0, 'Salary'] = 77000   # Alice's salary
```

---

## Code Playground

Experiment with what you have learned in this lesson below:

<figure style="margin:0;border:1px solid #e2e8f0;border-radius:8px;overflow:hidden"><iframe src="https://coddy.tech/embed-editor?lang=matplotlib&theme=light&layout=stacked&code=aW1wb3J0IHBhbmRhcyBhcyBwZAoKZGF0YSA9IHsKICAgICdOYW1lJzogWydBbGljZScsICdCb2InLCAnQ2hhcmxpZSddLAogICAgJ0FnZSc6IFsyNSwgMzAsIDM1XSwKICAgICdDaXR5JzogWydOZXcgWW9yaycsICdQYXJpcycsICdMb25kb24nXQp9CgpkZiA9IHBkLkRhdGFGcmFtZShkYXRhKQpwcmludChkZikKcHJpbnQoZGYuc2hhcGUpCgojIEFkZCBhIGNvbHVtbiwgdGhlbiBkZWxldGUgb25lCmRmWydTYWxhcnknXSA9IFs3MDAwMCwgODAwMDAsIDkwMDAwXQpkZWwgZGZbJ0FnZSddCnByaW50KGRmKQo&credit=0" width="100%" height="460" style="display:block;border:0;width:100%" title="Python code editor by Coddy" loading="lazy"></iframe><figcaption style="box-sizing:border-box;padding:5px 12px;font:11px/1.4 system-ui,-apple-system,sans-serif;color:#64748b;background:#f1f5f9;border-top:1px solid #e2e8f0;text-align:right" dir="ltr">Powered by <a href="https://coddy.tech/embed" target="_blank" rel="noopener" style="color:#0077ff;font-weight:600;text-decoration:none;">Coddy</a></figcaption></figure>

---

{% quiz %}
? A Data Frame has 10 rows and 4 columns. What does `df.shape` return?
- [ ] `(4, 10)`
- [x] `(10, 4)`
- [ ] `40`
> `shape` is always (rows, columns).

? Which axis runs across the columns?
- [ ] Axis 0
- [x] Axis 1
> Axis 0 is the rows (the index) and axis 1 is the columns.
{% endquiz %}

---

## Summary

This lesson introduced the basics of Data Frames in Pandas. We explored how to create Data Frames, perform basic operations, and how they serve as a cornerstone for data manipulation in Python.

---
