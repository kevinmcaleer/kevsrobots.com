---
layout: lesson
title: Importing and Exporting Data
author: Kevin McAleer
type: page
cover: /learn/pandas_and_numpy/assets/pandas05.png
date: 2023-11-14
previous: 05_basics.html
next: 07_cleansing.html
description: Learn how to import data from various file formats into Pandas Data Frames
  and export Data Frames back into these formats for efficient data handling.
percent: 48
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

This lesson is centered on importing and exporting data using Pandas. We'll cover how to read data from various sources like CSV, Excel, and YAML files into Pandas Data Frames, and similarly, how to export Data Frames into these file formats. Mastering these skills is essential for efficient data handling and analysis.

---

## Importing Data

### Reading from CSV

To read data from a CSV file into a Data Frame, use `pd.read_csv()`:

```python
import pandas as pd

# Reading from a CSV file
df = pd.read_csv('path/to/your/file.csv')
print(df)
```

---

### Reading from Excel

Reading from an Excel file is just as straightforward:

```python
# Reading from an Excel file
df = pd.read_excel('path/to/your/file.xlsx')
print(df)
```

---

### Removing duplicates

To remove duplicate rows from a Data Frame, use `df.drop_duplicates()`:

```python
# Removing duplicate rows
df = df.drop_duplicates()
```

---

### Reading from YAML

To read YAML data, you'll need an additional library, `PyYAML`:

```python
import yaml
import pandas as pd

# Reading from a YAML file
with open('path/to/your/file.yaml', 'r') as file:
    yaml_data = yaml.safe_load(file)

df = pd.DataFrame(yaml_data)
print(df)
```

---

## Exporting Data

### Writing to CSV

You can export a Data Frame to a CSV file using `df.to_csv()`:

```python
# Writing to a CSV file
df.to_csv('path/to/your/newfile.csv')
```

---

### Writing to Excel

Similarly, to export to an Excel file:

```python
# Writing to an Excel file
df.to_excel('path/to/your/newfile.xlsx')
```

---

## Summary

In this lesson, we've covered the fundamentals of importing and exporting data using Pandas. You've learned how to work with different file formats, which is a key part of the data analysis workflow.

---
