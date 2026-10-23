---
layout: lesson
title: Recap and Review
author: Kevin McAleer
type: page
cover: /learn/pandas_and_numpy/assets/pandas.png
date: 2023-11-14
previous: 11_case_studies.html
description: Consolidate your learning journey in data manipulation with Pandas and
  NumPy, reviewing key concepts and discussing how to continue developing your data
  analysis skills.
percent: 100
duration: 2
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

Welcome to the final lesson of our `Data Manipulation with Pandas and NumPy` course. This lesson serves as a recap and review, summarizing the key concepts we've covered and providing guidance on how to continue developing your skills in data analysis.

## Key Topics Revisited

### Pandas and NumPy Basics

- Introduction to Pandas and NumPy
- Installation and setup

### Data Frames and Operations

- Creating and manipulating Data Frames
- Basic operations like sorting, filtering, and aggregating data

### Data Cleaning and Preparation

- Handling missing data
- Data transformation techniques

### Advanced Data Manipulation

- Merging, joining, and handling time series data

### Visualization

- Using Pandas and Matplotlib for data visualization

### Practical Applications

- Real-world examples and case studies

## Continuing Your Data Analysis Journey

### Further Learning

- Deepen your understanding of Pandas and NumPy through advanced tutorials and documentation.
- Explore other Python libraries like SciPy and Seaborn for extended functionalities.
- Store your data in a database with [Create Databases with Python and SQLite3](/learn/sqlite3/), or learn the language of databases with [Getting Started with SQL](/learn/sql/).

### Projects

- Apply your skills to personal or open-source projects.
- Analyze datasets from different domains to gain diverse experience.

{% quiz %}
? Which method would you use to fill gaps in a smoothly changing sensor reading?
- [ ] `dropna()`
- [x] `interpolate()`
- [ ] `drop_duplicates()`
> Interpolation estimates each missing value from its neighbours.

? Which method gives you count, mean, min, max and percentiles in one go?
- [x] `describe()`
- [ ] `info()`
- [ ] `value_counts()`
> `describe()` summarises every numeric column.
{% endquiz %}

---

## Conclusion

Congratulations on completing the `Data Manipulation with Pandas and NumPy` course. You've acquired valuable skills that are essential in the field of data analysis. Remember, the journey of learning and improvement is continuous, and there's always more to explore and master.

---

We hope you found this course enriching and empowering. If you have any final questions or thoughts, please share them in the comments or reach out to us directly. Keep learning, keep analyzing, and all the best in your data analysis endeavors!

---
