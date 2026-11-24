---
layout: lesson
title: Recap and Review
type: page
description: Consolidate your learning journey in data manipulation with Pandas and NumPy, reviewing key concepts and discussing how to continue developing your data analysis skills.
cover: /learn/pandas_and_numpy/assets/banners/12_recap.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to recall the main Pandas tools from the course and pick your next project.


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
- If Python is still new, revisit [Python for beginners](/learn/python/).

### Projects

- Apply your skills to personal or open-source projects.
- Analyze datasets from different domains to gain diverse experience.

## Try it Yourself

Pull the whole course together with a small project on your own robot data:

1. Make a Data Frame of a week of readings from your robot (a battery level, a distance or a temperature). If you do not have a real log, invent one with NumPy as in the case study lesson.
2. Save it to CSV, read it back, and check for missing values.
3. Use `groupby()` or `resample()` to find the average for each day.
4. Challenge: chart it with `plot()` and save the chart as a picture.

---

## Check your understanding

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

? You have two tables of robot logs that share a `robot` column. Which function matches them up by that column?
- [ ] `pd.concat()`
- [x] `pd.merge()`
- [ ] `df.describe()`
> `merge()` matches rows by a key, while `concat()` simply stacks tables.
{% endquiz %}

---

## Conclusion

Congratulations on completing the `Data Manipulation with Pandas and NumPy` course. You've acquired valuable skills that are essential in the field of data analysis. Remember, the journey of learning and improvement is continuous, and there's always more to explore and master.

---

We hope you found this course enriching and empowering. If you have any final questions or thoughts, please share them in the comments or reach out to us directly. Keep learning, keep analyzing, and all the best in your data analysis endeavors!

---
