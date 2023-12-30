---
layout: lesson
title: Pandas and NumPy
author: Kevin McAleer
type: page
cover: /learn/pandas_and_numpy/assets/pandas01.png
date: 2023-11-14
previous: 01_overview.html
next: 03_installing.html
description: Kickstart your journey in data manipulation in Python with an introduction
  to Pandas and NumPy, two powerhouse libraries.
percent: 16
duration: 3
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

Welcome to the first lesson of the `Data Manipulation with Pandas and NumPy` course. This lesson serves as your gateway into the world of data analysis and manipulation in Python. Pandas and NumPy are two of the most popular libraries used in data science and analytics. They provide powerful tools to manipulate, analyze, and visualize data in Python.

---

## What is Pandas?

`Pandas` is an open-source library providing high-performance, easy-to-use data structures and data analysis tools. Its main data structure, the DataFrame, allows you to store and manipulate tabular data in rows of observations and columns of variables.

The Pandsas Documentation is hosted at <https://pandas.pydata.prg>.

---

## What is NumPy?

`NumPy`, short for Numerical Python, is a foundational package for numerical computing in Python. It provides support for large, multi-dimensional arrays and matrices, along with a collection of mathematical functions to operate on these arrays.

We use Numpy to perform mathematical and logical operations on arrays. It is the fundamental package for scientific computing in Python.

For example, let's create a NumPy array:

```python
import numpy as np

arr = np.array([1, 2, 3, 4, 5])
print(arr)
```

Output:

```shell
[1 2 3 4 5]
```

---

## Installing Pandas and NumPy

Before diving into the functionalities of Pandas and NumPy, you need to install these libraries. Here's how you can do it:

```python
pip install pandas numpy
```

---

## Basic Concepts

### Data Frames

Data Frames are two-dimensional, size-mutable, and potentially heterogeneous tabular data structures with labeled axes (rows and columns). Think of it as a spreadsheet with superpowers.

---

### Series

A Series, in Pandas, is a one-dimensional array-like object that can hold many data types, such as numbers or strings.

---

## Code Playground

To follow along with the examples in this course, you can use the interactive code playground below. Click the `Run` button to execute the code and see the output.

<iframe src="https://trinket.io/embed/python3/9ce74dbb65?runOption=run&showInstructions=true" width="100%" height="300" frameborder="0" marginwidth="0" marginheight="0" allowfullscreen></iframe>

---

## Summary

This introductory lesson provided a glimpse into what Pandas and NumPy are and their significance in data analysis. In the upcoming lessons, we will delve deeper into these libraries, exploring various functionalities and how they can be applied to real-world data.

---
