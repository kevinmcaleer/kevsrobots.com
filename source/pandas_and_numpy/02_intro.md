---
layout: lesson
title: Pandas and NumPy
type: page
description: Kickstart your journey in data manipulation in Python with an introduction to Pandas and NumPy, two powerhouse libraries.
cover: /learn/pandas_and_numpy/assets/pandas01.png
---

![Cover photo]({{page.cover}}){:class="cover"}

## Overview

Welcome to the first lesson of the `Data Manipulation with Pandas and NumPy` course. This lesson serves as your gateway into the world of data analysis and manipulation in Python. Pandas and NumPy are two of the most popular libraries used in data science and analytics. They provide powerful tools to manipulate, analyze, and visualize data in Python.

---

## What is Pandas?

`Pandas` is an open-source library providing high-performance, easy-to-use data structures and data analysis tools. Its main data structure, the DataFrame, allows you to store and manipulate tabular data in rows of observations and columns of variables.

The Pandas documentation is hosted at <https://pandas.pydata.org>.

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

```shell
pip install pandas numpy
```

The next lesson covers installation in more detail.

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

<figure style="margin:0;border:1px solid #e2e8f0;border-radius:8px;overflow:hidden"><iframe src="https://coddy.tech/embed-editor?lang=matplotlib&theme=light&layout=stacked&code=aW1wb3J0IG51bXB5IGFzIG5wCmltcG9ydCBwYW5kYXMgYXMgcGQKCiMgQSBOdW1QeSBhcnJheQphcnIgPSBucC5hcnJheShbMSwgMiwgMywgNCwgNV0pCnByaW50KGFycikKCiMgQSBQYW5kYXMgU2VyaWVzCnMgPSBwZC5TZXJpZXMoWzEsIDIsIDMsIDQsIDVdKQpwcmludChzKQo&credit=0" width="100%" height="460" style="display:block;border:0;width:100%" title="Python code editor by Coddy" loading="lazy"></iframe><figcaption style="box-sizing:border-box;padding:5px 12px;font:11px/1.4 system-ui,-apple-system,sans-serif;color:#64748b;background:#f1f5f9;border-top:1px solid #e2e8f0;text-align:right" dir="ltr">Powered by <a href="https://coddy.tech/embed" target="_blank" rel="noopener" style="color:#0077ff;font-weight:600;text-decoration:none;">Coddy</a></figcaption></figure>

---

{% quiz %}
? What is a Pandas Series?
- [x] A one-dimensional labelled array, like a single column
- [ ] A two-dimensional table
- [ ] A type of chart
> A DataFrame is made of Series: each column is one.

? Which library is Pandas built on top of?
- [ ] Matplotlib
- [x] NumPy
- [ ] Seaborn
> Pandas stores its data in NumPy arrays, which is why the two are taught together.
{% endquiz %}

---

## Summary

This introductory lesson provided a glimpse into what Pandas and NumPy are and their significance in data analysis. In the upcoming lessons, we will delve deeper into these libraries, exploring various functionalities and how they can be applied to real-world data.

---
