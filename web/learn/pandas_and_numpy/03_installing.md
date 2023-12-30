---
layout: lesson
title: Installing Pandas and NumPy
author: Kevin McAleer
type: page
cover: /learn/pandas_and_numpy/assets/pandas02.png
date: 2023-11-14
previous: 02_intro.html
next: 04_dataframes.html
description: This lesson guides you through the installation process of Pandas and
  NumPy, setting the stage for data manipulation tasks in Python.
percent: 24
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

In this lesson, we will walk through the process of installing Pandas and NumPy. These libraries are essential tools in data manipulation and analysis. We'll ensure you have these tools set up correctly to start exploring the vast functionalities they offer.

---

## Prerequisites

Before installing Pandas and NumPy, make sure you have Python installed on your system. Python 3.6 or later is recommended for the best experience. You can check your Python version by running:

```shell
python --version
```

---

## Installing Pandas and NumPy

Pandas and NumPy can be easily installed using Python's package manager, pip. If you have Python installed, pip should already be available on your system.

Run the following commands in your terminal or command prompt to install Pandas and NumPy:

```shell
pip install pandas
pip install numpy
```

These commands download and install the latest versions of Pandas and NumPy from the Python Package Index (PyPI).

---

## Verifying the Installation

After installation, you can verify that Pandas and NumPy are correctly installed by importing them in Python. Open your Python interpreter or a Python script and try the following commands:

```python
import pandas as pd
import numpy as np
print("Pandas version:", pd.__version__)
print("NumPy version:", np.__version__)
```

If the installation was successful, these commands should run without errors, and you will see the installed versions of Pandas and NumPy.

---

## Troubleshooting

If you encounter any issues during installation, ensure that your Python and pip are up-to-date. You can update pip using:

```shell
pip install --upgrade pip
```

For other issues, consulting the official Pandas and NumPy documentation or community forums can be helpful.

---

## Jupyter Notebooks

You may prefer to use a Juypter Notebook, such as the free Google Collaboratory, to run your code. If so, you can install Pandas and NumPy in a Jupyter Notebook by running the following commands in a code cell:

```python
!pip install pandas
!pip install numpy
```

To access Google Collaboratory, you will need a Google account. You can access Google Collaboratory here - <https://colab.research.google.com/>.

---

## Summary

Congratulations! You have successfully installed Pandas and NumPy. These libraries are now ready to be used for a wide range of data manipulation and analysis tasks.

---
