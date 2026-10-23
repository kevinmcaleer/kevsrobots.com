---
layout: lesson
title: Installing Pandas and NumPy
type: page
description: This lesson guides you through the installation process of Pandas and NumPy, setting the stage for data manipulation tasks in Python.
cover: /learn/pandas_and_numpy/assets/pandas02.png
---

![Cover photo]({{page.cover}}){:class="cover"}

## Overview

In this lesson, we will walk through the process of installing Pandas and NumPy. These libraries are essential tools in data manipulation and analysis. We'll ensure you have these tools set up correctly to start exploring the vast functionalities they offer.

---

## Prerequisites

Before installing Pandas and NumPy, make sure you have Python installed on your system. Current versions of Pandas need Python 3.10 or later. You can check your Python version by running:

```shell
python --version
```

---

## Installing Pandas and NumPy

Pandas and NumPy can be easily installed using Python's package manager, pip. If you have Python installed, pip should already be available on your system.

Run the following commands in your terminal or command prompt to install Pandas and NumPy:

```shell
pip install pandas numpy
```

This downloads and installs the latest versions of Pandas and NumPy from the Python Package Index (PyPI).

> ## On a Raspberry Pi or recent Linux
>
> If pip says `externally-managed-environment`, create a virtual environment first:
>
> ```shell
> python3 -m venv ~/data-env
> source ~/data-env/bin/activate
> pip install pandas numpy matplotlib
> ```

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

You may prefer to use a Jupyter Notebook, such as the free Google Colaboratory (Colab), to run your code. Colab already has Pandas and NumPy installed. In other notebooks you can install them by running this in a code cell:

```python
%pip install pandas numpy
```

To use Google Colab, you will need a Google account. You can access it here - <https://colab.research.google.com/>.

---

## Summary

Congratulations! You have successfully installed Pandas and NumPy. These libraries are now ready to be used for a wide range of data manipulation and analysis tasks.

---
