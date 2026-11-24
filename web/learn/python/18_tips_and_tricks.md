---
layout: lesson
title: 'Bonus: Python Tips, Tricks and Best Practices'
author: Kevin McAleer
type: page
cover: /learn/python/assets/banners/18_tips_and_tricks.jpg
date: 2023-07-20
previous: 17_apps.html
description: Write more Pythonic code, format it with Black, and find out where to
  go next after this course.
percent: 100
duration: 5
quiz_total: 58
date_updated: 2026-11-24
navigation:
- name: Python for beginners
- content:
  - section: Introduction
    content:
    - name: Introduction to Python Programming
      link: 00_intro.html
    - name: Python Basics
      link: 02_basics.html
    - name: Python Data Structures
      link: 03_data_structures.html
    - name: Control Flow in Python
      link: 04_flow.html
    - name: Working with Files in Python
      link: 05_files.html
    - name: Error Handling and Exceptions in Python
      link: 06_errors.html
    - name: Python Libraries and Modules
      link: 07_libraries.html
    - name: Python Object-Oriented Programming (OOP)
      link: 08_oop.html
    - name: Working with Data in Python
      link: 09_data.html
  - section: Real world Python
    content:
    - name: Web Scraping with Python
      link: 10_webscraping.html
    - name: Data Visualisation
      link: 11_data_visualisation.html
    - name: Machine Learning with Python
      link: 12_ml.html
    - name: Deep Learning with Python
      link: 13_deep_learning.html
    - name: Python for Automating Tasks
      link: 14_automation.html
    - name: Python and Databases
      link: 15_databases.html
    - name: Advanced Python Topics
      link: 16_advanced.html
    - name: 'Project: Building a Simple Python Application'
      link: 17_apps.html
    - name: 'Bonus: Python Tips, Tricks and Best Practices'
      link: 18_tips_and_tricks.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to write list comprehensions and use `enumerate()` and built-in functions in place of clumsy loops, format your code with Black, and choose your next course.

---

Working code is the first goal. Readable code is the second, because you will come back to it in a month and wonder what on earth you were thinking. These tips help you write code that other makers (and future you) can follow at a glance.

## Writing Pythonic code

"Pythonic" code uses Python's idioms and follows widely accepted conventions. Here are three habits that make code shorter and clearer.

**Use list comprehensions** instead of building a list with a loop:

{% capture snakie_code %}
# Non-Pythonic
squares = []
for i in range(10):
    squares.append(i * i)

# Pythonic
squares = [i * i for i in range(10)]
print(squares)

# With a filter: only the sensor readings that are too close
distances = [80, 15, 42, 9, 63]
too_close = [d for d in distances if d < 20]
print(too_close)   # [15, 9]
{% endcapture %}
{% include snakie.html code=snakie_code name="tips_comprehension.py" engine="pyodide" %}

**Loop over items directly**, and use `enumerate()` when you also need the position:

{% capture snakie_code %}
fruits = ["apple", "banana", "cherry"]

# Non-Pythonic
for i in range(len(fruits)):
    print(i, fruits[i])

# Pythonic
for i, fruit in enumerate(fruits):
    print(i, fruit)
{% endcapture %}
{% include snakie.html code=snakie_code name="tips_and_tricks.py" engine="pyodide" %}

**Use built-in functions** like `sum()`, `max()`, `min()` and `sorted()` instead of writing your own loops:

{% capture snakie_code %}
scores = [7, 3, 9]

# Non-Pythonic
total = 0
for s in scores:
    total += s

# Pythonic
total = sum(scores)
print(total, max(scores), sorted(scores))
{% endcapture %}
{% include snakie.html code=snakie_code name="tips_builtins.py" engine="pyodide" %}

## PEP 8 and the Black code formatter

`PEP 8` is Python's official style guide. It recommends things like four spaces per indentation level, lowercase names with underscores for variables and functions (`battery_level`), and whitespace that makes code visually clear. Following it means your code looks familiar to every other Python programmer.

[`Black`](https://black.readthedocs.io/en/stable/) is a code formatter that applies a consistent style automatically, so you never have to argue about spaces. Its default maximum line length is 88 characters. Install it in your virtual environment and run it on a file:

```bash
pip install black
black your_file.py
```
{: .no-snakie }

Black rewrites the file in place, so commit or back up your work first if you want to compare before and after. It is also available as an editor plug-in, which formats your code each time you save.

## Further learning resources

- [Python Documentation](https://docs.python.org/3/): the official documentation and tutorial.
- [Real Python](https://realpython.com/): tutorials, articles and other educational content.
- [PyBites](https://pybit.es/): Python code challenges and exercises.
- [Project Euler](https://projecteuler.net/): a series of challenging mathematical and programming problems.

---

## Try it Yourself

1. Rewrite this loop as a list comprehension: `doubled = []` then `for n in [1, 2, 3]: doubled.append(n * 2)`.
1. Make a list of five distances and use a comprehension to build a list of only those over 30.
1. Use `enumerate()` to print a numbered list of your robot's sensors, starting from 1 (hint: `enumerate(items, start=1)`).
1. Challenge: install Black in a virtual environment, deliberately write some untidy code with odd spacing, and watch it get cleaned up.

## Common Issues

- **Problem**: `black: command not found`.
- **Solution**: Activate the virtual environment you installed it in, or run `python -m black your_file.py`.
- **Why**: Black is installed inside the environment, so the command only exists there.

- **Problem**: A list comprehension has become long and unreadable.
- **Solution**: Switch back to a normal `for` loop.
- **Why**: Comprehensions are for short, simple jobs. Clear beats clever.

## Check your understanding

{% quiz %}
? Which is the most Pythonic way to add up a list called `scores`?
- [ ] A `while` loop with a counter
- [ ] `for i in range(len(scores))`
- [x] `sum(scores)`
> Built-in functions are shorter, clearer and usually faster.

? What does `[d for d in distances if d < 20]` build?
- [x] A list of only the distances under 20
- [ ] A single number
- [ ] A dictionary
> A list comprehension with an `if` keeps only the items that pass the test.

? What does Black do?
- [ ] Runs your program faster
- [x] Automatically formats your code in a consistent style
- [ ] Finds bugs
> Black rewrites spacing and layout, but it does not change what your code does.
{% endquiz %}

---

## Summary and what's next

Congratulations, you have gone from `print("Hello, World!")` to variables, data structures, loops, functions, files, errors, modules, classes, a tour of real-world Python and a complete application. That is a big step. Python is a skill you build by making things, so keep going.

Here is where to head next:

- [Data Manipulation with Pandas and Numpy](/learn/pandas_and_numpy/): work with real data
- [Create Databases with Python and SQLite3](/learn/sqlite3/): store your data properly
- [Mastering Pydantic for Robust Data Validation](/learn/pydantic/): make sure your data is the right shape
- [Learn MicroPython - The basics](/learn/micropython/): use your Python skills to control electronics and robots
- [Reinforcement Learning](/learn/reinforcement_learning/): teach an agent by reward
