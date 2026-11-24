---
layout: lesson
title: Working with Files in Python
author: Kevin McAleer
type: page
cover: /learn/python/assets/banners/05_files.jpg
date: 2023-07-20
previous: 04_flow.html
next: 06_errors.html
description: Read and write text and binary files, and keep a robot's log with the
  with statement.
percent: 25
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

By the end of this lesson you will be able to write and read text files, append to a log, work with binary data, and use `with` so files are always closed.

---

Variables vanish when a program stops. Files are how a program remembers things: a robot's log, a list of settings, a high score. In this lesson we will build a tiny robot log file.

```mermaid
flowchart LR
    P["Your program"] -- "write / append" --> F[("robot_log.txt")]
    F -- "read" --> P
```

## Opening and closing files

The built-in `open()` function takes a file name and a **mode** and returns a file object. The common modes are:

| Mode | Meaning |
| --- | --- |
| `'r'` | read (the default) |
| `'w'` | write: creates the file, or empties it if it exists |
| `'a'` | append: adds to the end |
| `'b'` | binary mode, combined with another, such as `'rb'` |
| `'+'` | read and write |
{:class="table table-single"}

Always close a file when you are finished, so that your data is saved properly.

## Reading and writing text

Use `write()` to put text in a file and `read()`, `readline()` or `readlines()` to get it back. `write()` does not add a new line for you, so add `\n` yourself.

{% capture snakie_code %}
f = open("robot_log.txt", "w")
f.write("Robot started\n")
f.write("Battery 87 percent\n")
f.close()

f = open("robot_log.txt", "r")
content = f.read()
f.close()

print(content)
{% endcapture %}
{% include snakie.html code=snakie_code name="files_1.py" engine="pyodide" %}

**Common mistake:** opening with `'w'` when you meant `'a'`. Write mode wipes the existing file first, so your old log is gone. Use `'a'` to add a line to the end:

{% capture snakie_code %}
with open("robot_log.txt", "a") as f:
    f.write("Obstacle at 12 cm\n")

with open("robot_log.txt") as f:
    for line in f:
        print(line.strip())
{% endcapture %}
{% include snakie.html code=snakie_code name="files_1b.py" engine="pyodide" %}

Looping straight over the file gives you one line at a time, which also works for huge files. `strip()` removes the newline at the end of each line. Append mode creates the file if it is missing, so this works even on a first run.

## The with statement

Forgetting `close()` is easy, and an error halfway through would skip it. The `with` statement closes the file for you as soon as the indented block ends, even if something goes wrong. Use it by default.

{% capture snakie_code %}
with open("robot_log.txt", "w") as f:
    f.write("Hello, World!\n")

with open("robot_log.txt", "r") as f:
    print(f.read())
{% endcapture %}
{% include snakie.html code=snakie_code name="files_3.py" engine="pyodide" %}

## Binary files

Not every file is text. Images, sound and sensor dumps are raw bytes. Add `'b'` to the mode to read and write bytes.

{% capture snakie_code %}
data = bytes(range(5))

with open("test.bin", "wb") as f:
    f.write(data)

with open("test.bin", "rb") as f:
    data = f.read()

print(list(data))   # [0, 1, 2, 3, 4]
{% endcapture %}
{% include snakie.html code=snakie_code name="files_2.py" engine="pyodide" %}

Files you create with a bare name such as `"robot_log.txt"` appear in the folder the program runs from. In the browser Run box they live in a temporary file system and disappear when you refresh.

> **Teachers**: Files saved by browser Run boxes are temporary, so learners will not find them on the computer. To practise saving real files, have learners run the code in Thonny or a terminal, and agree where they save their work, particularly on shared or reset-on-logout machines.

---

## Try it Yourself

1. Write three lines of your own to a file called `notes.txt`, then read them back and print them.
1. Run an append example twice, then read the file. Check both lines are there.
1. Write a function `log(message)` that appends `message` plus a newline to `robot_log.txt`. Call it three times.
1. Challenge: read `robot_log.txt` and print how many lines it contains.

## Common Issues

- **Problem**: `FileNotFoundError: [Errno 2] No such file or directory`.
- **Solution**: Check the spelling, and that you are running Python from the folder that holds the file. Opening with `'w'` or `'a'` creates a missing file, `'r'` does not.
- **Why**: Relative file names are looked up from the folder the program runs in.

- **Problem**: My old data has gone after running the program again.
- **Solution**: Use mode `'a'` to add to the file. Mode `'w'` starts with an empty file every time.
- **Why**: Write mode truncates the file on open.

- **Problem**: `TypeError: write() argument must be str, not int`.
- **Solution**: Convert with `str(value)` or an f-string, such as `f.write(f"{value}\n")`.
- **Why**: Text files hold strings, not numbers.

## Check your understanding

{% quiz %}
? Which mode adds to the end of an existing file without deleting what's there?
- [ ] `'w'`
- [x] `'a'`
- [ ] `'r'`
> `'w'` empties the file first. `'a'` appends.

? Why is `with open(...) as f:` preferred?
- [ ] It's faster
- [x] It closes the file automatically, even if an error happens
- [ ] It's the only way to read binary files
> The `with` statement guarantees the file gets closed when the block ends.

? Which mode would you use to read an image file as raw bytes?
- [ ] `'r'`
- [ ] `'w'`
- [x] `'rb'`
> Adding `b` switches to binary mode, so you get bytes instead of text.
{% endquiz %}

---

## Summary

You can now save data to files and read it back, in text or binary, and `with` keeps it tidy. Next we will learn how to cope when things go wrong.
