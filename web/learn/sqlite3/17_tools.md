---
layout: lesson
title: Tools
author: Kevin McAleer
type: page
cover: /learn/sqlite3/assets/banners/17_tools.jpg
date: 2023-07-15
previous: 16_demo_program.html
description: Desktop Apps for managing SQlite3 databases
percent: 100
duration: 4
quiz_total: 60
date_updated: 2026-11-24
navigation:
- name: Create Databases with Python and SQLite3
- content:
  - section: Overview
    content:
    - name: Course Overview
      link: 00_intro.html
  - section: Getting Started
    content:
    - name: Introduction to Databases
      link: 01_introduction_to_databases.html
    - name: Getting Started with Python and SQLite3
      link: 02_getting_started_with_sqlite3.html
    - name: Creating Tables and Schema Design
      link: 03_creating_tables_and_schema.html
    - name: Inserting and Retrieving Data
      link: 04.inserting_and_retrieving_data.html
    - name: Updating and Deleting Data
      link: 05_updating_and_deleting_data.html
  - section: Advanced topics
    content:
    - name: Querying and Joining Tables
      link: 06_querying_and_joining_tables.html
    - name: Advanced SQL Queries
      link: 07_advanced_queries.html
    - name: Database Transactions and Error Handling
      link: 08_transations_and_error_handling.html
    - name: Working with Indexes and Performance Optimization
      link: 09_indexes_and_performance.html
    - name: Securing and Backing Up SQLite Databases
      link: 10_securing_and_backing_up.html
  - section: Summary and Conclusion
    content:
    - name: Course Summary
      link: 11_summary.html
  - section: bonus
    content:
    - name: First Normal Form (1NF)
      link: 13_1nf.html
    - name: Second Normal Form (2NF)
      link: 14_2nf.html
    - name: Third Normal Form (3NF)
      link: 15_3nf.html
    - name: Boyce-Codd Normal Form (BCNF)
      link: 12_boyce_codd_normal_form.html
    - name: Demo program
      link: 16_demo_program.html
    - name: Tools
      link: 17_tools.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to open a SQLite database in a desktop tool or the command-line shell, and look inside the files your Python programs create.

## SQLite3 Tools for macOS, Windows and Linux

SQLite3 provides a variety of tools that you can use to manage and work with SQLite databases. These tools offer graphical interfaces and additional features beyond the command-line interface, making it easier to interact with databases. Let's explore how to use SQLite3 tools on macOS and Windows, including desktop tools.

### macOS

On macOS, SQLite comes with a command-line shell (`sqlite3`) by default. However, there are also desktop tools available for a more user-friendly experience:

- **DB Browser for SQLite:** This is a popular open-source desktop tool that provides a graphical interface to work with SQLite databases. You can download it from the official website: <https://sqlitebrowser.org/>

- **SQLiteStudio:** Another free, open-source database manager for SQLite, available for macOS, Windows and Linux: <https://sqlitestudio.pl/>

### Windows

On Windows, similar to macOS, SQLite3 tools can be accessed through the command-line interface. Additionally, there are several desktop tools available:

- **DB Browser for SQLite** and **SQLiteStudio:** Both are available for Windows as well, from the links above.

- **SQLite Expert Personal:** SQLite Expert offers a personal edition, a user-friendly tool with a graphical interface to manage SQLite databases. You can download it from their website: <https://www.sqliteexpert.com/>

- **Navicat for SQLite:** Navicat is a commercial database management tool that supports various databases, including SQLite. It provides an intuitive graphical interface for easy database management. You can download a trial version from their website: <https://www.navicat.com/en/products/navicat-for-sqlite>

### Linux and Raspberry Pi

Install the command-line tool and DB Browser for SQLite with:

```bash
sudo apt install sqlite3 sqlitebrowser
```

Then open a database in the terminal with `sqlite3 mydatabase.db`, or start DB Browser from the Programming menu.

By using these desktop tools, you can interact with SQLite databases in a more visual and user-friendly way. These tools often provide features such as browsing database schemas, executing SQL queries, importing/exporting data, and more.

Remember to choose a tool that suits your needs and preferences. With desktop tools, you can take advantage of their intuitive interfaces and additional functionalities to efficiently work with SQLite databases on any operating system.

---

## Try it Yourself

1. Install DB Browser for SQLite (or SQLiteStudio) and open `mydatabase.db`. Browse the `books` table.
2. Run `SELECT * FROM books WHERE year > 1900` in the tool's query tab.
3. In a terminal try `sqlite3 mydatabase.db`, then `.tables`, `.schema books`, and `.quit`.
4. Challenge: add a row in the tool, save it, and then read it from Python.

## Common Issues

- **Problem**: Your Python program says `database is locked`.
- **Solution**: In the desktop tool, save your changes (DB Browser has a Write Changes button) or close the database.
- **Why**: The tool has an open transaction on the file, and SQLite lets only one writer in at a time.

- **Problem**: The tool shows an empty database.
- **Solution**: You opened a different file. Check the folder your Python program ran from.
- **Why**: `sqlite3.connect('mydatabase.db')` creates the file in the current folder.

> **Teachers**: Installing desktop tools usually needs administrator rights on school computers. If IT can't install them, learners can still use the `sqlite3` command or read their data from Python. Navicat is a commercial product, so stick to the free tools for a class.

---

## Check your understanding

{% quiz %}
? Which command installs the shell and DB Browser on a Raspberry Pi?
- [ ] `pip install sqlite`
- [x] `sudo apt install sqlite3 sqlitebrowser`
- [ ] `python -m sqlite3`
> This is the command shown in the Linux and Raspberry Pi section.

? Why might your program say `database is locked` while a desktop tool has the file open?
- [x] The tool holds unsaved changes, so only one writer is allowed
- [ ] The file is corrupt
- [ ] SQLite is turned off
> Save or close in the tool, then run the program again.

? Which of these is a free, open-source SQLite tool?
- [ ] Navicat for SQLite
- [x] DB Browser for SQLite
> Navicat is described as commercial in the lesson.
{% endquiz %}

---
