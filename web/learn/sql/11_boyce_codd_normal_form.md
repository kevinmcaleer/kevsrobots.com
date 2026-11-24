---
layout: lesson
title: Boyce-Codd Normal Form (BCNF)
author: Kevin McAleer
type: page
cover: /learn/sql/assets/banners/11_boyce_codd_normal_form.jpg
date: 2024-11-01
previous: 10_introduction_to_normalization.html
next: 12_subqueries_and_nested_queries.html
description: Understand the stricter Boyce-Codd normal form, spot a violation in a
  workshop mentoring table, and fix it by splitting the table.
percent: 72
duration: 7
quiz_total: 46
date_updated: 2026-11-24
navigation:
- name: Getting Started with SQL
- content:
  - section: Introduction
    content:
    - name: Course Overview
      link: 00_intro.html
  - section: SQL Basics
    content:
    - name: Introduction to SQL and Relational Databases
      link: 01_introduction_to_sql.html
    - name: Setting Up Your SQL Environment
      link: 02_setting_up_environment.html
    - name: Basic SQL Commands and Syntax
      link: 03_basic_sql_commands.html
    - name: Working with Tables and Data Types
      link: 04_working_with_tables.html
    - name: Data Insertion, Modification, and Deletion
      link: 05_data_insertion_and_modification.html
    - name: Querying Data with SELECT
      link: 06_querying_data_with_select.html
    - name: Advanced Filtering and Conditional Logic
      link: 07_advanced_filtering.html
    - name: Working with Joins
      link: 08_working_with_joins.html
  - section: Intermediate Concepts
    content:
    - name: Grouping and Aggregating Data
      link: 09_grouping_and_aggregating_data.html
    - name: Introduction to Database Normalization
      link: 10_introduction_to_normalization.html
    - name: Boyce-Codd Normal Form (BCNF)
      link: 11_boyce_codd_normal_form.html
    - name: Subqueries and Nested Queries
      link: 12_subqueries_and_nested_queries.html
    - name: Views and Indexes
      link: 13_views_and_indexes.html
    - name: SQL Functions and Expressions
      link: 14_sql_functions_and_expressions.html
  - section: Conclusion
    content:
    - name: Wrap-up and Final Project
      link: 15_wrap_up_and_final_project.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to state the Boyce-Codd rule, recognise a table that breaks it, and split that table so it complies.

---

Third normal form catches most design problems. **Boyce-Codd normal form (BCNF)** closes a small gap that remains, and it appears when a table has more than one way to identify a row. It is a stricter cousin of 3NF, sometimes called "3.5NF".

## The rule

Some columns **determine** others: knowing `robot_id` tells you the robot's name. We write that as `robot_id -> name`, and call it a **functional dependency**.

> **BCNF**: whenever one set of columns determines another, the determining columns must be able to identify the whole row on their own (a **candidate key**).

In plain words: if a column can decide the value of another column, it had better be a key. Almost all well-designed tables already follow this, which is why the problem cases feel a bit contrived.

## A table that breaks the rule

The workshop runs tool sessions. Each builder books a tool, and each tool is taught by one mentor. The rules of the club:

1. A builder can book several tools, but each tool only once.
2. Each **mentor teaches exactly one tool** (Ada always does soldering, Grace always does 3D printing).
3. A tool can have more than one mentor.

```sql
CREATE TABLE tool_sessions (
    builder TEXT,
    tool    TEXT,
    mentor  TEXT,
    PRIMARY KEY (builder, tool)
);

INSERT INTO tool_sessions VALUES
    ('Kev', 'soldering',   'Ada'),
    ('Sam', 'soldering',   'Ada'),
    ('Kev', '3d printing', 'Grace'),
    ('Jo',  '3d printing', 'Grace');
```

The key is (`builder`, `tool`), and that determines the mentor. This table is in 3NF. But rule 2 gives us another dependency, `mentor -> tool`, and `mentor` alone is **not** a key, because Ada appears in several rows. That breaks BCNF.

```mermaid
flowchart LR
    A["builder + tool"] -->|"determines"| C["mentor"]
    C -->|"determines (but is not a key)"| B["tool"]
```

## Why it matters: an anomaly

Ada switches from soldering to laser cutting. We try to record that for one row:

```sql
UPDATE tool_sessions SET tool = 'laser cutting'
WHERE mentor = 'Ada' AND builder = 'Kev';

SELECT * FROM tool_sessions ORDER BY builder, tool;
```

```text
builder  tool           mentor
-------  -------------  ------
Jo       3d printing    Grace
Kev      3d printing    Grace
Kev      laser cutting  Ada
Sam      soldering      Ada
```

Now Ada teaches laser cutting for Kev and soldering for Sam, which contradicts rule 2. The table let us store a state that should be impossible, because the fact "Ada teaches X" is written in every one of her rows.

Put the data back before we fix the design:

```sql
DELETE FROM tool_sessions;
INSERT INTO tool_sessions VALUES
    ('Kev', 'soldering',   'Ada'),
    ('Sam', 'soldering',   'Ada'),
    ('Kev', '3d printing', 'Grace'),
    ('Jo',  '3d printing', 'Grace');
```

## The fix: split the table

Give the `mentor -> tool` fact its own table, where `mentor` is the key. Then keep only who booked which mentor:

```sql
CREATE TABLE mentors (
    mentor TEXT PRIMARY KEY,
    tool   TEXT NOT NULL
);
CREATE TABLE bookings (
    builder TEXT NOT NULL,
    mentor  TEXT NOT NULL REFERENCES mentors (mentor),
    PRIMARY KEY (builder, mentor)
);

INSERT INTO mentors  SELECT DISTINCT mentor, tool FROM tool_sessions;
INSERT INTO bookings SELECT builder, mentor FROM tool_sessions;
```

`INSERT ... SELECT` copies rows straight from one table into another, which is a handy way to migrate data. Check the pieces:

```sql
SELECT * FROM mentors;
```

```text
mentor  tool
------  -----------
Ada     soldering
Grace   3d printing
```

Now Ada's change is one update, in one place:

```sql
UPDATE mentors SET tool = 'laser cutting' WHERE mentor = 'Ada';

SELECT b.builder, m.tool, m.mentor
FROM bookings AS b
JOIN mentors AS m ON m.mentor = b.mentor
ORDER BY b.builder, m.tool;
```

```text
builder  tool           mentor
-------  -------------  ------
Jo       3d printing    Grace
Kev      3d printing    Grace
Kev      laser cutting  Ada
Sam      laser cutting  Ada
```

Every one of Ada's bookings now agrees. The join rebuilds the original view of the data whenever you want it.

## 3NF versus BCNF

| Question | 3NF | BCNF |
|---|-----|------|
| Rule | Non-key columns depend only on the key | Every determining column set must be a key |
| Strictness | Allows a dependency whose target is part of a key | Allows no exceptions |
| Cost | Rarely loses anything | Can lose the ability to enforce a rule with a constraint |
{:class="table table-single"}

The trade-off is in that last row. After our split, nothing stops one builder from booking two soldering mentors, whereas rule 1 said each tool only once per builder. BCNF removed a contradiction but made one rule harder to enforce. In practice, designers aim for BCNF and accept 3NF when a split would lose an important rule.

> **Teachers**: this is the most abstract lesson in the course. Try the "who determines what" game on paper first: write each rule on a card, and have pairs mark which columns decide which. Students who struggle can safely stop at 3NF and move on, as BCNF is a refinement and nothing later depends on it.

## Try it Yourself

1. Add a new mentor, `Linus`, who teaches `laser cutting`, and book Jo with him. Which tables do you change?
2. Query which builders have booked a mentor who teaches `soldering`. Use the join from above.
3. Challenge: a `courses(room, hour, teacher)` table has the rule that each teacher always uses the same room. Which dependency breaks BCNF, and how would you split the table?

## Common Issues

- **Problem**: I can't tell whether a column is a key or just unique-looking.
- **Solution**: Ask "could two rows ever share this value in the real world?" If yes, it isn't a key, however unique your sample data looks.
- **Why**: Keys are rules about the real world, not patterns in a few rows.

- **Problem**: `FOREIGN KEY constraint failed` when I insert into `bookings`.
- **Solution**: Insert into `mentors` first, as `bookings` points at it.
- **Why**: A booking can't refer to a mentor who doesn't exist yet.

- **Problem**: My split table seems to have lost data.
- **Solution**: Use `INSERT ... SELECT DISTINCT` for the lookup table, and check row counts with `COUNT(*)`.
- **Why**: Without `DISTINCT`, duplicate mentors cause a primary key clash. Without the copy, rows never arrive.

## Check your understanding

{% quiz %}
? What does BCNF require?
- [ ] Every table must have at least three columns
- [x] Whenever some columns determine another column, they must be a key of the table
- [ ] No table may have a foreign key
> That is the Boyce-Codd rule: every determinant must be a candidate key.

? In `tool_sessions`, why does `mentor -> tool` break BCNF?
- [x] `mentor` determines `tool`, but `mentor` alone is not a key, since Ada has several rows
- [ ] `tool` is not unique
- [ ] The table has a composite primary key
> The determining column isn't a key, so the fact is repeated and can contradict itself.

? What did splitting into `mentors` and `bookings` fix?
- [ ] It made the queries shorter
- [x] Each mentor's tool is stored once, so it can't conflict with itself
- [ ] It removed the need for joins
> The update anomaly is gone, though a join is now needed to see tool and builder together.
{% endquiz %}

Now let's learn to put one query inside another.
