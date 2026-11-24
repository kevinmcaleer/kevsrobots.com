---
title: Boyce-Codd Normal Form (BCNF)
description: >-
    BCNF helps reduce data duplication and ensures accurate data storage in relational databases
layout: lesson
type: page
cover: /learn/sqlite3/assets/banners/12_boyce_codd_normal_form.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to explain what Boyce-Codd Normal Form adds to 3NF, and spot a table that breaks it.

> **Aside: "The key, the whole key, and nothing but the key, so help me Codd"**
>
> This is a well-known way to remember the normal forms, in the style of a courtroom oath. In a well-designed table, every column should depend on:
>
> - **the key** (First Normal Form: each row is identified by a key, with no repeating groups)
> - **the whole key** (Second Normal Form: not just part of a composite key)
> - **and nothing but the key** (Third Normal Form: not on some other column)
>
> "So help me Codd" is a nod to Edgar F. Codd, who invented the relational model of databases in 1970.

---

Boyce-Codd Normal Form (BCNF) is a slightly stricter version of Third Normal Form. It was developed by Raymond F. Boyce and Edgar F. Codd in 1974.

The rule is: **for every dependency in the table, the column (or columns) doing the determining must be a key**. If knowing column A tells you column B, then A must be able to identify the whole row.

Most tables in 3NF are already in BCNF. The difference only shows up in tables with more than one possible key that overlap.

---

## Example

A college runs tutorials. Each tutor teaches only one subject, but a subject can have several tutors. Each student has one tutor per subject:

| Student | Subject | Tutor   |
|---------|---------|---------|
| Alice   | Maths   | Mr Khan |
| Alice   | Physics | Ms Lee  |
| Bob     | Maths   | Mr Khan |
| Charlie | Maths   | Ms Patel|
{:class="table table-single"}

The key is (Student, Subject). The table is in 3NF, but Tutor → Subject (each tutor teaches one subject), and Tutor isn't a key. So it breaks BCNF. If Mr Khan switched to teaching Physics, we'd have to update several rows, and could get it wrong.

The fix is to split it:

**Tutors**

| Tutor    | Subject |
|----------|---------|
| Mr Khan  | Maths   |
| Ms Lee   | Physics |
| Ms Patel | Maths   |
{:class="table table-single"}

**Tutorials**

| Student | Tutor    |
|---------|----------|
| Alice   | Mr Khan  |
| Alice   | Ms Lee   |
| Bob     | Mr Khan  |
| Charlie | Ms Patel |
{:class="table table-single"}

Now each tutor's subject is stored once.

> ## Don't overdo it
>
> Normalising further means more tables and more joins. For most projects, aiming for 3NF is plenty, and BCNF is worth knowing for the cases where 3NF still leaves duplication.

## Try it Yourself

1. In the tutorials table, which column determines `Subject`? Is it a key?
2. If Mr Khan switches to Physics, how many rows in the original table must change? How many in the split version?
3. Write the `CREATE TABLE` statements for `tutors` and `tutorials`, then a `JOIN` that rebuilds the original three columns.
4. Challenge: find an example from your own life (clubs, classes, parts and suppliers) where one column determines another that isn't a key.

---

## Check your understanding

{% quiz %}
? Complete the saying: "The key, the whole key, and nothing but the key..."
- [x] "...so help me Codd"
- [ ] "...so help me SQL"
> A memory aid for 1NF, 2NF and 3NF, named after Edgar F. Codd.

? In the tutorials example, why does the table break BCNF?
- [x] `Tutor` determines `Subject`, but `Tutor` is not a key
- [ ] It has more than three columns
- [ ] `Student` is not unique
> BCNF says whatever determines a column must be a key.

? For most projects, which normal form is a sensible target?
- [ ] 1NF
- [x] 3NF, with BCNF worth knowing about
- [ ] Always the highest possible
> The lesson warns that normalising further means more tables and more joins.
{% endquiz %}

---
