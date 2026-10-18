---
layout: lesson
title: First Steps in mongosh
author: Kevin McAleer
type: page
cover: /learn/mongodb/assets/banners/04_first_steps_in_mongosh.jpg
date: 2026-10-18
previous: 03_installing_mongodb.html
next: 05_inserting_documents.html
description: Find your way around the MongoDB shell, create the workshop database
  and store your very first part
percent: 25
duration: 6
date_updated: 2026-10-18
navigation:
- name: MongoDB - Your First Database
- content:
  - section: Introduction
    content:
    - name: Introduction
      link: 00_intro.html
  - section: Database Basics
    content:
    - name: What is a Database?
      link: 01_what_is_a_database.html
    - name: Documents and Collections
      link: 02_documents_and_collections.html
    - name: Installing MongoDB
      link: 03_installing_mongodb.html
  - section: Your First Database
    content:
    - name: First Steps in mongosh
      link: 04_first_steps_in_mongosh.html
    - name: Inserting Documents
      link: 05_inserting_documents.html
    - name: Finding Documents
      link: 06_finding_documents.html
    - name: Query Operators
      link: 07_query_operators.html
  - section: Changing Data
    content:
    - name: Updating Documents
      link: 08_updating_documents.html
    - name: Deleting Documents
      link: 09_deleting_documents.html
  - section: Shaping Your Data
    content:
    - name: Nested Documents and Arrays
      link: 10_nested_documents_and_arrays.html
    - name: Designing Your Data
      link: 11_designing_your_data.html
    - name: Aggregation Basics
      link: 12_aggregation_basics.html
    - name: Indexes
      link: 13_indexes.html
  - section: MongoDB and Python
    content:
    - name: Python and PyMongo
      link: 14_python_and_pymongo.html
    - name: Building the Parts Bin App
      link: 15_building_the_parts_bin_app.html
  - section: Wrapping Up
    content:
    - name: Keeping Your Data Safe
      link: 16_keeping_your_data_safe.html
    - name: Summary and Next Steps
      link: 17_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

You have a server running and a prompt blinking at you. In this lesson we will look around, create the `workshop` database, and store our first part in it. By the end you will have used the handful of commands you will type every single time you open `mongosh`.

If your shell isn't open, start it the same way as last lesson (Docker users: `docker exec -it mongodb mongosh`).

---

## The prompt

The prompt looks like this:

```
test>
```

The word before the `>` is the database you are currently using. `mongosh` always starts in a database called `test`. You type a command after the `>`, press **Enter**, and the result appears underneath.

### A word about the syntax

`mongosh` speaks **JavaScript**, the language of web pages. You don't need to know JavaScript for this course, but a few patterns will come up again and again:

- `db.parts.find()` - the dots go from big to small: the current database (`db`), then the `parts` collection, then the `find` action
- The round brackets `()` at the end mean "do it now". Leave them off and nothing useful happens
- Anything inside `{ }` is a document, written just like the ones in [lesson 02](02_documents_and_collections)
- Text goes in quotes: `"servo"`. Numbers don't: `12`

---

## Looking around

Let's see what databases the server already has:

```javascript
show dbs
```

```
admin   40.00 KiB
config  12.00 KiB
local   40.00 KiB
```

These three are MongoDB's own housekeeping databases. Leave them alone. Your sizes may be slightly different, and Atlas users may see a different list.

---

## Creating the workshop database

To switch to a database, use `use`:

```javascript
use workshop
```

```
switched to db workshop
```

The prompt changes to `workshop>`. You can always check which database you are in by typing `db` on its own:

```javascript
db
```

```
workshop
```

Now let's look at the list again:

```javascript
show dbs
```

```
admin   40.00 KiB
config  12.00 KiB
local   40.00 KiB
```

Where has it gone? This catches everyone out. MongoDB creates databases **lazily**, meaning it doesn't actually make the database until you put something in it. `use workshop` just says "I'd like to work in a database called workshop". There are no collections yet either:

```javascript
show collections
```

This prints nothing at all, which is MongoDB's way of saying "empty".

---

## Storing your first part

Let's put a servo in the drawer. `insertOne` adds **one** document to a collection:

```javascript
db.parts.insertOne({ name: "SG90 Micro Servo", category: "servo", quantity: 12 })
```

```
{
  acknowledged: true,
  insertedId: ObjectId('6ad5238e22220cc4e52f6769')
}
```

Two things just happened. The `parts` collection didn't exist, so MongoDB created it. And because the collection now has data in it, the `workshop` database was created too. The reply tells us:

- `acknowledged: true` - the server received and saved the document
- `insertedId` - the unique `_id` MongoDB gave our servo. Yours will be a different jumble of letters and numbers

Now both show up:

```javascript
show collections
```

```
parts
```

```javascript
show dbs
```

```
admin      40.00 KiB
config     12.00 KiB
local      40.00 KiB
workshop    8.00 KiB
```

---

## Reading it back

`find()` with empty brackets returns every document in the collection:

```javascript
db.parts.find()
```

```
[
  {
    _id: ObjectId('6ad5238e22220cc4e52f6769'),
    name: 'SG90 Micro Servo',
    category: 'servo',
    quantity: 12
  }
]
```

There is our servo, with the `_id` field MongoDB added for us. The square brackets around it mean `find()` gives back a list, which just happens to have one item in it for now.

To count documents rather than show them, use `countDocuments()`:

```javascript
db.parts.countDocuments()
```

```
1
```

Let's add a second part, then count again:

```javascript
db.parts.insertOne({ name: "HC-SR04 Ultrasonic Sensor", category: "sensor", quantity: 6 })
```

```
{
  acknowledged: true,
  insertedId: ObjectId('6ad5238e22220cc4e52f676a')
}
```

```javascript
db.parts.countDocuments()
```

```
2
```

---

## Handy shell tricks

A few things that will save you a lot of typing:

| Key or command | What it does |
|---|---|
| **Up arrow** | Bring back the previous command so you can run or edit it |
| **Tab** | Auto-complete. Type `db.parts.countD` and press Tab to get `countDocuments` |
| `cls` | Clear the screen |
| `help` | List the shell's built-in commands |
| `exit` | Leave `mongosh` (`quit` works too) |
{:class="table table-single"}

When you `exit` and come back later, remember that you start in `test` again. Type `use workshop` first, and your parts will be right where you left them.

---

## Try it Yourself

1. Add a third part of your own choosing with `insertOne`, then check the count is now 3.
2. Type `use test`, then `db.parts.countDocuments()`. Why is the answer different? Switch back with `use workshop`.
3. Run `db.parts.find()` again and find the `_id` of your new part.

<details><summary>Show answers</summary>

1. For example `db.parts.insertOne({ name: "N20 Gear Motor", category: "motor", quantity: 8 })`, then `db.parts.countDocuments()` returns `3`.
2. It returns `0`, because `test` is a different database with no `parts` collection in it. `db` always means "the database I am using right now".
3. It is the `ObjectId('...')` at the top of the last document in the list.

</details>

Don't worry about tidying up these test parts. Next lesson starts by clearing the collection and loading the full Parts Bin.

---

## Common Issues

**Problem:** You type `use workshop` but `show dbs` doesn't list it.  
**Solution:** Insert a document first.  
**Why:** Databases and collections are only created when the first document is written.

**Problem:** `db.Parts.find()` returns nothing, even though you just added parts.  
**Solution:** Use the exact name, `db.parts.find()`.  
**Why:** Collection names are case sensitive. `Parts` and `parts` are two different collections.

**Problem:** `TypeError: db.parts.countDocument is not a function`  
**Solution:** Check the spelling (it is `countDocuments`, with an "s") or use **Tab** to auto-complete.  
**Why:** The shell only knows the exact names of its functions.

**Problem:** Typing `db.parts.find` prints a wall of text about a `Function`.  
**Solution:** Add the brackets: `db.parts.find()`.  
**Why:** Without brackets you are asking the shell to describe the function rather than run it.

---

## Recap

- The prompt shows your current database; `db` prints it and `use` switches it
- `show dbs` and `show collections` list what exists, and databases appear only once they hold data
- `insertOne` adds a document, `find()` shows them and `countDocuments()` counts them
- Up arrow, Tab, `cls`, `help` and `exit` make the shell much friendlier

Next, we will look properly at inserting documents and load the full Parts Bin.
