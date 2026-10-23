---
layout: lesson
title: Query Operators
author: Kevin McAleer
type: page
cover: /learn/mongodb/assets/banners/07_query_operators.jpg
date: 2026-10-18
previous: 06_finding_documents.html
next: 08_updating_documents.html
description: Go beyond exact matches with $gt, $lt, $in, $or, $exists and $regex -
  and build low-stock and search queries
percent: 40
duration: 9
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

So far every filter has been an exact match: "category is servo", "quantity is 2". But the questions we really want to ask our Parts Bin sound more like this:

- What am I running low on?
- What costs less than three quid?
- Have I got anything with "servo" in the name?

For these we need **query operators**. An operator is a special keyword, always starting with a dollar sign `$`, that tells MongoDB *how* to compare a value rather than just "is it equal".

The pattern is always the same - instead of a plain value, you give the field a small document containing the operator:

```javascript
{ field: { $operator: value } }
```

Let's try some. This lesson uses the same 13 parts as [lesson 06](06_finding_documents).

---

## Comparison operators

| Operator | Meaning |
|---|---|
| `$eq` | equal to |
| `$ne` | not equal to |
| `$gt` | greater than |
| `$gte` | greater than or equal to |
| `$lt` | less than |
| `$lte` | less than or equal to |
{:class="table table-single"}

Which parts do I have more than five of?

```javascript
db.parts.find({ quantity: { $gt: 5 } }, { name: 1, quantity: 1, _id: 0 })
```

```
[
  { name: 'SG90 Micro Servo', quantity: 12 },
  { name: 'HC-SR04 Ultrasonic Sensor', quantity: 6 },
  { name: 'N20 Gear Motor', quantity: 8 },
  { name: '18650 Battery', quantity: 6 }
]
```

What costs £3 or less?

```javascript
db.parts.find({ price: { $lte: 3 } }, { name: 1, price: 1, _id: 0 })
```

```
[
  { name: 'SG90 Micro Servo', price: 2.5 },
  { name: 'HC-SR04 Ultrasonic Sensor', price: 1.8 },
  { name: 'L298N Motor Driver', price: 2.99 }
]
```

You can put two operators on the same field to make a range. Everything from £4 up to (but not including) £7:

```javascript
db.parts.find({ price: { $gte: 4, $lt: 7 } }, { name: 1, price: 1, _id: 0 })
```

```
[
  { name: 'MG996R Servo', price: 6.99 },
  { name: 'VL53L0X Time-of-Flight Sensor', price: 4.99 },
  { name: 'Raspberry Pi Pico W', price: 6.3 },
  { name: 'DRV8833 Motor Driver', price: 4.5 },
  { name: 'PCA9685 Servo Driver', price: 5.99 },
  { name: 'M3 Screws (pack of 100)', price: 4 }
]
```

`$ne` flips a match around - `{ category: { $ne: "driver" } }` returns the ten parts that aren't drivers. `$eq` does the same as a plain value, so `{ category: { $eq: "power" } }` and `{ category: "power" }` are identical. You'll rarely need it.

---

## One of these: $in and $nin

`$in` takes a list and matches if the field equals **any** of the values in it:

```javascript
db.parts.find({ category: { $in: ["servo", "motor"] } }, { name: 1, category: 1, _id: 0 })
```

```
[
  { name: 'SG90 Micro Servo', category: 'servo' },
  { name: 'MG996R Servo', category: 'servo' },
  { name: 'N20 Gear Motor', category: 'motor' }
]
```

`$nin` ("not in") is the opposite. It works on array fields too - this finds anything tagged `i2c` **or** `wifi`:

```javascript
db.parts.find({ tags: { $in: ["i2c", "wifi"] } }, { name: 1, tags: 1, _id: 0 })
```

```
[
  {
    name: 'VL53L0X Time-of-Flight Sensor',
    tags: [ 'sensor', 'distance', 'i2c', '3v3' ]
  },
  {
    name: 'Raspberry Pi Pico W',
    tags: [ 'microcontroller', 'wifi', 'micropython' ]
  },
  { name: 'PCA9685 Servo Driver', tags: [ 'servo', 'i2c' ] }
]
```

---

## Combining conditions: $and, $or, $not

We already know that putting several fields in a filter means AND. For OR, we need `$or`, which takes a **list** of filters. A document matches if any one of them is true. Boards, or anything over £7:

```javascript
db.parts.find(
  { $or: [ { category: "board" }, { price: { $gt: 7 } } ] },
  { name: 1, price: 1, _id: 0 }
)
```

```
[
  { name: 'Raspberry Pi Pico W', price: 6.3 },
  { name: 'Raspberry Pi 5 4GB', price: 55 },
  { name: 'WS2812B LED Strip 1m', price: 7.5 }
]
```

You can spread a command over several lines like this - mongosh waits until the brackets are all closed before running it.

`$and` also exists, and these two queries are identical:

```javascript
db.parts.find({ $and: [ { category: "sensor" }, { quantity: { $gte: 5 } } ] })
db.parts.find({ category: "sensor", quantity: { $gte: 5 } })
```

Both return just the HC-SR04. Use the short form normally, and reach for `$and` when you need to combine two `$or`s.

`$not` wraps another operator and reverses it. `{ quantity: { $not: { $gt: 3 } } }` means "quantity is not more than 3".

---

## Does the field exist? $exists

Because MongoDB doesn't force every document to have the same fields, it's handy to ask "which documents have this field at all?" Only the boards have a `ram_kb` spec:

```javascript
db.parts.find({ "specs.ram_kb": { $exists: true } }, { name: 1, _id: 0 })
```

```
[ { name: 'Raspberry Pi Pico W' }, { name: 'Raspberry Pi 5 4GB' } ]
```

The dot in `"specs.ram_kb"` means "the `ram_kb` field inside `specs`" - we'll cover that properly in [lesson 10](10_nested_documents_and_arrays). `$exists: false` finds documents that are *missing* a field, which is great for spotting incomplete records.

---

## Searching text: $regex

A **regular expression** (regex for short) is a pattern for matching text. You can get very fancy, but the simplest use is "does this text contain this word?"

```javascript
db.parts.find({ name: { $regex: "servo" } }, { name: 1, _id: 0 })
```

Nothing comes back! Our names say "Servo" with a capital S, and regex is case sensitive by default. Add the `i` option for case-**i**nsensitive:

```javascript
db.parts.find({ name: { $regex: "servo", $options: "i" } }, { name: 1, _id: 0 })
```

```
[
  { name: 'SG90 Micro Servo' },
  { name: 'MG996R Servo' },
  { name: 'PCA9685 Servo Driver' }
]
```

mongosh also understands a shorter form, with the pattern between slashes:

```javascript
db.parts.find({ name: /servo/i }, { name: 1, _id: 0 })
```

Two extra symbols are worth knowing: `^` means "starts with" and `$` means "ends with". So `/^raspberry/i` finds both Pis and `/driver$/i` finds the three drivers.

---

## Putting it to work: stock checks

Now we can answer the question every maker dreads. What's running low? Let's say anything with fewer than 3 in stock, lowest first, with its shelf so we can go and check:

```javascript
db.parts.find(
  { quantity: { $lt: 3 } },
  { name: 1, quantity: 1, location: 1, _id: 0 }
).sort({ quantity: 1 })
```

```
[
  {
    name: 'WS2812B LED Strip 1m',
    quantity: 0,
    location: { shelf: 'E', bin: 2 }
  },
  {
    name: 'Raspberry Pi 5 4GB',
    quantity: 1,
    location: { shelf: 'C', bin: 2 }
  },
  {
    name: 'DRV8833 Motor Driver',
    quantity: 2,
    location: { shelf: 'D', bin: 3 }
  },
  {
    name: 'PCA9685 Servo Driver',
    quantity: 2,
    location: { shelf: 'D', bin: 4 }
  }
]
```

And what's completely out of stock? That one doesn't need an operator at all:

```javascript
db.parts.find({ quantity: 0 }, { name: 1, _id: 0 })
```

```
[ { name: 'WS2812B LED Strip 1m' } ]
```

Time to order some LEDs. Operators work with everything from the last lesson, so `countDocuments({ quantity: { $lt: 3 } })` gives `4`, and `.sort().limit()` gives you a "top three" of anything. We'll use exactly these queries in the Python app in [lesson 15](15_building_the_parts_bin_app).

---

## Try it Yourself

1. Find every part priced from £3 to £5 inclusive, cheapest first.
2. Find parts tagged `5v` **or** `3v3` that you have fewer than 5 of.
3. Find any part with "sensor" in its name, whatever the capitalisation.
4. Find parts that are **not** servos or boards and cost more than £4.

<details><summary>Show answers</summary>

1. `db.parts.find({ price: { $gte: 3, $lte: 5 } }, { name: 1, price: 1, _id: 0 }).sort({ price: 1 })` - the N20 motor, 18650 battery, M3 screws, DRV8833 and VL53L0X.
2. `db.parts.find({ tags: { $in: ["5v", "3v3"] }, quantity: { $lt: 5 } }, { name: 1, quantity: 1, _id: 0 })` - the VL53L0X, L298N, DRV8833 and the LED strip.
3. `db.parts.find({ name: /sensor/i }, { name: 1, _id: 0 })` - the HC-SR04 and the VL53L0X.
4. `db.parts.find({ category: { $nin: ["servo", "board"] }, price: { $gt: 4 } }, { name: 1, price: 1, _id: 0 })` - the VL53L0X, DRV8833, PCA9685 and the LED strip.

</details>

---

## Common Issues

**Problem:** A query with `{ quantity: { gt: 5 } }` returns nothing, with no error.  
**Solution:** Don't forget the dollar sign: `$gt`.  
**Why:** Without the `$`, MongoDB thinks you want `quantity` to be the exact document `{ gt: 5 }`. No part matches that, so you get nothing back.

**Problem:** `MongoServerError[BadValue]: $or argument must be an array`  
**Solution:** Wrap the conditions in square brackets: `{ $or: [ {...}, {...} ] }`.  
**Why:** `$or`, `$and` and `$in` all take a list.

**Problem:** `Regular expression is invalid: missing closing parenthesis` when searching for "(pack".  
**Solution:** Put a backslash before special characters: `/\(pack/`.  
**Why:** Characters like `( ) . * + ? [ ]` have special meanings in a regex.

**Problem:** I wrote `{ category: "sensor", category: "board" }` and only got boards.  
**Solution:** Use `$in` instead: `{ category: { $in: ["sensor", "board"] } }`.  
**Why:** A field can only appear once in a document, so the second `category` quietly replaces the first.

---

## Recap

- Operators start with `$` and go inside a small document: `{ quantity: { $lt: 3 } }`
- `$gt`, `$gte`, `$lt`, `$lte`, `$ne` compare values; `$in` and `$nin` check against a list
- `$or` takes a list of filters; several fields in one filter already means AND
- `$exists` checks whether a field is there at all
- `$regex` with the `i` option (or `/word/i`) gives you a simple case-insensitive search

Next, we'll stop just looking at our data and start **changing** it - updating stock levels, adding tags and more.
