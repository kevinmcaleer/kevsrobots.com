---
layout: lesson
title: Inserting Documents
author: Kevin McAleer
type: page
cover: /learn/mongodb/assets/banners/05_inserting_documents.jpg
date: 2026-10-18
previous: 04_first_steps_in_mongosh.html
next: 06_finding_documents.html
description: Add one part or many, meet the ObjectId, learn MongoDB's data types and
  load the full Parts Bin
percent: 30
duration: 12
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

Last lesson we put two parts in the drawer by hand. Now let's learn inserting properly: adding several documents at once, what that mysterious `_id` really is, and which kinds of values MongoDB can store. At the end of the lesson we will load the full Parts Bin, which every later lesson relies on.

Open `mongosh` and switch to the workshop first:

```javascript
use workshop
```

---

## insertOne and insertMany

You already know `insertOne`. It takes **one** document:

```javascript
db.parts.insertOne({ name: "MG996R Servo", category: "servo", quantity: 4, price: 6.99 })
```

```
{
  acknowledged: true,
  insertedId: ObjectId('6ad5245dd9ebf8421d27936d')
}
```

`insertMany` takes a **list** of documents, wrapped in square brackets `[ ]` and separated by commas:

```javascript
db.parts.insertMany([
  { name: "N20 Gear Motor", category: "motor", quantity: 8 },
  { name: "18650 Battery", category: "power", quantity: 6 }
])
```

```
{
  acknowledged: true,
  insertedIds: {
    '0': ObjectId('6ad5245dd9ebf8421d27936e'),
    '1': ObjectId('6ad5245dd9ebf8421d27936f')
  }
}
```

Notice the reply now says `insertedIds`, plural. The `'0'` and `'1'` are positions in your list (programmers count from zero), so you can tell which ID went with which part.

```javascript
db.parts.countDocuments()
```

```
5
```

Two from last lesson plus three just now. If you added an extra part in the last Try it Yourself, you will see 6.

Use `insertMany` whenever you have more than one document. It sends them all to the server in one trip, which is much quicker than lots of separate `insertOne` calls.

---

## What is inside an ObjectId?

Every document must have an `_id` field, and no two documents in a collection can share one. If you don't supply an `_id`, MongoDB creates an **ObjectId** for you.

An ObjectId looks like random gibberish, but it is 12 bytes with a structure:

| Bytes | What it holds |
|---|---|
| First 4 | The time it was created, in seconds |
| Next 5 | A random value, unique to the program that made it |
| Last 3 | A counter that goes up by one each time |
{:class="table table-single"}

That counter is why the IDs above look so similar: `...936d`, `...936e`, `...936f`.

Because the time is baked in, you can ask an ObjectId when it was made. Let's save the result of an insert in a **variable** (a named box that holds a value) called `result`. The shell prints nothing for that line, so type `result` afterwards to see it:

```javascript
const result = db.parts.insertOne({ name: "L298N Motor Driver", category: "driver", quantity: 3 })
result
```

```
{
  acknowledged: true,
  insertedId: ObjectId('6ad5245dd9ebf8421d279370')
}
```

Now dig the timestamp out of the new ID:

```javascript
result.insertedId.getTimestamp()
```

```
ISODate('2026-10-18T19:56:13.000Z')
```

That is the moment the part went in, in **UTC** (world time, shown by the `Z` at the end). A free "date added" on every document.

---

## Data types

MongoDB can store all the JSON types from [lesson 02](02_documents_and_collections), plus a proper date. Let's insert one document that uses them all:

```javascript
db.parts.insertOne({
  name: "Data Types Demo",
  quantity: 5,
  price: 4.00,
  in_stock: true,
  supplier: null,
  tags: ["demo", "test"],
  location: { shelf: "Z", bin: 9 },
  added: new Date("2026-10-18")
})
```

`findOne` is like `find` but returns just the first match, without the surrounding list. Let's read our demo back:

```javascript
db.parts.findOne({ name: "Data Types Demo" })
```

```
{
  _id: ObjectId('6ad5245dd9ebf8421d279371'),
  name: 'Data Types Demo',
  quantity: 5,
  price: 4,
  in_stock: true,
  supplier: null,
  tags: [ 'demo', 'test' ],
  location: { shelf: 'Z', bin: 9 },
  added: ISODate('2026-10-18T00:00:00.000Z')
}
```

| Field | Type | Notes |
|---|---|---|
| `name` | String | Text in quotes |
| `quantity` | Number | A whole number |
| `price` | Number | `4.00` is shown as `4` - same value, MongoDB just drops the trailing zeros |
| `in_stock` | Boolean | `true` or `false`, no quotes |
| `supplier` | Null | "Nothing here yet" |
| `tags` | Array | A list of values |
| `location` | Nested document | A document inside a document |
| `added` | Date | Made with `new Date(...)` and shown as `ISODate(...)` |
{:class="table table-single"}

Always use `new Date("YYYY-MM-DD")` for dates rather than typing them as a string like `"2026-10-18"`. A real date can be sorted and compared; a string is just text. `new Date()` with nothing inside gives you right now.

---

## Loading the Parts Bin

Time to clear out our experiments and load the real data. `drop()` deletes a whole collection, documents and all:

```javascript
db.parts.drop()
```

```
true
```

Now paste in the full Parts Bin. It is long, so copy the whole block in one go:

```javascript
use workshop

db.parts.insertMany([
  { name: "SG90 Micro Servo", category: "servo", quantity: 12, price: 2.50,
    location: { shelf: "A", bin: 1 }, tags: ["servo", "5v", "smars"],
    specs: { torque_kg_cm: 1.8, voltage: "4.8-6V" }, used_in: ["SMARS", "BurgerBot"],
    added: new Date("2026-01-12") },
  { name: "MG996R Servo", category: "servo", quantity: 4, price: 6.99,
    location: { shelf: "A", bin: 2 }, tags: ["servo", "metal gear"],
    specs: { torque_kg_cm: 11, voltage: "4.8-7.2V" }, used_in: ["Robot Arm"],
    added: new Date("2026-02-03") },
  { name: "HC-SR04 Ultrasonic Sensor", category: "sensor", quantity: 6, price: 1.80,
    location: { shelf: "B", bin: 1 }, tags: ["sensor", "distance", "5v"],
    specs: { range_cm: 400, voltage: "5V" }, used_in: ["SMARS", "BurgerBot"],
    added: new Date("2026-01-12") },
  { name: "VL53L0X Time-of-Flight Sensor", category: "sensor", quantity: 4, price: 4.99,
    location: { shelf: "B", bin: 2 }, tags: ["sensor", "distance", "i2c", "3v3"],
    specs: { range_cm: 200, voltage: "3.3V" }, used_in: [],
    added: new Date("2026-03-21") },
  { name: "Raspberry Pi Pico W", category: "board", quantity: 5, price: 6.30,
    location: { shelf: "C", bin: 1 }, tags: ["microcontroller", "wifi", "micropython"],
    specs: { ram_kb: 264, voltage: "3.3V" }, used_in: ["SMARS", "BurgerBot"],
    added: new Date("2026-01-20") },
  { name: "Raspberry Pi 5 4GB", category: "board", quantity: 1, price: 55.00,
    location: { shelf: "C", bin: 2 }, tags: ["computer", "linux"],
    specs: { ram_kb: 4194304, voltage: "5V" }, used_in: ["Robot Arm"],
    added: new Date("2026-02-14") },
  { name: "N20 Gear Motor", category: "motor", quantity: 8, price: 3.20,
    location: { shelf: "D", bin: 1 }, tags: ["motor", "smars"],
    specs: { rpm: 100, voltage: "6V" }, used_in: ["SMARS"],
    added: new Date("2026-01-12") },
  { name: "L298N Motor Driver", category: "driver", quantity: 3, price: 2.99,
    location: { shelf: "D", bin: 2 }, tags: ["motor driver", "5v"],
    specs: { channels: 2, voltage: "5-35V" }, used_in: ["BurgerBot"],
    added: new Date("2026-02-03") },
  { name: "DRV8833 Motor Driver", category: "driver", quantity: 2, price: 4.50,
    location: { shelf: "D", bin: 3 }, tags: ["motor driver", "3v3"],
    specs: { channels: 2, voltage: "2.7-10.8V" }, used_in: ["SMARS"],
    added: new Date("2026-03-21") },
  { name: "PCA9685 Servo Driver", category: "driver", quantity: 2, price: 5.99,
    location: { shelf: "D", bin: 4 }, tags: ["servo", "i2c"],
    specs: { channels: 16, voltage: "3.3-5V" }, used_in: ["Robot Arm"],
    added: new Date("2026-02-14") },
  { name: "18650 Battery", category: "power", quantity: 6, price: 3.50,
    location: { shelf: "E", bin: 1 }, tags: ["battery", "lithium"],
    specs: { capacity_mah: 2600, voltage: "3.7V" }, used_in: ["BurgerBot"],
    added: new Date("2026-01-20") },
  { name: "WS2812B LED Strip 1m", category: "lighting", quantity: 0, price: 7.50,
    location: { shelf: "E", bin: 2 }, tags: ["led", "neopixel", "5v"],
    specs: { leds: 60, voltage: "5V" }, used_in: [],
    added: new Date("2026-03-02") },
  { name: "M3 Screws (pack of 100)", category: "hardware", quantity: 3, price: 4.00,
    location: { shelf: "F", bin: 1 }, tags: ["hardware", "m3"],
    specs: { length_mm: 10 }, used_in: ["SMARS", "BurgerBot", "Robot Arm"],
    added: new Date("2026-01-12") }
])
```

```
{
  acknowledged: true,
  insertedIds: {
    '0': ObjectId('6ad5245dd9ebf8421d279372'),
    '1': ObjectId('6ad5245dd9ebf8421d279373'),
    '2': ObjectId('6ad5245dd9ebf8421d279374'),
    '3': ObjectId('6ad5245dd9ebf8421d279375'),
    '4': ObjectId('6ad5245dd9ebf8421d279376'),
    '5': ObjectId('6ad5245dd9ebf8421d279377'),
    '6': ObjectId('6ad5245dd9ebf8421d279378'),
    '7': ObjectId('6ad5245dd9ebf8421d279379'),
    '8': ObjectId('6ad5245dd9ebf8421d27937a'),
    '9': ObjectId('6ad5245dd9ebf8421d27937b'),
    '10': ObjectId('6ad5245dd9ebf8421d27937c'),
    '11': ObjectId('6ad5245dd9ebf8421d27937d'),
    '12': ObjectId('6ad5245dd9ebf8421d27937e')
  }
}
```

The `use workshop` line at the top just makes sure you are in the right place (if you already are, `mongosh` says `already on db workshop`). Check the count:

```javascript
db.parts.countDocuments()
```

```
13
```

Thirteen parts, from servos to screws. **This is the data the rest of the course uses.** If you ever mess it up, you can reset by running `db.parts.drop()` and then pasting that block again.

---

## Typos create new fields

Here is the flip side of a flexible schema. Let's add a charger board, but misspell `quantity`:

```javascript
db.parts.insertOne({ name: "TP4056 Charger Board", category: "power", quantty: 5, price: 0.80 })
```

```
{
  acknowledged: true,
  insertedId: ObjectId('6ad5245dd9ebf8421d27937f')
}
```

No error. MongoDB happily stored a brand new field called `quantty`:

```javascript
db.parts.findOne({ name: "TP4056 Charger Board" })
```

```
{
  _id: ObjectId('6ad5245dd9ebf8421d27937f'),
  name: 'TP4056 Charger Board',
  category: 'power',
  quantty: 5,
  price: 0.8
}
```

Now count the parts that have exactly 5 in stock. You can pass `countDocuments` a filter, which we will explore properly next lesson:

```javascript
db.parts.countDocuments({ quantity: 5 })
```

```
1
```

Only the Pico W. The charger board is invisible to any search on `quantity`, because as far as MongoDB knows it doesn't have one. Mistakes like this quietly hide data, so double-check field names, and in [lesson 11](11_designing_your_data) we will add rules that reject them.

Let's remove the broken document. `deleteOne` gets a full lesson of its own later, but for now:

```javascript
db.parts.deleteOne({ name: "TP4056 Charger Board" })
```

```
{ acknowledged: true, deletedCount: 1 }
```

We are back to 13 parts.

---

## Duplicate _id errors

You can choose your own `_id` instead of an ObjectId, as long as it is unique. Let's try it in a throwaway collection called `scratch`:

```javascript
db.scratch.insertOne({ _id: "servo-1", name: "SG90 Micro Servo" })
```

```
{ acknowledged: true, insertedId: 'servo-1' }
```

Do it again with the same `_id` and MongoDB refuses:

```javascript
db.scratch.insertOne({ _id: "servo-1", name: "MG996R Servo" })
```

```
Uncaught MongoServerError: E11000 duplicate key error collection: workshop.scratch index: _id_ dup key: { _id: "servo-1" }
```

`E11000` is the error code for "duplicate key", and you will see it whenever something that must be unique isn't.

### Ordered inserts

What happens if a duplicate turns up in the **middle** of an `insertMany`? Let's clear `scratch` and find out:

```javascript
db.scratch.drop()
db.scratch.insertMany([
  { _id: 1, name: "first" },
  { _id: 2, name: "second" },
  { _id: 1, name: "duplicate!" },
  { _id: 3, name: "third" }
])
```

```
Uncaught:
MongoBulkWriteError: E11000 duplicate key error collection: workshop.scratch index: _id_ dup key: { _id: 1 }
Result: BulkWriteResult {
  insertedCount: 2,
  ...
```

```javascript
db.scratch.find()
```

```
[ { _id: 1, name: 'first' }, { _id: 2, name: 'second' } ]
```

By default inserts are **ordered**: MongoDB works down the list and **stops** at the first error. The first two went in, the duplicate failed, and `third` was never even tried.

Add `{ ordered: false }` and MongoDB carries on past errors instead:

```javascript
db.scratch.drop()
db.scratch.insertMany([
  { _id: 1, name: "first" },
  { _id: 2, name: "second" },
  { _id: 1, name: "duplicate!" },
  { _id: 3, name: "third" }
], { ordered: false })
```

You still get the same error report, but this time it says `insertedCount: 3`:

```javascript
db.scratch.find()
```

```
[
  { _id: 1, name: 'first' },
  { _id: 2, name: 'second' },
  { _id: 3, name: 'third' }
]
```

Tidy up the scratch collection so only `parts` is left:

```javascript
db.scratch.drop()
```

---

## Try it Yourself

1. Use `insertMany` to add two parts of your own to `scratch`, then use `find()` to see their ObjectIds. Which characters differ between them?
2. Run `new Date()` on its own. What does it show?
3. Insert `{ _id: "servo-1" }` into `scratch` twice. What is the error code?
4. Finish by dropping `scratch` and checking `db.parts.countDocuments()` is still 13.

<details><summary>Show answers</summary>

1. Usually only the last few characters - that is the counter going up. The first part (the timestamp) is the same if they were made in the same second.
2. The current date and time in UTC, something like `ISODate('2026-10-18T19:56:13.349Z')`.
3. `E11000`, the duplicate key error.
4. `db.scratch.drop()` returns `true` and the count is `13`. If not, reset with `db.parts.drop()` and paste the Parts Bin block again.

</details>

---

## Common Issues

**Problem:** `MongoBulkWriteError: E11000 duplicate key error` when loading the Parts Bin.  
**Solution:** Run `db.parts.drop()` first, then paste the block again.  
**Why:** If you pasted it twice, or something went wrong halfway, the collection may already hold some of the documents.

**Problem:** Your count is 26, not 13.  
**Solution:** Drop and reload as above.  
**Why:** Without our own `_id`s, every paste creates 13 new documents with new ObjectIds, so pasting twice gives you two of everything.

**Problem:** After pasting, the shell sits at a `|` prompt and never runs the command.  
**Solution:** A bracket isn't closed. Press **Ctrl+C** to cancel, then copy the whole block again, right down to the final `])`.  
**Why:** `mongosh` waits for more lines until every `(`, `[` and `{` has its partner.

**Problem:** Dates sort or compare strangely.  
**Solution:** Store them with `new Date("2026-01-12")`, not as the string `"2026-01-12"`.  
**Why:** Strings are just text. Only real dates can be compared as dates.

---

## Recap

- `insertOne` adds one document; `insertMany` adds a list in one trip
- Every document has a unique `_id`, and an ObjectId has its creation time inside (`getTimestamp()`)
- MongoDB stores strings, numbers, booleans, null, arrays, nested documents and real dates
- A misspelt field name silently creates a new field, and inserts stop at the first error unless `ordered: false`
- The Parts Bin is loaded: 13 parts in `workshop.parts`

Next, we will start asking the Parts Bin questions with `find()`.
