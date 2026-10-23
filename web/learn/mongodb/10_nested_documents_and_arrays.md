---
layout: lesson
title: Nested Documents and Arrays
author: Kevin McAleer
type: page
cover: /learn/mongodb/assets/banners/10_nested_documents_and_arrays.jpg
date: 2026-10-18
previous: 09_deleting_documents.html
next: 11_designing_your_data.html
description: Query and update fields inside sub-documents and lists with dot notation,
  $all, $size, $elemMatch and the positional $ operator
percent: 55
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

One of the best things about MongoDB is that a document doesn't have to be flat. Our parts already have a document *inside* a document (`location: { shelf: "A", bin: 1 }`) and lists of values (`tags`, `used_in`). A document inside another document is called a **sub-document** or **embedded document**.

So far we've mostly looked at these from the outside. In this lesson we'll reach inside them - finding everything on shelf D, every part that runs on 5V, and which shop has a Pico W in stock.

> **Starting fresh:** lessons 08 and 09 changed some parts, so let's reset. Run `db.parts.drop()` and re-run the `insertMany` from [lesson 05](05_inserting_documents). The outputs below assume the original 13 parts.

---

## Dot notation

To reach a field inside a sub-document, join the names with a dot. `"location.shelf"` means "the `shelf` field inside `location`". What's on shelf D?

```javascript
db.parts.find({ "location.shelf": "D" }, { name: 1, location: 1, _id: 0 })
```

```
[
  { name: 'N20 Gear Motor', location: { shelf: 'D', bin: 1 } },
  { name: 'L298N Motor Driver', location: { shelf: 'D', bin: 2 } },
  { name: 'DRV8833 Motor Driver', location: { shelf: 'D', bin: 3 } },
  { name: 'PCA9685 Servo Driver', location: { shelf: 'D', bin: 4 } }
]
```

The quotes around `"location.shelf"` are **required**. Leave them out and mongosh gets confused by the dot:

```
Uncaught:
SyntaxError: Unexpected token, expected "," (1:24)

> 1 | db.parts.find({ location.shelf: "D" })
    |                         ^
```

Dot notation works with everything we've learned so far - operators, projection and sorting. Which sensors reach 300cm or more?

```javascript
db.parts.find({ "specs.range_cm": { $gte: 300 } }, { name: 1, "specs.range_cm": 1, _id: 0 })
```

```
[ { name: 'HC-SR04 Ultrasonic Sensor', specs: { range_cm: 400 } } ]
```

See how the projection `"specs.range_cm": 1` brought back just that one field from inside `specs`. And here's shelf D sorted by bin, highest first:

```javascript
db.parts.find({ "location.shelf": "D" }, { name: 1, "location.bin": 1, _id: 0 }).sort({ "location.bin": -1 })
```

```
[
  { name: 'PCA9685 Servo Driver', location: { bin: 4 } },
  { name: 'DRV8833 Motor Driver', location: { bin: 3 } },
  { name: 'L298N Motor Driver', location: { bin: 2 } },
  { name: 'N20 Gear Motor', location: { bin: 1 } }
]
```

### Why not match the whole sub-document?

You *can* match a whole sub-document:

```javascript
db.parts.find({ location: { shelf: "A", bin: 1 } }, { name: 1, _id: 0 })
```

```
[ { name: 'SG90 Micro Servo' } ]
```

But swap the order to `{ bin: 1, shelf: "A" }` and you get nothing. Matching a whole sub-document needs the exact same fields in the exact same order. Dot notation doesn't care about order, so prefer `{ "location.shelf": "A", "location.bin": 1 }`.

---

## Querying arrays

In [lesson 06](06_finding_documents) we saw that `{ tags: "5v" }` finds any part with `5v` somewhere in its list. Here are three more tools for lists.

### $all - contains all of these

`$all` matches when the list contains **every** value you give, in any order:

```javascript
db.parts.find({ tags: { $all: ["sensor", "distance"] } }, { name: 1, tags: 1, _id: 0 })
```

```
[
  {
    name: 'HC-SR04 Ultrasonic Sensor',
    tags: [ 'sensor', 'distance', '5v' ]
  },
  {
    name: 'VL53L0X Time-of-Flight Sensor',
    tags: [ 'sensor', 'distance', 'i2c', '3v3' ]
  }
]
```

Compare that with `$in` from [lesson 07](07_query_operators), which matches if the list contains **any** of the values.

### $size - how many items

`$size` matches a list of an exact length. Parts with an empty `used_in` aren't in any project yet:

```javascript
db.parts.find({ used_in: { $size: 0 } }, { name: 1, _id: 0 })
```

```
[
  { name: 'VL53L0X Time-of-Flight Sensor' },
  { name: 'WS2812B LED Strip 1m' }
]
```

And the part every robot needs:

```javascript
db.parts.find({ used_in: { $size: 3 } }, { name: 1, used_in: 1, _id: 0 })
```

```
[
  {
    name: 'M3 Screws (pack of 100)',
    used_in: [ 'SMARS', 'BurgerBot', 'Robot Arm' ]
  }
]
```

You can also pick an item by its position, counting from zero. `{ "tags.0": "sensor" }` finds parts whose **first** tag is `sensor`.

---

## Arrays of sub-documents

Lists can hold sub-documents too. Let's record where we can buy the Pico W and the SG90, with each shop's price and whether it's in stock:

```javascript
db.parts.updateOne(
  { name: "Raspberry Pi Pico W" },
  { $set: { suppliers: [
    { shop: "Pimoroni", price: 6.30, in_stock: true },
    { shop: "The Pi Hut", price: 6.60, in_stock: false }
  ] } }
)

db.parts.updateOne(
  { name: "SG90 Micro Servo" },
  { $set: { suppliers: [
    { shop: "Pimoroni", price: 3.00, in_stock: false },
    { shop: "The Pi Hut", price: 2.40, in_stock: true }
  ] } }
)
```

Dot notation reaches into every item in the list. Who sells through The Pi Hut?

```javascript
db.parts.find({ "suppliers.shop": "The Pi Hut" }, { name: 1, _id: 0 })
```

```
[ { name: 'SG90 Micro Servo' }, { name: 'Raspberry Pi Pico W' } ]
```

### The trap, and $elemMatch

Now a trickier question: which parts can I buy **in stock at Pimoroni**?

```javascript
db.parts.find({ "suppliers.shop": "Pimoroni", "suppliers.in_stock": true }, { name: 1, _id: 0 })
```

```
[ { name: 'SG90 Micro Servo' }, { name: 'Raspberry Pi Pico W' } ]
```

That's wrong! Pimoroni is out of SG90s. MongoDB checked each condition separately: "does *some* supplier say Pimoroni?" (yes) and "is *some* supplier in stock?" (yes, The Pi Hut). The two conditions matched different items.

`$elemMatch` fixes this. It says "one single item in the list must match **all** of these":

```javascript
db.parts.find(
  { suppliers: { $elemMatch: { shop: "Pimoroni", in_stock: true } } },
  { name: 1, _id: 0 }
)
```

```
[ { name: 'Raspberry Pi Pico W' } ]
```

That's the right answer. Whenever you have two or more conditions about the **same item** in a list, reach for `$elemMatch`.

---

## Updating nested fields

Dot notation works in updates too. We've moved the MG996R to bin 3:

```javascript
db.parts.updateOne({ name: "MG996R Servo" }, { $set: { "location.bin": 3 } })
db.parts.findOne({ name: "MG996R Servo" }, { name: 1, location: 1, _id: 0 })
```

```
{ name: 'MG996R Servo', location: { shelf: 'A', bin: 3 } }
```

Only `bin` changed - `shelf` is untouched. Watch what happens if we leave the dot out and set the whole sub-document:

```javascript
db.parts.updateOne({ name: "MG996R Servo" }, { $set: { location: { bin: 2 } } })
db.parts.findOne({ name: "MG996R Servo" }, { name: 1, location: 1, _id: 0 })
```

```
{ name: 'MG996R Servo', location: { bin: 2 } }
```

The shelf has vanished, because we replaced all of `location`. Let's put it back with `{ $set: { location: { shelf: "A", bin: 2 } } }`.

### The positional $ operator

The Pi Hut now has Pico Ws in stock at £6.50. We need to change one item **inside** the `suppliers` list - but which one? The positional operator `$` means "the item that matched my filter":

```javascript
db.parts.updateOne(
  { name: "Raspberry Pi Pico W", "suppliers.shop": "The Pi Hut" },
  { $set: { "suppliers.$.price": 6.50, "suppliers.$.in_stock": true } }
)
db.parts.findOne({ name: "Raspberry Pi Pico W" }, { name: 1, suppliers: 1, _id: 0 })
```

```
{
  name: 'Raspberry Pi Pico W',
  suppliers: [
    { shop: 'Pimoroni', price: 6.3, in_stock: true },
    { shop: 'The Pi Hut', price: 6.5, in_stock: true }
  ]
}
```

The filter found The Pi Hut's entry, and `suppliers.$` pointed the update at it. Pimoroni's entry wasn't touched. `$push` and `$pull` from [lesson 08](08_updating_documents) work on these lists too - `{ $pull: { suppliers: { shop: "Kitronik" } } }` removes a whole supplier entry.

After this lesson, the Pico W and SG90 have `suppliers` lists, and everything else matches the seed data.

---

## Try it Yourself

1. List everything on shelf B, sorted by bin, showing just the name and bin.
2. Find every part whose `specs.voltage` is exactly `"3.3V"`.
3. Find parts tagged with **both** `motor driver` and `5v`.
4. Pimoroni has restocked the SG90. Use the positional `$` to set that supplier's `in_stock` to `true`.

<details><summary>Show answers</summary>

1. `db.parts.find({ "location.shelf": "B" }, { name: 1, "location.bin": 1, _id: 0 }).sort({ "location.bin": 1 })` - the HC-SR04 in bin 1 and the VL53L0X in bin 2.
2. `db.parts.find({ "specs.voltage": "3.3V" }, { name: 1, _id: 0 })` - the VL53L0X and the Pico W.
3. `db.parts.find({ tags: { $all: ["motor driver", "5v"] } }, { name: 1, _id: 0 })` - just the L298N.
4. `db.parts.updateOne({ name: "SG90 Micro Servo", "suppliers.shop": "Pimoroni" }, { $set: { "suppliers.$.in_stock": true } })`

</details>

---

## Common Issues

**Problem:** `SyntaxError: Unexpected token, expected ","` on a query with a dot in it.  
**Solution:** Put quotes around the field name: `"location.shelf"`.  
**Why:** Without quotes, the shell reads the dot as JavaScript rather than part of a field name.

**Problem:** `The positional operator did not find the match needed from the query.`  
**Solution:** Include the array field in the filter, e.g. `"suppliers.shop": "The Pi Hut"`.  
**Why:** `$` points at the item your filter matched. If the filter doesn't mention the list, there's nothing for it to point at.

**Problem:** `Failed to parse $size. Expected a number`  
**Solution:** `$size` only takes an exact number, so `{ $size: { $gt: 1 } }` won't work. Use `{ "used_in.1": { $exists: true } }` to mean "has at least two items".  
**Why:** `$size` is a simple length check. Checking whether position 1 exists is a handy workaround.

**Problem:** A query on two conditions in an array of sub-documents returns too much.  
**Solution:** Wrap the conditions in `$elemMatch`.  
**Why:** Without it, each condition can be satisfied by a different item in the list.

---

## Recap

- Dot notation (`"location.shelf"`) reaches inside sub-documents - always in quotes
- `$all` needs every value in the list; `$size` checks the list's length
- Use `$elemMatch` when several conditions must all be true of the same list item
- `$set` with dot notation changes one nested field; without it you replace the whole sub-document
- The positional `$` updates the list item your filter matched

Next, we'll step back and think about **designing** our data - what to embed, what to keep in a separate collection, and how to stop typos getting in.
