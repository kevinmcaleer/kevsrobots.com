---
layout: lesson
title: Updating Documents
author: Kevin McAleer
type: page
cover: /learn/mongodb/assets/banners/08_updating_documents.jpg
date: 2026-10-18
previous: 07_query_operators.html
next: 09_deleting_documents.html
description: Change data in place with updateOne and updateMany - $set, $unset, $inc,
  array operators, upserts and the mistakes to avoid
percent: 45
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

A parts database that never changes isn't much use. Every time we pull a servo out of a bin, buy a new reel of LEDs or decide a part belongs in another project, the Parts Bin needs to know.

In this lesson we'll **update** documents - change them in place, without deleting and re-adding them.

> **Starting fresh:** the outputs below assume the original 13 parts. If yours have drifted, run `db.parts.drop()` and re-run the `insertMany` from [lesson 05](05_inserting_documents).

---

## updateOne and $set

Every update has two parts:

1. A **filter** - exactly like the ones from `find()` - saying *which* document to change
2. An **update** - saying *what* to change, using an update operator

The most common update operator is `$set`, which sets a field to a new value. Let's say we've counted the SG90 bin and there are actually 10, not 12:

```javascript
db.parts.updateOne({ name: "SG90 Micro Servo" }, { $set: { quantity: 10 } })
```

```
{
  acknowledged: true,
  insertedId: null,
  matchedCount: 1,
  modifiedCount: 1,
  upsertedCount: 0
}
```

That reply is worth reading:

- **matchedCount** - how many documents the filter found
- **modifiedCount** - how many were actually changed

Run exactly the same command again:

```
{
  acknowledged: true,
  insertedId: null,
  matchedCount: 1,
  modifiedCount: 0,
  upsertedCount: 0
}
```

It found the servo, but the quantity was already 10, so there was nothing to change. If you ever see `matchedCount: 0`, your filter didn't find anything - usually a typo in the name.

`$set` can change several fields at once, and if a field doesn't exist yet, it gets created:

```javascript
db.parts.updateOne(
  { name: "SG90 Micro Servo" },
  { $set: { notes: "Plastic gears - strip easily", price: 2.75 } }
)
db.parts.findOne({ name: "SG90 Micro Servo" }, { name: 1, price: 1, notes: 1, _id: 0 })
```

```
{
  name: 'SG90 Micro Servo',
  price: 2.75,
  notes: 'Plastic gears - strip easily'
}
```

---

## Removing a field with $unset

Changed your mind about that note? `$unset` removes a field completely. The value you give it doesn't matter - an empty string is the convention:

```javascript
db.parts.updateOne({ name: "SG90 Micro Servo" }, { $unset: { notes: "" } })
db.parts.findOne({ name: "SG90 Micro Servo" }, { name: 1, price: 1, notes: 1, _id: 0 })
```

```
{ name: 'SG90 Micro Servo', price: 2.75 }
```

---

## Counting up and down with $inc

Here's the one we'll use most. `$inc` **inc**reases a number by an amount - and a negative amount decreases it. We've just used two SG90s on a build:

```javascript
db.parts.updateOne({ name: "SG90 Micro Servo" }, { $inc: { quantity: -2 } })
db.parts.findOne({ name: "SG90 Micro Servo" }, { name: 1, quantity: 1, _id: 0 })
```

```
{ name: 'SG90 Micro Servo', quantity: 8 }
```

Why not just `$set` it to 8? Because `$inc` doesn't need to know the old value. If two people (or two programs) use a servo at the same moment, both decrements count. With `$set`, one would overwrite the other.

The LED strip finally arrived - five of them:

```javascript
db.parts.updateOne({ name: "WS2812B LED Strip 1m" }, { $inc: { quantity: 5 } })
```

Here's a neat trick: put a condition in the filter so you can't go below zero. We only have one Raspberry Pi 5, so trying to use two does nothing:

```javascript
db.parts.updateOne(
  { name: "Raspberry Pi 5 4GB", quantity: { $gte: 2 } },
  { $inc: { quantity: -2 } }
)
```

```
{
  acknowledged: true,
  insertedId: null,
  matchedCount: 0,
  modifiedCount: 0,
  upsertedCount: 0
}
```

`matchedCount: 0` tells us "not enough in stock". Our Python app will use exactly this in lesson 15.

---

## Updating arrays: $push, $addToSet, $pull

Our `used_in` and `tags` fields are lists, and they have their own operators. `$push` adds an item to the end. We've started using SG90s in the Robot Arm:

```javascript
db.parts.updateOne({ name: "SG90 Micro Servo" }, { $push: { used_in: "Robot Arm" } })
```

The catch: run it twice and you get two copies.

```
{
  name: 'SG90 Micro Servo',
  used_in: [ 'SMARS', 'BurgerBot', 'Robot Arm', 'Robot Arm' ]
}
```

`$pull` removes **every** matching item from a list:

```javascript
db.parts.updateOne({ name: "SG90 Micro Servo" }, { $pull: { used_in: "Robot Arm" } })
```

```
{ name: 'SG90 Micro Servo', used_in: [ 'SMARS', 'BurgerBot' ] }
```

For lists where each item should only appear once - like tags and project names - use `$addToSet` instead. It only adds the value if it isn't already there:

```javascript
db.parts.updateOne({ name: "SG90 Micro Servo" }, { $addToSet: { used_in: "Robot Arm" } })
```

Run that twice and the second time you'll see `modifiedCount: 0`. The list ends up as `[ 'SMARS', 'BurgerBot', 'Robot Arm' ]`, with no duplicate.

---

## Changing lots of documents: updateMany

`updateOne` changes the **first** match only. `updateMany` changes **every** match. Let's tag both servos as `pwm` (the kind of signal they use):

```javascript
db.parts.updateMany({ category: "servo" }, { $addToSet: { tags: "pwm" } })
```

```
{
  acknowledged: true,
  insertedId: null,
  matchedCount: 2,
  modifiedCount: 2,
  upsertedCount: 0
}
```

### Recording when something changed

It's useful to stamp a document with the date it was last touched. `$currentDate` sets a field to the server's current date and time:

```javascript
db.parts.updateMany({ category: "driver" }, { $currentDate: { last_updated: true } })
db.parts.find({ category: "driver" }, { name: 1, last_updated: 1, _id: 0 })
```

```
[
  {
    name: 'L298N Motor Driver',
    last_updated: ISODate('2026-10-18T19:52:59.793Z')
  },
  {
    name: 'DRV8833 Motor Driver',
    last_updated: ISODate('2026-10-18T19:52:59.793Z')
  },
  {
    name: 'PCA9685 Servo Driver',
    last_updated: ISODate('2026-10-18T19:52:59.793Z')
  }
]
```

You can do the same with `$set` and `new Date()`, which is handy when you're changing something else at the same time:

```javascript
db.parts.updateOne(
  { name: "N20 Gear Motor" },
  { $set: { quantity: 6, last_updated: new Date() } }
)
```

---

## Upsert: update it, or create it

What if you update a part that doesn't exist yet?

```javascript
db.parts.updateOne({ name: "Arduino Uno" }, { $inc: { quantity: 1 } })
```

You get `matchedCount: 0` and nothing happens. Sometimes, though, you want "update it if it's there, add it if it isn't". That's called an **upsert** (update + insert), and you switch it on with a third argument:

```javascript
db.parts.updateOne(
  { name: "TT Gear Motor" },
  { $set: { category: "motor", quantity: 4, price: 1.50 } },
  { upsert: true }
)
```

```
{
  acknowledged: true,
  insertedId: ObjectId('6ad5239be10decb533e9ce83'),
  matchedCount: 0,
  modifiedCount: 0,
  upsertedCount: 1
}
```

`upsertedCount: 1` means a new document was created. It took its `name` from the filter and the rest from `$set`. Run the same command again and it matches the existing document instead of making a second one.

---

## replaceOne, and why it bites

`replaceOne` swaps a whole document for a new one. Everything except `_id` is thrown away:

```javascript
db.parts.replaceOne({ name: "TT Gear Motor" }, { quantity: 6 })
db.parts.findOne({ quantity: 6, name: { $exists: false } })
```

```
{ _id: ObjectId('6ad5239be10decb533e9ce83'), quantity: 6 }
```

Our motor has lost its name, category and price. That's almost never what you meant. Let's put it right by replacing it with a complete document:

```javascript
db.parts.replaceOne(
  { quantity: 6, name: { $exists: false } },
  { name: "TT Gear Motor", category: "motor", quantity: 6, price: 1.50 }
)
```

Use `updateOne` with `$set` for day-to-day changes, and save `replaceOne` for when you really do want to start a document over.

---

## Your data now

After following along, your Parts Bin differs from the original seed data like this:

- SG90: quantity 8, price 2.75, used in Robot Arm too, tagged `pwm`
- MG996R: tagged `pwm`
- LED strip: quantity 5 (back in stock!)
- N20 motor: quantity 6
- Drivers and the N20 have a `last_updated` date
- A new 14th part, the TT Gear Motor

That's fine for the next lesson, where we'll delete the TT motor again.

---

## Try it Yourself

1. You've used one HC-SR04 sensor. Take it off the stock count.
2. Add the tag `favourite` to the Pico W. Run it twice - what's the `modifiedCount` the second time?
3. Add `BurgerBot` to the `used_in` list of every driver. Why are `matchedCount` and `modifiedCount` different?
4. Remove the `smars` tag from every part that has it.

<details><summary>Show answers</summary>

1. `db.parts.updateOne({ name: "HC-SR04 Ultrasonic Sensor" }, { $inc: { quantity: -1 } })` - it goes from 6 to 5.
2. `db.parts.updateOne({ name: "Raspberry Pi Pico W" }, { $addToSet: { tags: "favourite" } })` - the second run gives `modifiedCount: 0` because the tag is already there.
3. `db.parts.updateMany({ category: "driver" }, { $addToSet: { used_in: "BurgerBot" } })` gives `matchedCount: 3, modifiedCount: 2`. The L298N was already used in BurgerBot, so it didn't change.
4. `db.parts.updateMany({ tags: "smars" }, { $pull: { tags: "smars" } })` - two parts changed (the SG90 and the N20 motor).

</details>

---

## Common Issues

**Problem:** `MongoInvalidArgumentError: Update document requires atomic operators`  
**Solution:** You forgot `$set`. Write `{ $set: { quantity: 6 } }`, not `{ quantity: 6 }`.  
**Why:** `updateOne` needs an operator so it knows what to do with the value. mongosh stops you here because the operator-free version means "replace", which is what `replaceOne` is for.

**Problem:** `Cannot apply $inc to a value of non-numeric type ... has the field 'quantity' of non-numeric type string`  
**Solution:** Fix the value with `$set: { quantity: 8 }` (no quotes), then `$inc` will work.  
**Why:** Somewhere a quantity was saved as the text `"8"` instead of the number `8`. You can't add 1 to text.

**Problem:** `matchedCount: 0` when you're sure the part exists.  
**Solution:** Run the same filter with `find()` first to check it.  
**Why:** The filter is an exact match - a missing capital letter or extra space means no match.

**Problem:** Duplicate items keep appearing in `tags` or `used_in`.  
**Solution:** Use `$addToSet` instead of `$push`, and `$pull` to clean up the copies.  
**Why:** `$push` always adds, even if the item is already in the list.

---

## Recap

- `updateOne` changes the first match, `updateMany` changes them all - both take a filter and an update
- `$set` changes or adds fields, `$unset` removes them, `$inc` adds or subtracts
- `$addToSet` and `$pull` keep lists tidy; `$push` allows duplicates
- Check `matchedCount` and `modifiedCount` to see what really happened
- `upsert: true` creates the document if it doesn't exist; `replaceOne` throws away every field you don't include

Next, we'll learn how to **delete** documents - and how to avoid deleting everything by accident.
