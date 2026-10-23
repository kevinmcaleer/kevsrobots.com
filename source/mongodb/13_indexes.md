---
title: Indexes
description: Learn what an index is, how to see whether MongoDB uses one with explain(), and how to create single, compound, unique and text indexes for your Parts Bin.
layout: lesson
type: page
cover: /learn/mongodb/assets/banners/13_indexes.jpg
date_updated: 2026-10-18
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Every query we've run so far has been instant. That's because our Parts Bin holds 13 documents. Point the same queries at a million sensor readings and things get sluggish - unless you've built an **index**.

In this lesson we'll see how MongoDB finds documents, measure how much work it's doing, and make it do far less.

Let's start from the original seed data. Run `db.parts.drop()`, then re-run the `insertMany` from [lesson 05](05_inserting_documents).

---

## What is an index?

Think about a big electronics textbook. If you want the page about the L298N, you could start at page 1 and read every page until you find it. Or you could flip to the index at the back, look up "L298N" in the alphabetical list, and go straight to page 214.

A MongoDB index is exactly that. It's a separate, sorted list of the values in one field, each pointing at the documents that hold it. Without one, MongoDB reads every document in the collection - a **collection scan**. With one, it looks up the value in the index and fetches only the documents it needs - an **index scan**.

Every collection gets one index for free, on `_id`:

```javascript
db.parts.getIndexes()
```

```
[ { v: 2, key: { _id: 1 }, name: '_id_' } ]
```

That's why looking up a document by `_id` is always fast.

---

## Seeing the work with `explain()`

Add `.explain("executionStats")` to the end of a query and MongoDB tells you *how* it ran it, instead of giving you the results:

```javascript
db.parts.find({ category: "sensor" }).explain("executionStats")
```

A big document comes back. Here are the interesting bits, trimmed from the real output:

```
  winningPlan: {
    isCached: false,
    stage: 'COLLSCAN',
    filter: { category: { '$eq': 'sensor' } },
    direction: 'forward'
  },
  ...
  executionStats: {
    executionSuccess: true,
    nReturned: 2,
    executionTimeMillis: 0,
    totalKeysExamined: 0,
    totalDocsExamined: 13,
    ...
```

- `stage: 'COLLSCAN'` - a collection scan. MongoDB read the whole book.
- `nReturned: 2` - it found the two sensors.
- `totalDocsExamined: 13` - but it had to look at all 13 parts to find them.

Reading 13 to return 2 isn't a problem today. Reading 13 million to return 2 would be.

---

## Creating an index

Let's index the `category` field. The `1` means sort the index in ascending (A to Z) order; `-1` would be descending:

```javascript
db.parts.createIndex({ category: 1 })
```

```
category_1
```

MongoDB names the index after its field and direction. Now run the same explain again:

```javascript
db.parts.find({ category: "sensor" }).explain("executionStats")
```

```
  winningPlan: {
    isCached: false,
    stage: 'FETCH',
    inputStage: {
      stage: 'IXSCAN',
      keyPattern: { category: 1 },
      indexName: 'category_1',
      ...
  executionStats: {
    executionSuccess: true,
    nReturned: 2,
    executionTimeMillis: 0,
    totalKeysExamined: 2,
    totalDocsExamined: 2,
    ...
```

Now the plan has two stages. `IXSCAN` looked up "sensor" in the index, then `FETCH` collected just those documents. Two documents examined to return two - no wasted effort.

---

## Making the difference visible

With 13 documents both versions take 0 milliseconds, so let's make a scratch collection big enough to feel it. This loop builds 100,000 fake sensor readings from 500 imaginary sensors - think of a weather station logging every second:

```javascript
const docs = []
for (let i = 0; i < 100000; i++) {
  docs.push({ sensor: "sensor-" + (i % 500), value: Math.round(Math.random() * 1000) / 10, at: new Date(Date.now() - i * 1000) })
}
```

```
100000
```

```javascript
const result = db.readings.insertMany(docs)
db.readings.countDocuments()
```

```
100000
```

(We store the insert result in `result` so mongosh doesn't print 100,000 ObjectIds at you.) Now find every reading from one sensor. We can reach into the explain output with dots to pick out just the numbers we want:

```javascript
db.readings.find({ sensor: "sensor-42" }).explain("executionStats").executionStats.totalDocsExamined
```

```
100000
```

```javascript
db.readings.find({ sensor: "sensor-42" }).explain("executionStats").executionStats.executionTimeMillis
```

```
23
```

All 100,000 documents read to find 200. Add an index and try again:

```javascript
db.readings.createIndex({ sensor: 1 })
db.readings.find({ sensor: "sensor-42" }).explain("executionStats").executionStats.totalDocsExamined
```

```
200
```

```javascript
db.readings.find({ sensor: "sensor-42" }).explain("executionStats").executionStats.executionTimeMillis
```

```
1
```

From 23ms down to 1ms - your exact times will differ, but the gap will be similar, and it grows as the collection does. Tidy up the scratch collection when you're done:

```javascript
db.readings.drop()
```

---

## Compound indexes

A **compound index** covers more than one field. Our low-stock check often asks "which drivers are running low?":

```javascript
db.parts.createIndex({ category: 1, quantity: 1 })
```

```
category_1_quantity_1
```

```javascript
db.parts.find({ category: "driver", quantity: { $lt: 3 } }, { _id: 0, name: 1, quantity: 1 })
```

```
[
  { name: 'DRV8833 Motor Driver', quantity: 2 },
  { name: 'PCA9685 Servo Driver', quantity: 2 }
]
```

With only the `category` index, MongoDB would fetch all 3 drivers and then check each quantity. The compound index is sorted by category and *then* by quantity, so it goes straight to the 2 that match.

Order matters. A compound index can also answer queries on its *first* field alone, like a phone book sorted by surname then first name still helps you find every Smith. But it can't help with the second field alone - the phone book is no use for finding everyone called Dave. That means our `category_1` index is now redundant, so let's remove it:

```javascript
db.parts.dropIndex("category_1")
```

```
{ nIndexesWas: 3, ok: 1 }
```

---

## Unique indexes

An index can also enforce a rule. A **unique index** refuses any document whose value is already in the index. Every part name in our workshop should appear only once:

```javascript
db.parts.createIndex({ name: 1 }, { unique: true })
```

```
name_1
```

Now try adding a second SG90:

```javascript
db.parts.insertOne({ name: "SG90 Micro Servo", category: "servo", quantity: 10, price: 2.20 })
```

```
Uncaught MongoServerError: E11000 duplicate key error collection: workshop.parts index: name_1 dup key: { name: "SG90 Micro Servo" }
```

E11000 is MongoDB's code for "duplicate key". If you'd bought more SG90s, the right move is a `$inc` on the existing part, as in [lesson 08](08_updating_documents).

---

## Text indexes

In [lesson 07](07_query_operators) we searched names with `$regex`. A **text index** is built for word searches instead. It splits text into words and understands simple plurals:

```javascript
db.parts.createIndex({ name: "text", tags: "text" })
```

```
name_text_tags_text
```

Search it with `$text`:

```javascript
db.parts.find({ $text: { $search: "servos" } }, { _id: 0, name: 1 })
```

```
[
  { name: 'MG996R Servo' },
  { name: 'PCA9685 Servo Driver' },
  { name: 'SG90 Micro Servo' }
]
```

We searched for "servos" and it matched "Servo". Several words match *any* of them, and wrapping a phrase in escaped quotes matches it exactly:

```javascript
db.parts.find({ $text: { $search: "\"motor driver\"" } }, { _id: 0, name: 1 })
```

```
[ { name: 'DRV8833 Motor Driver' }, { name: 'L298N Motor Driver' } ]
```

Text search matches whole words, not fragments - `"serv"` finds nothing. A collection can only have one text index, so list every field you want searchable in it.

---

## Don't index everything

If indexes make reads faster, why not index every field? Because every index has a cost:

- **Writes get slower.** Every insert, update and delete has to update every index too.
- **They take space.** That small `sensor` index on our readings took about 900KB, and MongoDB works best when indexes fit in memory.

A good rule: index the fields you filter and sort by often, and leave the rest. Use `explain()` to check before and after.

Here's where your indexes stand now:

```javascript
db.parts.getIndexes()
```

```
[
  { v: 2, key: { _id: 1 }, name: '_id_' },
  {
    v: 2,
    key: { category: 1, quantity: 1 },
    name: 'category_1_quantity_1'
  },
  { v: 2, key: { name: 1 }, name: 'name_1', unique: true },
  {
    v: 2,
    key: { _fts: 'text', _ftsx: 1 },
    name: 'name_text_tags_text',
    weights: { name: 1, tags: 1 },
    default_language: 'english',
    language_override: 'language',
    textIndexVersion: 3
  }
]
```

Keep these. The unique index on `name` will protect our Parts Bin app from duplicates later in the course. Your parts data itself is unchanged.

---

## Try it Yourself

1. Create an index on the nested field `location.shelf` and use `explain()` to prove that `find({ "location.shelf": "D" })` uses it.
2. Run `explain()` on `db.parts.find({ quantity: { $lt: 3 } })`. Does our compound index help? Why?
3. Use the text index to find which part is tagged `lithium`.

<details><summary>Show answers</summary>

1. Dot notation works in indexes too:

```javascript
db.parts.createIndex({ "location.shelf": 1 })
db.parts.find({ "location.shelf": "D" }).explain().queryPlanner.winningPlan.inputStage.indexName
```

```
location.shelf_1
```

`totalDocsExamined` is 4 - exactly the four parts on shelf D. Remove it afterwards with `db.parts.dropIndex({ "location.shelf": 1 })` - you can drop by name or by the same key pattern you created it with.

2. No - the winning plan's stage is `COLLSCAN`. The compound index is sorted by `category` first, so it can't help a query that only mentions `quantity`. That's the phone book problem.

3. Tags are part of the text index:

```javascript
db.parts.find({ $text: { $search: "lithium" } }, { _id: 0, name: 1 })
```

```
[ { name: '18650 Battery' } ]
```

</details>

---

## Common Issues

**Problem:** `MongoServerError: E11000 duplicate key error` when creating a unique index.  
**Solution:** Find and fix the duplicates first, then create the index again.  
**Why:** MongoDB won't build a unique index over data that already breaks the rule.

**Problem:** `MongoServerError[IndexOptionsConflict]: An equivalent index already exists with a different name and options` when creating a text index.  
**Solution:** Drop the existing text index, then create one that lists all the fields you want.  
**Why:** A collection can have only one text index.

**Problem:** `explain()` still shows `COLLSCAN` after creating an index.  
**Solution:** Check the field name in your query matches the index exactly, and that a compound index's first field is in your filter.  
**Why:** MongoDB only uses an index that can actually narrow down the search.

---

## Recap

- An index is a sorted lookup list that lets MongoDB skip reading every document.
- `explain("executionStats")` shows `COLLSCAN` vs `IXSCAN` and how many documents were examined.
- Compound indexes cover several fields and work for queries on their first field; unique indexes block duplicates; text indexes power word search with `$text`.
- Every index slows writes and uses space, so only index what you query often.

Next, we leave mongosh behind and start talking to MongoDB from Python.
