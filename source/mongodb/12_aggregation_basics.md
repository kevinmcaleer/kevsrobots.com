---
title: Aggregation Basics
description: Use MongoDB's aggregation pipeline to count, total and group your Parts Bin data - stock per category, total workshop value and parts per project.
layout: lesson
type: page
cover: /learn/mongodb/assets/banners/12_aggregation_basics.jpg
date_updated: 2026-10-18
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

`find()` is brilliant at answering "show me the parts that match this". But some questions aren't about individual parts at all. How many parts do I have in each category? What's my whole workshop worth? Which project uses the most parts?

For questions like these MongoDB has the **aggregation pipeline**. To **aggregate** just means to gather lots of things together into a summary - a count, a total, an average.

This lesson starts from the original seed data, so let's reset first. Run `db.parts.drop()`, then re-run the `insertMany` from [lesson 05](05_inserting_documents). You should have 13 parts.

---

## The pipeline idea

Picture an assembly line in a robot factory. Raw parts go in one end. The first station throws out anything faulty, the next sorts parts into crates, the next labels each crate, and finished crates come out the far end.

An aggregation pipeline works the same way. Your documents go in at the top and pass through a list of **stages**. Each stage does one job and hands its results to the next. Nothing in your collection changes - the pipeline only works on copies as they flow past.

You run one with `aggregate()`, passing a list (square brackets) of stages. Every stage name starts with a `$`:

```javascript
db.parts.aggregate([
  { stage one },
  { stage two },
  ...
])
```

---

## `$match` - filter the documents

`$match` is the quality-control station. It takes a filter written exactly like the ones you give `find()`:

```javascript
db.parts.aggregate([
  { $match: { category: "servo" } }
])
```

This gives back the two servos, every field included - the same as `db.parts.find({ category: "servo" })`. On its own `$match` isn't very exciting, but putting it first means later stages have less work to do.

---

## `$project` - choose and calculate fields

`$project` works like a projection in `find()` - pick the fields you want to keep:

```javascript
db.parts.aggregate([
  { $match: { category: "servo" } },
  { $project: { _id: 0, name: 1, quantity: 1 } }
])
```

```
[
  { name: 'SG90 Micro Servo', quantity: 12 },
  { name: 'MG996R Servo', quantity: 4 }
]
```

The clever part is that `$project` can also **compute** new fields. How much is the stock in each bin worth? That's quantity times price:

```javascript
db.parts.aggregate([
  { $project: { _id: 0, name: 1, quantity: 1, price: 1,
                stock_value: { $multiply: ["$quantity", "$price"] } } },
  { $sort: { stock_value: -1 } },
  { $limit: 3 }
])
```

```
[
  { name: 'Raspberry Pi 5 4GB', quantity: 1, price: 55, stock_value: 55 },
  {
    name: 'Raspberry Pi Pico W',
    quantity: 5,
    price: 6.3,
    stock_value: 31.5
  },
  { name: 'SG90 Micro Servo', quantity: 12, price: 2.5, stock_value: 30 }
]
```

Two important details here:

- **`"$quantity"` with a dollar sign means "the value of the quantity field"**. Without the `$` it would just be the word "quantity".
- `$sort` and `$limit` are stages too, doing the same jobs as `.sort()` and `.limit()` from [lesson 06](06_finding_documents). Here we sorted by our brand new `stock_value` field, which `find()` could never do.

---

## `$group` - put documents into crates

`$group` is where aggregation really shines. You tell it what to group by using a field called `_id`, then describe what to work out for each group:

```javascript
db.parts.aggregate([
  { $group: { _id: "$category", parts: { $sum: 1 } } }
])
```

```
[
  { _id: 'board', parts: 2 },
  { _id: 'motor', parts: 1 },
  { _id: 'sensor', parts: 2 },
  { _id: 'lighting', parts: 1 },
  { _id: 'power', parts: 1 },
  { _id: 'servo', parts: 2 },
  { _id: 'hardware', parts: 1 },
  { _id: 'driver', parts: 3 }
]
```

Each output document is one crate. `_id` holds the category, and `parts: { $sum: 1 }` adds 1 for every document dropped into that crate - in other words, it counts them. Things like `$sum` that build up a value as documents arrive are called **accumulators**.

The groups come out in no particular order, so let's also total the quantity and add a `$sort`:

```javascript
db.parts.aggregate([
  { $group: { _id: "$category",
              parts: { $sum: 1 },
              total_quantity: { $sum: "$quantity" } } },
  { $sort: { total_quantity: -1 } }
])
```

```
[
  { _id: 'servo', parts: 2, total_quantity: 16 },
  { _id: 'sensor', parts: 2, total_quantity: 10 },
  { _id: 'motor', parts: 1, total_quantity: 8 },
  { _id: 'driver', parts: 3, total_quantity: 7 },
  { _id: 'board', parts: 2, total_quantity: 6 },
  { _id: 'power', parts: 1, total_quantity: 6 },
  { _id: 'hardware', parts: 1, total_quantity: 3 },
  { _id: 'lighting', parts: 1, total_quantity: 0 }
]
```

Other handy accumulators are `$avg` (average), `$min`, `$max` and `$push` (collect values into a list).

---

## What's my workshop worth?

To get one answer for the whole collection, group everything into a single crate by setting `_id` to `null`. And we can multiply inside `$sum`:

```javascript
db.parts.aggregate([
  { $group: { _id: null,
              total_value: { $sum: { $multiply: ["$quantity", "$price"] } },
              total_items: { $sum: "$quantity" } } }
])
```

```
[ { _id: null, total_value: 263.77, total_items: 56 } ]
```

56 items worth 263.77 in total. Decimal maths on computers can leave long tails, though - without rounding, the drivers category comes out as `29.950000000000003`. So for money it's worth rounding. Here's the value of each category, rounded to 2 decimal places with `$round`:

```javascript
db.parts.aggregate([
  { $group: { _id: "$category",
              value: { $sum: { $multiply: ["$quantity", "$price"] } } } },
  { $project: { value: { $round: ["$value", 2] } } },
  { $sort: { value: -1 } }
])
```

```
[
  { _id: 'board', value: 86.5 },
  { _id: 'servo', value: 57.96 },
  { _id: 'sensor', value: 30.76 },
  { _id: 'driver', value: 29.95 },
  { _id: 'motor', value: 25.6 },
  { _id: 'power', value: 21 },
  { _id: 'hardware', value: 12 },
  { _id: 'lighting', value: 0 }
]
```

We'll use exactly this pipeline for the `value` report in our Parts Bin app in [lesson 15](15_building_the_parts_bin_app).

---

## `$unwind` - one document per list item

Which project uses the most parts? The project names live in the `used_in` list, and you can't group by a whole list. `$unwind` fixes that by splitting a document into one copy per list item:

```javascript
db.parts.aggregate([
  { $match: { name: "SG90 Micro Servo" } },
  { $unwind: "$used_in" },
  { $project: { _id: 0, name: 1, used_in: 1 } }
])
```

```
[
  { name: 'SG90 Micro Servo', used_in: 'SMARS' },
  { name: 'SG90 Micro Servo', used_in: 'BurgerBot' }
]
```

One servo, two projects, two documents - and `used_in` is now a plain string. Unwind every part, then group by project:

```javascript
db.parts.aggregate([
  { $unwind: "$used_in" },
  { $group: { _id: "$used_in", parts: { $sum: 1 } } },
  { $sort: { parts: -1, _id: 1 } }
])
```

```
[
  { _id: 'BurgerBot', parts: 6 },
  { _id: 'SMARS', parts: 6 },
  { _id: 'Robot Arm', parts: 4 }
]
```

BurgerBot and SMARS tie on 6, so the second sort key, `_id: 1`, puts them in alphabetical order.

---

## A taste of `$lookup`

In the last lesson we saw that references need a second query. The `$lookup` stage does that join inside the pipeline. If you made a `projects` collection in [lesson 11](11_designing_your_data), its `part_id` values pointed at the parts we've just dropped, so make a fresh one:

```javascript
db.projects.drop()
db.projects.insertOne({
  name: "SMARS",
  parts: [
    { part_id: db.parts.findOne({ name: "Raspberry Pi Pico W" })._id, qty: 1 },
    { part_id: db.parts.findOne({ name: "N20 Gear Motor" })._id, qty: 2 }
  ]
})
```

Now ask for each project's part IDs to be matched against `_id` in `parts`:

```javascript
db.projects.aggregate([
  { $match: { name: "SMARS" } },
  { $lookup: { from: "parts", localField: "parts.part_id",
               foreignField: "_id", as: "part_details" } },
  { $project: { _id: 0, name: 1, "part_details.name": 1, "part_details.quantity": 1 } }
])
```

```
[
  {
    name: 'SMARS',
    part_details: [
      { name: 'Raspberry Pi Pico W', quantity: 5 },
      { name: 'N20 Gear Motor', quantity: 8 }
    ]
  }
]
```

That's a project and its live stock levels in one query. None of the pipelines in this lesson changed the `parts` data.

---

## Try it Yourself

1. How many parts are tagged `"5v"`, and what's their average price?
2. What's the total stock value of just the servos?
3. For each shelf (`location.shelf`), list the names of parts with fewer than 5 in stock.

<details><summary>Show answers</summary>

1. `$match` first, then a single group:

```javascript
db.parts.aggregate([
  { $match: { tags: "5v" } },
  { $group: { _id: null, count: { $sum: 1 }, avg_price: { $avg: "$price" } } }
])
```

```
[ { _id: null, count: 4, avg_price: 3.6975 } ]
```

2. Same idea, with `$multiply` inside `$sum`:

```javascript
db.parts.aggregate([
  { $match: { category: "servo" } },
  { $group: { _id: null, value: { $sum: { $multiply: ["$quantity", "$price"] } } } }
])
```

```
[ { _id: null, value: 57.96 } ]
```

3. Dot notation works in `$group` too, and `$push` collects the names:

```javascript
db.parts.aggregate([
  { $match: { quantity: { $lt: 5 } } },
  { $group: { _id: "$location.shelf", low_parts: { $push: "$name" } } },
  { $sort: { _id: 1 } }
])
```

```
[
  { _id: 'A', low_parts: [ 'MG996R Servo' ] },
  { _id: 'B', low_parts: [ 'VL53L0X Time-of-Flight Sensor' ] },
  { _id: 'C', low_parts: [ 'Raspberry Pi 5 4GB' ] },
  {
    _id: 'D',
    low_parts: [
      'L298N Motor Driver',
      'DRV8833 Motor Driver',
      'PCA9685 Servo Driver'
    ]
  },
  { _id: 'E', low_parts: [ 'WS2812B LED Strip 1m' ] },
  { _id: 'F', low_parts: [ 'M3 Screws (pack of 100)' ] }
]
```

</details>

---

## Common Issues

**Problem:** Every total comes out as `0`.  
**Solution:** Write `{ $sum: "$quantity" }`, not `{ $sum: "quantity" }`.  
**Why:** Without the `$`, MongoDB sees the text "quantity" rather than the field's value, and text adds up to nothing.

**Problem:** `MongoServerError: a group specification must include an _id`  
**Solution:** Every `$group` needs an `_id`. Use `_id: null` if you want one group for everything.  
**Why:** `_id` is how `$group` decides which crate each document goes in.

**Problem:** `MongoServerError: The field 'category' must be an accumulator object`  
**Solution:** Put the field you're grouping by in `_id`, and give every other field an accumulator like `$sum` or `$push`.  
**Why:** Each group may hold many documents, so MongoDB needs to know how to combine their values.

**Problem:** Parts go missing after `$unwind`.  
**Solution:** Use `{ $unwind: { path: "$used_in", preserveNullAndEmptyArrays: true } }` if you want to keep them.  
**Why:** The VL53L0X and the LED strip have an empty `used_in` list, so there's nothing to unwind and they're dropped.

---

## Recap

- A pipeline is a list of stages; documents flow through each in turn, and your stored data doesn't change.
- `$match` filters, `$project` picks or calculates fields, `$sort` and `$limit` order and trim.
- `$group` with an `_id` and accumulators like `$sum` and `$avg` produces counts and totals - `_id: null` gives a grand total.
- `$unwind` splits a list into one document per item, and `$lookup` joins in documents from another collection.

Next, we'll look at indexes - the trick that keeps all these queries fast as your data grows.
