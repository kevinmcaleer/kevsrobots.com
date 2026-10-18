---
title: Finding Documents
description: Ask MongoDB questions with find() and findOne() - filters, projections, sorting, limits and counting
layout: lesson
type: page
cover: /learn/mongodb/assets/banners/06_finding_documents.jpg
date_updated: 2026-10-18
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Our Parts Bin now holds 13 parts. Storing data is only half the job though - the real fun starts when we ask it questions. "Which servos do I have?" "What's the most expensive thing on my shelves?" "How many sensors are there?"

In database language, a question is called a **query**. In this lesson we'll write our first proper queries using `find()` and `findOne()`.

> **Starting fresh:** this lesson expects the 13 parts from [lesson 05](05_inserting_documents). If you've been experimenting and things look different, run `db.parts.drop()` and then re-run the `insertMany` from lesson 05.

Open `mongosh` and switch to our database:

```javascript
use workshop
```

---

## find() and findOne()

There are two ways to fetch documents:

- `findOne()` returns **one** document - the first one it comes across - or `null` if nothing matches
- `find()` returns **every** document that matches

Try `findOne()` with nothing inside the brackets:

```javascript
db.parts.findOne()
```

```
{
  _id: ObjectId('6ad52340e98e08b554b1d0f2'),
  name: 'SG90 Micro Servo',
  category: 'servo',
  quantity: 12,
  price: 2.5,
  location: { shelf: 'A', bin: 1 },
  tags: [ 'servo', '5v', 'smars' ],
  specs: { torque_kg_cm: 1.8, voltage: '4.8-6V' },
  used_in: [ 'SMARS', 'BurgerBot' ],
  added: ISODate('2026-01-12T00:00:00.000Z')
}
```

Your `_id` will be different - that's fine, every ObjectId is unique. Notice that `2.50` has come back as `2.5`. It's the same number, mongosh just doesn't print trailing zeros.

You might see older tutorials adding `.pretty()` to the end of queries. You don't need it - mongosh formats results nicely all on its own.

---

## Filters - asking for something specific

Empty brackets mean "anything". To be specific, we pass a **filter**: a small document describing what we're looking for. Each field in the filter must match:

```javascript
db.parts.find({ category: "servo" })
```

```
[
  {
    _id: ObjectId('6ad52340e98e08b554b1d0f2'),
    name: 'SG90 Micro Servo',
    category: 'servo',
    quantity: 12,
    ...
  },
  {
    _id: ObjectId('6ad52340e98e08b554b1d0f3'),
    name: 'MG996R Servo',
    category: 'servo',
    quantity: 4,
    ...
  }
]
```

(We've trimmed the output with `...` to save space - you'll see the full documents.)

`find()` wraps its results in square brackets because it's returning a list. If nothing matches you simply get nothing back - no error.

Matching is exact, and that includes upper and lower case:

```javascript
db.parts.findOne({ name: "mg996r servo" })
```

```
null
```

The type matters too. `quantity: "12"` (a string) will not match a stored `quantity: 12` (a number).

### More than one field means AND

Put two fields in the filter and a document must match **both**:

```javascript
db.parts.find({ category: "driver", quantity: 2 })
```

That returns the DRV8833 and PCA9685 drivers, but not the L298N, because it has 3 in stock.

---

## Projection - choosing which fields come back

Full documents get noisy fast. A **projection** is a second argument to `find()` that says which fields you want to see. Use `1` to include a field:

```javascript
db.parts.find({ category: "driver" }, { name: 1, quantity: 1 })
```

```
[
  {
    _id: ObjectId('6ad52340e98e08b554b1d0f9'),
    name: 'L298N Motor Driver',
    quantity: 3
  },
  {
    _id: ObjectId('6ad52340e98e08b554b1d0fa'),
    name: 'DRV8833 Motor Driver',
    quantity: 2
  },
  {
    _id: ObjectId('6ad52340e98e08b554b1d0fb'),
    name: 'PCA9685 Servo Driver',
    quantity: 2
  }
]
```

`_id` always tags along unless you switch it off with `_id: 0`:

```javascript
db.parts.find({ category: "driver" }, { name: 1, quantity: 1, _id: 0 })
```

```
[
  { name: 'L298N Motor Driver', quantity: 3 },
  { name: 'DRV8833 Motor Driver', quantity: 2 },
  { name: 'PCA9685 Servo Driver', quantity: 2 }
]
```

Much tidier. You can also go the other way and use `0` to **exclude** fields, keeping everything else:

```javascript
db.parts.find({ category: "board" }, { specs: 0, tags: 0, used_in: 0, added: 0, _id: 0 })
```

```
[
  {
    name: 'Raspberry Pi Pico W',
    category: 'board',
    quantity: 5,
    price: 6.3,
    location: { shelf: 'C', bin: 1 }
  },
  {
    name: 'Raspberry Pi 5 4GB',
    category: 'board',
    quantity: 1,
    price: 55,
    location: { shelf: 'C', bin: 2 }
  }
]
```

The one rule: pick a style. Mixing `1`s and `0`s in the same projection is an error - `_id: 0` is the only exception.

---

## Sorting, limiting and skipping

You can tack extra instructions onto the end of `find()`. `sort()` orders the results: `1` means smallest first (A to Z), `-1` means largest first.

```javascript
db.parts.find({}, { name: 1, price: 1, _id: 0 }).sort({ price: -1 })
```

```
[
  { name: 'Raspberry Pi 5 4GB', price: 55 },
  { name: 'WS2812B LED Strip 1m', price: 7.5 },
  { name: 'MG996R Servo', price: 6.99 },
  { name: 'Raspberry Pi Pico W', price: 6.3 },
  { name: 'PCA9685 Servo Driver', price: 5.99 },
  { name: 'VL53L0X Time-of-Flight Sensor', price: 4.99 },
  { name: 'DRV8833 Motor Driver', price: 4.5 },
  { name: 'M3 Screws (pack of 100)', price: 4 },
  { name: '18650 Battery', price: 3.5 },
  { name: 'N20 Gear Motor', price: 3.2 },
  { name: 'L298N Motor Driver', price: 2.99 },
  { name: 'SG90 Micro Servo', price: 2.5 },
  { name: 'HC-SR04 Ultrasonic Sensor', price: 1.8 }
]
```

Notice the empty `{}` filter - we want every part, but we still need it there so the projection lands in the second slot.

`limit()` keeps only the first few results. Here are our three priciest parts:

```javascript
db.parts.find({}, { name: 1, price: 1, _id: 0 }).sort({ price: -1 }).limit(3)
```

```
[
  { name: 'Raspberry Pi 5 4GB', price: 55 },
  { name: 'WS2812B LED Strip 1m', price: 7.5 },
  { name: 'MG996R Servo', price: 6.99 }
]
```

`skip()` jumps over some results first, which is how you'd show "page two":

```javascript
db.parts.find({}, { name: 1, price: 1, _id: 0 }).sort({ price: -1 }).skip(3).limit(3)
```

```
[
  { name: 'Raspberry Pi Pico W', price: 6.3 },
  { name: 'PCA9685 Servo Driver', price: 5.99 },
  { name: 'VL53L0X Time-of-Flight Sensor', price: 4.99 }
]
```

You can sort by more than one field. `sort({ category: 1, quantity: -1 })` sorts by category A to Z, and within each category puts the biggest stock first.

---

## Counting

Sometimes you just want a number. `countDocuments()` takes the same kind of filter as `find()`:

```javascript
db.parts.countDocuments()
```

```
13
```

```javascript
db.parts.countDocuments({ category: "sensor" })
```

```
2
```

---

## Matching values inside a list

Here's a lovely MongoDB trick. Our `tags` field is an **array** - a list of values. If you filter on an array field with a single value, MongoDB checks whether that value is **anywhere in the list**:

```javascript
db.parts.find({ tags: "servo" }, { name: 1, tags: 1, _id: 0 })
```

```
[
  { name: 'SG90 Micro Servo', tags: [ 'servo', '5v', 'smars' ] },
  { name: 'MG996R Servo', tags: [ 'servo', 'metal gear' ] },
  { name: 'PCA9685 Servo Driver', tags: [ 'servo', 'i2c' ] }
]
```

The PCA9685 is a driver, not a servo, but it carries the `servo` tag - so it shows up. The same works for `used_in`. Which parts go into my SMARS robot?

```javascript
db.parts.find({ used_in: "SMARS" }, { name: 1, _id: 0 })
```

```
[
  { name: 'SG90 Micro Servo' },
  { name: 'HC-SR04 Ultrasonic Sensor' },
  { name: 'Raspberry Pi Pico W' },
  { name: 'N20 Gear Motor' },
  { name: 'DRV8833 Motor Driver' },
  { name: 'M3 Screws (pack of 100)' }
]
```

That's a shopping list for building another SMARS. We'll dig much deeper into arrays in [lesson 10](10_nested_documents_and_arrays).

---

## Try it Yourself

1. List the names (and nothing else) of every part used in **BurgerBot**.
2. Find the single cheapest part in the workshop, showing just its name and price.
3. Count how many motor drivers (`category: "driver"`) you have.
4. Run `findOne({ category: "board" })`. Which board comes back, and why that one?

<details><summary>Show answers</summary>

1. `db.parts.find({ used_in: "BurgerBot" }, { name: 1, _id: 0 })` - six parts, from the SG90 to the M3 screws.
2. `db.parts.find({}, { name: 1, price: 1, _id: 0 }).sort({ price: 1 }).limit(1)` gives `[ { name: 'HC-SR04 Ultrasonic Sensor', price: 1.8 } ]`.
3. `db.parts.countDocuments({ category: "driver" })` returns `3`.
4. The Raspberry Pi Pico W. `findOne()` returns the first match it finds, which (with no sort) is usually the one inserted first. Don't rely on that order - if order matters, use `find().sort().limit(1)`.

</details>

---

## Common Issues

**Problem:** A query you're sure is right returns nothing.  
**Solution:** Check the spelling, the capital letters and the type. `"Servo"` is not `"servo"`, and `"12"` is not `12`.  
**Why:** Equality filters are exact matches. We'll learn case-insensitive searching in the next lesson.

**Problem:** `MongoServerError[Location31254]: Cannot do exclusion on field quantity in inclusion projection`  
**Solution:** Use only `1`s or only `0`s in a projection (apart from `_id: 0`).  
**Why:** MongoDB can't guess what you want when you say "show only these" and "hide those" at the same time.

**Problem:** My projection is being treated as a filter and nothing comes back.  
**Solution:** If you want every document, put an empty `{}` first: `find({}, { name: 1 })`.  
**Why:** The first argument is always the filter, the second is always the projection.

---

## Recap

- `find()` returns every match, `findOne()` returns just one (or `null`)
- A filter with several fields means all of them must match
- A projection picks the fields you see - use `_id: 0` to hide the id
- `sort()`, `limit()` and `skip()` control order and how many results you get
- Filtering an array field with a single value finds documents where the list **contains** that value

Next up, we'll go beyond exact matches with **query operators** - "less than", "one of these", "contains the word" and more.
