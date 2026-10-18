---
title: Deleting Documents
description: Remove documents safely with deleteOne and deleteMany, drop collections and databases, and consider a soft delete instead
layout: lesson
type: page
cover: /learn/mongodb/assets/banners/09_deleting_documents.jpg
date_updated: 2026-10-18
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Sooner or later something leaves the workshop for good. A servo gets its gears stripped, a sensor gets let out of its magic smoke, or you just added a part by mistake. Time to **delete** it.

Deleting is the easiest thing in MongoDB to do, and the easiest to get wrong. There's no recycle bin and no undo button. Once a document is gone, it's gone - unless you've got a backup (we'll cover backups in [lesson 16](16_keeping_your_data_safe)).

So this lesson is as much about good habits as it is about commands.

This lesson carries on from [lesson 08](08_updating_documents), where we added a 14th part, the TT Gear Motor. If you've reset your data since then, that's fine - just skip the first example.

---

## The golden habit: find first, then delete

Before deleting anything, run `find()` with the **same filter** you're about to delete with. It shows you exactly what's going to disappear:

```javascript
db.parts.find({ name: "TT Gear Motor" })
```

```
[
  {
    _id: ObjectId('6ad5239be10decb533e9ce83'),
    name: 'TT Gear Motor',
    category: 'motor',
    quantity: 6,
    price: 1.5
  }
]
```

That's the one, and only the one. Now press the up arrow, change `find` to `deleteOne`, and run it:

```javascript
db.parts.deleteOne({ name: "TT Gear Motor" })
```

```
{ acknowledged: true, deletedCount: 1 }
```

`deletedCount` tells you how many documents were removed. Run it again and you'll get `deletedCount: 0` - there's nothing left to delete, and that isn't an error.

```javascript
db.parts.countDocuments()
```

```
13
```

Back to our 13 parts.

---

## deleteOne vs deleteMany

Just like `updateOne`, `deleteOne` removes only the **first** document that matches - even if lots of documents match. Let's add a few test parts so we've got something safe to experiment with:

```javascript
db.parts.insertMany([
  { name: "Test Part 1", category: "test" },
  { name: "Test Part 2", category: "test" },
  { name: "Test Part 3", category: "test" }
])
```

Now delete with a filter that matches all three:

```javascript
db.parts.deleteOne({ category: "test" })
```

```
{ acknowledged: true, deletedCount: 1 }
```

```javascript
db.parts.find({ category: "test" }, { name: 1, _id: 0 })
```

```
[ { name: 'Test Part 2' }, { name: 'Test Part 3' } ]
```

Only one went. To remove **every** match, use `deleteMany`. Following our habit, we check first:

```javascript
db.parts.countDocuments({ category: "test" })
```

```
2
```

Two, as expected. Now delete them:

```javascript
db.parts.deleteMany({ category: "test" })
```

```
{ acknowledged: true, deletedCount: 2 }
```

The numbers agree, so we know we removed exactly what we meant to. If `countDocuments` had said 13, we'd have stopped and checked our filter.

---

## The scariest command: deleteMany({})

Remember that an empty filter `{}` matches **everything**. So this:

```javascript
db.parts.deleteMany({})
```

deletes every part in the collection, instantly, with no "are you sure?". Please don't run that on `parts`! Let's try it on a scratch collection instead:

```javascript
db.scratch.insertMany([ { n: 1 }, { n: 2 }, { n: 3 } ])
db.scratch.deleteMany({})
```

```
{ acknowledged: true, deletedCount: 3 }
```

The collection is now empty, but it still exists:

```javascript
show collections
```

```
parts
scratch
```

mongosh does give you one small safety net. If you forget the filter completely and type `db.parts.deleteMany()`, it refuses:

```
Uncaught MongoshInvalidInputError: [COMMON-10001] Missing required argument at position 0 (Collection.deleteMany)
```

But `{}` is a perfectly valid filter, so it won't save you from that one.

---

## Dropping collections and databases

If you want rid of a whole collection - documents, indexes and all - use `drop()`. It's quicker than `deleteMany({})` and removes the collection itself:

```javascript
db.scratch.drop()
```

```
true
```

```javascript
show collections
```

```
parts
```

You've already used `db.parts.drop()` to reset your data back in [lesson 05](05_inserting_documents).

One level up, `db.dropDatabase()` removes the **current database** and everything in it. Let's make a throwaway database to try it on:

```javascript
use playground
db.junk.insertOne({ hello: "world" })
db.dropDatabase()
```

```
{ ok: 1, dropped: 'playground' }
```

Notice `dropDatabase()` doesn't take a name - it drops whichever database you're currently using. Always glance at your prompt first. If it says `workshop>`, you're about to lose your Parts Bin. Switch back now:

```javascript
use workshop
```

---

## The alternative: soft delete

Here's a question worth asking before you delete anything: **might I want this information later?** A part you've run out of and stopped buying still tells you what you paid for it and which projects used it.

A **soft delete** means marking a document as gone instead of really removing it. We just add a field:

```javascript
db.parts.updateOne({ name: "MG996R Servo" }, { $set: { archived: true } })
```

Then our everyday queries leave archived parts out:

```javascript
db.parts.countDocuments({ archived: { $ne: true } })
```

```
12
```

We use `$ne: true` rather than `archived: false` because most parts don't have an `archived` field at all. "Not equal to true" includes documents where the field is missing.

The MG996R is still there whenever we want it:

```javascript
db.parts.find({ archived: true }, { name: 1, _id: 0 })
```

```
[ { name: 'MG996R Servo' } ]
```

And undoing a soft delete is easy - which is the whole point:

```javascript
db.parts.updateOne({ name: "MG996R Servo" }, { $unset: { archived: "" } })
```

Lots of real apps work this way. The downside is that you have to remember to add `archived: { $ne: true }` to your queries.

After this lesson your Parts Bin is back to 13 parts, still carrying the changes from lesson 08.

---

## Try it Yourself

1. Insert a part called "Broken Servo", then delete it using the find-first habit.
2. Soft delete the LED strip. Write a query that finds active (not archived) lighting parts. What comes back? Then undo the soft delete.
3. What does `db.parts.deleteMany({ category: "spaceship" })` return? Is it an error?

<details><summary>Show answers</summary>

1. `db.parts.insertOne({ name: "Broken Servo", category: "servo", quantity: 1 })`, then `db.parts.find({ name: "Broken Servo" })` to check, then `db.parts.deleteOne({ name: "Broken Servo" })` which returns `{ acknowledged: true, deletedCount: 1 }`.
2. `db.parts.updateOne({ name: "WS2812B LED Strip 1m" }, { $set: { archived: true } })`, then `db.parts.find({ category: "lighting", archived: { $ne: true } })` returns nothing, because it's our only lighting part. Undo with `db.parts.updateOne({ name: "WS2812B LED Strip 1m" }, { $unset: { archived: "" } })`.
3. `{ acknowledged: true, deletedCount: 0 }`. Not an error - nothing matched, so nothing was deleted.

</details>

---

## Common Issues

**Problem:** I deleted more than I meant to.  
**Solution:** Restore from a backup, or re-run the `insertMany` from lesson 05 to get the starter data back.  
**Why:** MongoDB has no undo. This is exactly why we check with `find()` or `countDocuments()` first, and why backups matter.

**Problem:** `deleteMany` returned `deletedCount: 0` but the parts are definitely there.  
**Solution:** Check your filter for typos - including in the field name. `{ categroy: "test" }` matches nothing.  
**Why:** A misspelt field name is just a field no document has, so nothing matches.

**Problem:** I ran `dropDatabase()` and lost the wrong database.  
**Solution:** Check the prompt (or type `db`) before running it.  
**Why:** `dropDatabase()` always drops the database you're currently in.

---

## Recap

- `deleteOne` removes the first match, `deleteMany` removes every match
- Always run `find()` or `countDocuments()` with the same filter before deleting
- `deleteMany({})` empties the whole collection - there's no undo
- `drop()` removes a collection; `dropDatabase()` removes the current database
- A soft delete (`archived: true`) keeps the history and is easy to undo

Next, we'll dig into the nested documents and arrays inside our parts - shelves, bins, specs and suppliers.
