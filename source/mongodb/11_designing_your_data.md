---
title: Designing Your Data
description: Learn when to embed data inside a document and when to reference another collection, and add schema validation to stop typos and bad values getting into your Parts Bin.
layout: lesson
type: page
cover: /learn/mongodb/assets/banners/11_designing_your_data.jpg
date_updated: 2026-10-18
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

So far we've added fields wherever it felt handy - MongoDB never forces a layout on you. But sooner or later you'll ask "should this live inside the part, or somewhere else?", and you'll make a typo that quietly creates a brand new field.

In this lesson we'll look at the two big design choices - **embedding** and **referencing** - and then add some rules to the `parts` collection so MongoDB itself refuses bad data.

> **Starting fresh:** the outputs below assume the original 13 parts, so run `db.parts.drop()` and re-run the `insertMany` from [lesson 05](05_inserting_documents) before you begin.

---

## The question every design comes down to

When two pieces of information are related, you have two choices:

- **Embed** - put one inside the other. Our `location` and `specs` fields are embedded: they're little documents living inside each part. These are often called **sub-documents** (a document nested inside another document).
- **Reference** - keep them in separate collections and store the `_id` of one inside the other, like writing "see page 42" rather than copying page 42 out.

Neither is "right". It depends on how you use the data.

---

## Rules of thumb

1. **Things you read together should live together.** Whenever you look at a part, you want its shelf and bin. So `location` is embedded - one read gets everything.
2. **If a list can grow forever, give it its own collection.** A part has a handful of tags, but a log of every time you took one out of the bin will keep growing for years. A single document has a hard limit of 16MB, and huge ones are slow anyway.
3. **If something is shared by lots of other things, or has a life of its own, reference it.** A robot project uses many parts, and each part can be in many projects. A project also has its own details - a description, a status, a link to the build guide.

These relationships have names you'll see in the docs:

| Relationship | Parts Bin example | Usual choice |
|---|---|---|
| One-to-one | A part and its `location` | Embed |
| One-to-few | A part and its `tags`, or a few `suppliers` | Embed |
| One-to-many (growing forever) | A part and every stock change ever made | Separate collection, reference the part |
| Many-to-many | Parts and robot projects | Reference |
{:class="table table-single"}

---

## Where `used_in` falls short

Let's look at how we track projects right now:

```javascript
db.parts.find({}, { _id: 0, name: 1, used_in: 1 }).limit(3)
```

```
[
  { name: 'SG90 Micro Servo', used_in: [ 'SMARS', 'BurgerBot' ] },
  { name: 'MG996R Servo', used_in: [ 'Robot Arm' ] },
  {
    name: 'HC-SR04 Ultrasonic Sensor',
    used_in: [ 'SMARS', 'BurgerBot' ]
  }
]
```

That answers "which projects use this servo?", but there's nowhere to put anything *about* a project, such as how many of each part it needs. Rename BurgerBot and you have to update every part that mentions it. Type `"Burgerbot"` once and you've invented a fourth project without noticing.

---

## A `projects` collection that references parts

Let's make a separate `projects` collection. Each project holds a short list of the parts it needs, and each entry points at a part by its `_id`.

First, grab the `_id` of each part we need and keep it in a variable (a named box that holds a value for later). `const` is just JavaScript's way of creating one:

```javascript
const pico = db.parts.findOne({ name: "Raspberry Pi Pico W" })._id
const motor = db.parts.findOne({ name: "N20 Gear Motor" })._id
const driver = db.parts.findOne({ name: "DRV8833 Motor Driver" })._id
const sonar = db.parts.findOne({ name: "HC-SR04 Ultrasonic Sensor" })._id
```

mongosh prints nothing for those lines. Type a variable's name to peek inside:

```javascript
pico
```

```
ObjectId('6ad5236f81ab2acfa6ebb061')
```

Your ObjectId will be different - every one is unique. Now create the project:

```javascript
db.projects.insertOne({
  name: "SMARS",
  description: "Screwless Modular Assemblable Robotic System",
  status: "building",
  parts: [
    { part_id: pico, qty: 1 },
    { part_id: motor, qty: 2 },
    { part_id: driver, qty: 1 },
    { part_id: sonar, qty: 1 }
  ]
})
```

```
{
  acknowledged: true,
  insertedId: ObjectId('6ad5237006a22337c6172873')
}
```

Notice we've used *both* ideas. The `parts` list is **embedded**, because a robot needs only a handful of parts and we always want the list with the project. But each entry **references** a part rather than copying its name, price and stock.

```javascript
db.projects.findOne({ name: "SMARS" })
```

```
{
  _id: ObjectId('6ad5237006a22337c6172873'),
  name: 'SMARS',
  description: 'Screwless Modular Assemblable Robotic System',
  status: 'building',
  parts: [
    { part_id: ObjectId('6ad5236f81ab2acfa6ebb061'), qty: 1 },
    { part_id: ObjectId('6ad5236f81ab2acfa6ebb063'), qty: 2 },
    { part_id: ObjectId('6ad5236f81ab2acfa6ebb065'), qty: 1 },
    { part_id: ObjectId('6ad5236f81ab2acfa6ebb05f'), qty: 1 }
  ]
}
```

### Following the references

The project only holds IDs, so to see the actual parts we need a second query. Pull the IDs out into a list with JavaScript's `map()`, then use `$in` from [lesson 07](07_query_operators):

```javascript
const smars = db.projects.findOne({ name: "SMARS" })
const ids = smars.parts.map(p => p.part_id)
db.parts.find({ _id: { $in: ids } }, { _id: 0, name: 1, quantity: 1 })
```

```
[
  { name: 'HC-SR04 Ultrasonic Sensor', quantity: 6 },
  { name: 'Raspberry Pi Pico W', quantity: 5 },
  { name: 'N20 Gear Motor', quantity: 8 },
  { name: 'DRV8833 Motor Driver', quantity: 2 }
]
```

That's the cost of referencing: two trips to the database instead of one. The payoff is that stock lives in exactly one place, so every project sees the latest numbers. (MongoDB can also join collections in a single query with a `$lookup` stage - we'll meet that in the next lesson.)

We'll keep `used_in` on our parts too, because it's handy for quick reports. Storing the same fact in two places is a common, deliberate trade-off - just remember you've done it.

---

## Schema validation - stopping bad data at the door

A **schema** is a description of what your documents should look like: which fields they have and what type each one is. MongoDB doesn't need one, but you can add one so the server rejects documents that break your rules. This is called **schema validation**, and it uses a format called `$jsonSchema`.

Let's say every part *must* have a name, category, quantity and price, and quantity and price can't be negative. For an existing collection we use the `collMod` (collection modify) command:

```javascript
db.runCommand({
  collMod: "parts",
  validator: {
    $jsonSchema: {
      bsonType: "object",
      required: ["name", "category", "quantity", "price"],
      properties: {
        name: { bsonType: "string", description: "must be a string and is required" },
        category: { bsonType: "string", description: "must be a string and is required" },
        quantity: { bsonType: "number", minimum: 0, description: "must be a number, 0 or more" },
        price: { bsonType: "number", minimum: 0, description: "must be a number, 0 or more" }
      }
    }
  }
})
```

```
{ ok: 1 }
```

`bsonType: "number"` accepts any kind of number - whole numbers and decimals. Now let's misspell `quantity`:

```javascript
db.parts.insertOne({ name: "TT Motor", category: "motor", quantiy: 4, price: 1.20 })
```

```
Uncaught:
MongoServerError: Document failed validation
Additional information: {
  failingDocumentId: ObjectId('6ad52379e41f301e893ab484'),
  details: {
    operatorName: '$jsonSchema',
    schemaRulesNotSatisfied: [
      {
        operatorName: 'required',
        specifiedAs: { required: [ 'name', 'category', 'quantity', 'price' ] },
        missingProperties: [ 'quantity' ]
      }
    ]
  }
}
```

Rejected, and the error even tells us which field is missing. Validation also checks updates. We have 3 L298N drivers - let's try to use 5:

```javascript
db.parts.updateOne({ name: "L298N Motor Driver" }, { $inc: { quantity: -5 } })
```

```
Uncaught:
MongoServerError: Document failed validation
Additional information: {
  failingDocumentId: ObjectId('6ad5236f81ab2acfa6ebb064'),
  details: {
    operatorName: '$jsonSchema',
    schemaRulesNotSatisfied: [
      {
        operatorName: 'properties',
        propertiesNotSatisfied: [
          {
            propertyName: 'quantity',
            description: 'must be a number, 0 or more',
            details: [
              {
                operatorName: 'minimum',
                specifiedAs: { minimum: 0 },
                reason: 'comparison failed',
                consideredValue: -2
              }
            ]
          }
        ]
      }
    ]
  }
}
```

The stock would have gone to -2, so the whole update was refused. Using just one is fine:

```javascript
db.parts.updateOne({ name: "L298N Motor Driver" }, { $inc: { quantity: -1 } })
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

And the correctly spelled TT Motor goes straight in:

```javascript
db.parts.insertOne({ name: "TT Motor", category: "motor", quantity: 4, price: 1.20 })
```

```
{
  acknowledged: true,
  insertedId: ObjectId('6ad52379e41f301e893ab485')
}
```

### Creating a new collection with rules

For a brand new collection, pass the same `validator` to `db.createCollection("name", { validator: { ... } })`. You'll do exactly that in the challenges below.

A few things worth knowing:

- Rules only cover the fields you list. A typo in a field that isn't `required` (say `tgas` instead of `tags`) still slips through. You can add `additionalProperties: false` to ban unknown fields, but then you must list every field, including `_id`.
- Existing documents aren't checked when you add rules - only new inserts and updates.
- `db.parts.drop()` removes the rules along with the data, so the reset snippet from [lesson 05](05_inserting_documents) gives you a rule-free collection again.

After this lesson your `parts` collection has validation rules, an extra TT Motor (14 parts) and one fewer L298N driver, plus a new `projects` collection. The next lesson starts with a reset, so don't worry about tidying up.

---

## Try it Yourself

1. Add a `BurgerBot` project with a `status` of `"finished"` that references the Pico W, the L298N Motor Driver and two 18650 Batteries.
2. Create a `stock_log` collection with validation that requires `part_id` (an `objectId`), `change` (a number) and `at` (a `date`). Insert one valid entry for the SG90 servo, then try one where `part_id` is the string `"SG90"`.
3. Try inserting a part whose price is the string `"0.50"`. Which rule catches it?

<details><summary>Show answers</summary>

1. You can look up each `_id` right inside the document:

```javascript
db.projects.insertOne({
  name: "BurgerBot",
  status: "finished",
  parts: [
    { part_id: db.parts.findOne({ name: "Raspberry Pi Pico W" })._id, qty: 1 },
    { part_id: db.parts.findOne({ name: "L298N Motor Driver" })._id, qty: 1 },
    { part_id: db.parts.findOne({ name: "18650 Battery" })._id, qty: 2 }
  ]
})
```

2. Create the collection with its rules, then insert:

```javascript
db.createCollection("stock_log", {
  validator: {
    $jsonSchema: {
      bsonType: "object",
      required: ["part_id", "change", "at"],
      properties: {
        part_id: { bsonType: "objectId" },
        change: { bsonType: "number" },
        at: { bsonType: "date" }
      }
    }
  }
})
const sg90 = db.parts.findOne({ name: "SG90 Micro Servo" })._id
db.stock_log.insertOne({ part_id: sg90, change: -2, at: new Date() })
db.stock_log.insertOne({ part_id: "SG90", change: -2, at: new Date() })
```

The first insert works. The second fails with `reason: 'type did not match'`, `consideredType: 'string'`. This is the "grows forever" pattern from earlier - one small document per stock change, each pointing back at its part.

3. `db.parts.insertOne({ name: "Buzzer", category: "sound", quantity: 3, price: "0.50" })` fails on the `bsonType` rule for `price` - `"0.50"` in quotes is text, not a number.

</details>

---

## Common Issues

**Problem:** `MongoServerError: Document failed validation` on an insert that looks fine.  
**Solution:** Read the `details` section - it names the field (`propertyName` or `missingProperties`) and the rule it broke.  
**Why:** Usually it's a typo in a field name, or a number typed in quotes so it became a string.

**Problem:** A project's `part_id` finds nothing after you reset the `parts` collection.  
**Solution:** Recreate the project after resetting, looking the IDs up again.  
**Why:** The reset inserted brand new parts with brand new ObjectIds. A reference is only a stored ID - MongoDB doesn't check that it points at anything, and won't fix it for you.

**Problem:** Your rules don't seem to apply any more.  
**Solution:** Run the `collMod` command again.  
**Why:** Dropping a collection deletes its validation rules too.

---

## Recap

- **Embed** data you always read together and that stays small; **reference** data that's shared, has a life of its own, or grows forever.
- A reference is just an `_id` stored in another document - following it takes a second query (or `$lookup`).
- Embedding and referencing mix happily, as in our `projects` collection.
- `$jsonSchema` validation, added with `collMod` or `createCollection`, makes MongoDB reject missing fields, wrong types and negative stock.

Next up, we'll start asking bigger questions of our data with the aggregation pipeline.
