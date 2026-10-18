---
title: Documents and Collections
description: Learn to read JSON, and see how MongoDB organises data into databases, collections, documents and fields
layout: lesson
type: page
cover: /learn/mongodb/assets/banners/02_documents_and_collections.jpg
date_updated: 2026-10-18
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Before we install anything, let's learn to read the format MongoDB uses for its data. Once you can read a document, every command in the rest of the course will make a lot more sense.

---

## JSON in plain English

MongoDB documents look almost exactly like **JSON** (JavaScript Object Notation). JSON is a way of writing down information as text so that both people and programs can read it. You have probably seen it before without knowing its name.

Here is a part written as JSON:

```json
{
  "name": "SG90 Micro Servo",
  "category": "servo",
  "quantity": 12,
  "price": 2.50,
  "in_stock": true,
  "supplier": null,
  "tags": ["servo", "5v", "smars"],
  "location": { "shelf": "A", "bin": 1 }
}
```

It is built from just a few pieces:

- **Curly braces** `{ }` wrap the whole thing. Everything inside is about one item.
- Each line inside is a **key-value pair**. The **key** is a label (`"name"`), then a colon, then the **value** (`"SG90 Micro Servo"`).
- Pairs are separated by **commas**. There is no comma after the last one.

The values can be a handful of different types:

| Type | Looks like | Used for |
|---|---|---|
| String | `"SG90 Micro Servo"` | Text, always in quotes |
| Number | `12` or `2.50` | Counts, prices, measurements - no quotes |
| Boolean | `true` or `false` | Yes or no answers |
| Null | `null` | "No value" or "don't know yet" |
| Array (list) | `["servo", "5v", "smars"]` | Several values, in square brackets |
| Object (nested) | `{ "shelf": "A", "bin": 1 }` | A little group of key-value pairs inside another |
{:class="table table-single"}

Notice that `12` and `"12"` are different. The first is a number you can do sums with; the second is just text that happens to contain digits.

---

## Documents, collections and databases

Now we can name the parts of MongoDB.

- A **document** is one record, written in that JSON style. One part in our workshop is one document.
- A **field** is one key-value pair inside a document. `quantity: 12` is a field.
- A **collection** is a group of related documents. All our parts live together in a collection called `parts`.
- A **database** is a group of collections. Ours is called `workshop`. Later it might also hold a `projects` collection.
- The **server** can hold many databases at once.

So the hierarchy, from biggest to smallest, is:

**server** > **databases** > **collections** > **documents** > **fields**

In workshop terms: the server is the whole workshop, a database is one set of shelves, a collection is a drawer, each document is a labelled card in that drawer, and the fields are the lines written on the card.

---

## BSON - JSON with extra powers

Behind the scenes MongoDB doesn't store plain text JSON. It stores **BSON** (Binary JSON), a compact computer-friendly version of the same idea. You will never need to read BSON directly, but it brings two useful extras that plain JSON lacks:

- **Dates.** JSON has no proper date type, so people store dates as strings and hope for the best. BSON has a real date type, so MongoDB can sort by date and answer "what did I add in March?"
- **ObjectIds.** Every document needs a unique label. MongoDB adds a field called `_id` to each document, and if you don't supply one it fills it with an **ObjectId**: a 12-byte value that is unique and even has the time it was created baked inside. We will crack one open in [lesson 05](05_inserting_documents).

BSON also knows the difference between whole numbers (integers) and numbers with a decimal point (doubles), which keeps sums accurate.

---

## How mongosh shows a document

When you look at a document in `mongosh`, MongoDB's shell, it looks slightly different from strict JSON. Here is a real part from the Parts Bin, exactly as `mongosh` displayed it:

```
{
  _id: ObjectId('6ad52357ccb7d08b69e69b68'),
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

The differences from JSON are just presentation: keys have no quotes, strings use single quotes, and `2.50` is shown as `2.5` (it is the same number). Here is what each field is doing:

| Field | Type | What it means |
|---|---|---|
| `_id` | ObjectId | The unique ID MongoDB added automatically |
| `name` | String | What the part is called |
| `category` | String | The kind of part, so we can group them |
| `quantity` | Number | How many are in the bin right now |
| `price` | Number | Cost of one, in pounds |
| `location` | Nested object | Where to find it: which shelf, which bin |
| `tags` | Array | Handy labels for searching |
| `specs` | Nested object | Technical details that differ for every kind of part |
| `used_in` | Array | Which projects use this part |
| `added` | Date | When it went into the bin |
{:class="table table-single"}

---

## Flexible schema

A **schema** is the shape of your data: which fields exist and what type each one holds. In a spreadsheet every row has the same columns, whether it needs them or not.

MongoDB has a **flexible schema**. Documents in the same collection don't have to match. Our servo's `specs` holds `torque_kg_cm`, while our battery's `specs` holds `capacity_mah`, and our M3 screws only have `length_mm`. Nobody has to add an empty "torque" box to the screws.

That freedom is lovely, but it cuts both ways. MongoDB won't stop you misspelling a field name either. We will see that bite in lesson 05, and learn how to add guard rails in lesson 11.

---

## Try it Yourself

1. Write a JSON document for one part from your own parts drawer. Include at least one string, one number, one array and one nested object.
2. This JSON has three mistakes. Can you find them?

```json
{
  "name": "N20 Gear Motor",
  "quantity": "8",
  "tags": ["motor", "smars",],
  "location": { "shelf": "D" "bin": 1 }
}
```

3. Put these in order from biggest to smallest: field, collection, server, document, database.

<details><summary>Show answers</summary>

1. Anything like this is great:

```json
{
  "name": "Pimoroni Tiny 2040",
  "quantity": 2,
  "tags": ["microcontroller", "usb-c"],
  "location": { "shelf": "C", "bin": 3 }
}
```

2. `"8"` is a string, not a number, so it should be `8`. There is a stray comma after `"smars"` at the end of the list. And there is a missing comma between `"D"` and `"bin"`. The fixed version:

```json
{
  "name": "N20 Gear Motor",
  "quantity": 8,
  "tags": ["motor", "smars"],
  "location": { "shelf": "D", "bin": 1 }
}
```

3. Server, database, collection, document, field.

</details>

---

## Common Issues

**Problem:** A number is behaving like text (for example, sorting puts `"10"` before `"9"`).  
**Solution:** Remove the quotes so it is stored as a real number: `quantity: 10`, not `quantity: "10"`.  
**Why:** Anything in quotes is a string, and strings sort letter by letter, not by value.

**Problem:** You get an error about an unexpected token when writing JSON.  
**Solution:** Check for a missing comma between pairs, or an extra comma after the last one.  
**Why:** Strict JSON is fussy about commas. `mongosh` is more forgiving about trailing commas, but it is a good habit to get right.

---

## Recap

- JSON writes data as key-value pairs inside curly braces, with strings, numbers, booleans, null, arrays and nested objects
- MongoDB stores data as documents (records) made of fields, grouped into collections, grouped into databases
- BSON is MongoDB's binary version of JSON and adds real dates and ObjectIds
- Every document gets a unique `_id`, and documents in one collection can have different shapes

Next, let's get a MongoDB server running on your computer.
