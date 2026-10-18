---
layout: lesson
title: Building the Parts Bin App
author: Kevin McAleer
type: page
cover: /learn/mongodb/assets/banners/15_building_the_parts_bin_app.jpg
date: 2026-10-18
previous: 14_python_and_pymongo.html
next: 16_keeping_your_data_safe.html
description: Build parts_bin.py, a Python command line app that lists, searches, adds,
  uses, restocks and values the parts in your workshop
percent: 80
duration: 17
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

This is the lesson the whole course has been building towards. We're going to turn our `workshop` database into a proper tool: **`parts_bin.py`**, a command line app you can run any time you grab a part from a drawer.

When we're done, you'll be able to type things like this:

```bash
python parts_bin.py use "SG90 Micro Servo" --count 2
```

```
Used 2 x SG90 Micro Servo. 10 left.
```

Everything in it is something you've already learnt - `find`, `$regex`, `$inc`, aggregation and indexes - glued together with a bit of Python.

---

## Before we start

Work in the `parts_bin` folder from [lesson 14](14_python_and_pymongo), with the virtual environment switched on (`source venv/bin/activate`).

This lesson expects the 13 original parts, plus the **unique index on `name`** from [lesson 13](13_indexes). Our app relies on that index to stop you adding the same part twice. If you have reset your data since then, the index went with it, so run this in `mongosh`:

```javascript
db.parts.createIndex({ name: 1 }, { unique: true })
```

```
name_1
```

Running it again when the index already exists is harmless - MongoDB just hands back the same name.

---

## The plan

Our app has seven **commands**, sometimes called subcommands - the first word after `parts_bin.py` that says what you want to do:

| Command | What it does | MongoDB feature |
|---|---|---|
| `list [--category X]` | Show every part, or one category | `find` and `sort` |
| `find TEXT` | Search names and tags | `$regex` and `$or` |
| `add NAME ...` | Add a new part | `insert_one` and a unique index |
| `use NAME [--count N]` | Take parts out of stock | `$inc` with a `$gte` guard |
| `restock NAME COUNT` | Put parts back | `$inc` |
| `low [--threshold N]` | What's running low? | `$lt` |
| `value` | What's it all worth? | Aggregation |
{:class="table table-single"}

To read those words from the terminal we'll use **argparse**, a module that comes with Python. It turns the words you type after the script name (called **arguments**) into a tidy Python object, and writes a `--help` page for us for free.

---

## Step 1 - the skeleton and the list command

Create `parts_bin.py` with this. It's a complete, working app with just one command:

```python
"""Parts Bin - a command line stock tracker for your workshop."""

import argparse
import os
import re
import sys
from datetime import datetime, timezone

from pymongo import MongoClient, ReturnDocument
from pymongo.errors import DuplicateKeyError, ServerSelectionTimeoutError

MONGODB_URI = os.environ.get("MONGODB_URI", "mongodb://localhost:27017")
DB_NAME = "workshop"


def get_parts_collection():
    """Connect to MongoDB and return the parts collection."""
    client = MongoClient(MONGODB_URI, serverSelectionTimeoutMS=2000)
    try:
        client.admin.command("ping")
    except ServerSelectionTimeoutError:
        print(f"Could not reach MongoDB at {MONGODB_URI}")
        print("Is the server running? Try: docker start mongodb")
        sys.exit(1)
    return client[DB_NAME]["parts"]


def print_parts(found):
    """Print a list of part documents as a neat table."""
    found = list(found)
    if not found:
        print("No parts found.")
        return

    print(f"{'Name':<30} {'Category':<10} {'Qty':>4} {'Price':>7}  Bin")
    print("-" * 59)
    for part in found:
        bin_label = f"{part['location']['shelf']}{part['location']['bin']}"
        print(
            f"{part['name']:<30} {part['category']:<10} "
            f"{part['quantity']:>4} {part['price']:>7.2f}  {bin_label}"
        )
    print(f"\n{len(found)} part(s)")


def cmd_list(parts, args):
    query = {}
    if args.category:
        query["category"] = args.category
    print_parts(parts.find(query).sort([("location.shelf", 1), ("location.bin", 1)]))


def build_parser():
    parser = argparse.ArgumentParser(
        prog="parts_bin.py",
        description="Keep track of the parts in your workshop.",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("list", help="list every part")
    p.add_argument("--category", help="only show this category")
    p.set_defaults(func=cmd_list)

    return parser


def main():
    args = build_parser().parse_args()
    parts = get_parts_collection()
    args.func(parts, args)


if __name__ == "__main__":
    main()
```

We've put every import we'll need at the top now, so you don't have to come back for them later. Let's walk through the new ideas.

**`get_parts_collection()`** is the safe connection from lesson 14, wrapped in a function. It reads `MONGODB_URI` from the environment, so Atlas users don't need to change any code.

**`print_parts()`** makes the table. Inside an **f-string** (a string starting with `f`), `{part['name']:<30}` means "put the name here, left aligned, padded to 30 characters". `>4` means right aligned in 4 characters, and `>7.2f` means right aligned with exactly two decimal places - perfect for prices. That padding is what makes the columns line up.

**`cmd_list()`** builds a filter. With no `--category` it's `{}`, which matches everything. We sort by shelf and then bin, so the list reads in the same order as walking along your shelves.

**`build_parser()`** is where argparse lives. Each `sub.add_parser(...)` creates a command, `add_argument` describes what it accepts, and `set_defaults(func=cmd_list)` says "when someone types `list`, run `cmd_list`". `main()` then just calls whichever function was chosen. Adding a command later means writing one function and three or four parser lines.

Let's try it:

```bash
python parts_bin.py list
```

```
Name                           Category    Qty   Price  Bin
-----------------------------------------------------------
SG90 Micro Servo               servo        12    2.50  A1
MG996R Servo                   servo         4    6.99  A2
HC-SR04 Ultrasonic Sensor      sensor        6    1.80  B1
VL53L0X Time-of-Flight Sensor  sensor        4    4.99  B2
Raspberry Pi Pico W            board         5    6.30  C1
Raspberry Pi 5 4GB             board         1   55.00  C2
N20 Gear Motor                 motor         8    3.20  D1
L298N Motor Driver             driver        3    2.99  D2
DRV8833 Motor Driver           driver        2    4.50  D3
PCA9685 Servo Driver           driver        2    5.99  D4
18650 Battery                  power         6    3.50  E1
WS2812B LED Strip 1m           lighting      0    7.50  E2
M3 Screws (pack of 100)        hardware      3    4.00  F1

13 part(s)
```

```bash
python parts_bin.py list --category driver
```

```
Name                           Category    Qty   Price  Bin
-----------------------------------------------------------
L298N Motor Driver             driver        3    2.99  D2
DRV8833 Motor Driver           driver        2    4.50  D3
PCA9685 Servo Driver           driver        2    5.99  D4

3 part(s)
```

---

## Step 2 - searching with find

Add this function under `cmd_list`:

```python
def cmd_find(parts, args):
    pattern = {"$regex": re.escape(args.text), "$options": "i"}
    query = {"$or": [{"name": pattern}, {"tags": pattern}]}
    print_parts(parts.find(query).sort("name", 1))
```

And these lines in `build_parser()`, just above `return parser`:

```python
    p = sub.add_parser("find", help="search names and tags")
    p.add_argument("text", help="text to search for")
    p.set_defaults(func=cmd_find)
```

This is the case insensitive `$regex` from [lesson 07](07_query_operators), with `$or` so a match in the name *or* the tags counts. A regex on an array field like `tags` matches if any item in the list matches.

**`re.escape()`** is a safety step. Some characters, like `(` and `.`, have special meanings in a regex. `re.escape` turns them into plain characters, so searching for `(pack` doesn't cause an error.

You might wonder why we don't use the text index from lesson 13. A `$text` search matches whole words, so `bme` wouldn't find "BME280". A regex finds any part of a word, which is what you want when you can only half remember a part number.

```bash
python parts_bin.py find servo
```

```
Name                           Category    Qty   Price  Bin
-----------------------------------------------------------
MG996R Servo                   servo         4    6.99  A2
PCA9685 Servo Driver           driver        2    5.99  D4
SG90 Micro Servo               servo        12    2.50  A1

3 part(s)
```

```bash
python parts_bin.py find i2c
```

```
Name                           Category    Qty   Price  Bin
-----------------------------------------------------------
PCA9685 Servo Driver           driver        2    5.99  D4
VL53L0X Time-of-Flight Sensor  sensor        4    4.99  B2

2 part(s)
```

Neither of those has "i2c" in its name - they were found by their tags.

---

## Step 3 - adding parts

```python
def cmd_add(parts, args):
    new_part = {
        "name": args.name,
        "category": args.category,
        "quantity": args.quantity,
        "price": args.price,
        "location": {"shelf": args.shelf, "bin": args.bin},
        "tags": args.tag,
        "specs": {},
        "used_in": [],
        "added": datetime.now(timezone.utc),
    }
    try:
        result = parts.insert_one(new_part)
    except DuplicateKeyError:
        print(f"'{args.name}' is already in the parts bin. Use restock instead.")
        return
    print(f"Added {args.name} with _id {result.inserted_id}")
```

```python
    p = sub.add_parser("add", help="add a new part")
    p.add_argument("name")
    p.add_argument("--category", required=True)
    p.add_argument("--quantity", type=int, required=True)
    p.add_argument("--price", type=float, required=True)
    p.add_argument("--shelf", required=True)
    p.add_argument("--bin", type=int, required=True)
    p.add_argument("--tag", action="append", default=[], help="repeat for more tags")
    p.set_defaults(func=cmd_add)
```

`type=int` and `type=float` make argparse turn the text you typed into numbers, so `quantity` is stored as a number, not the string `"3"`. `action="append"` lets you repeat `--tag` and collects every one into a list - which is exactly the shape our `tags` field needs.

The clever bit is the `try` / `except`. We don't check for duplicates ourselves - the unique index on `name` does it. If the name already exists, MongoDB refuses the insert, PyMongo raises a `DuplicateKeyError`, and we print a friendly message instead.

```bash
python parts_bin.py add "BME280 Sensor" --category sensor --quantity 3 --price 8.50 --shelf B --bin 3 --tag sensor --tag i2c --tag temperature
```

```
Added BME280 Sensor with _id 6ad5252365577e12aaf2d107
```

Notice the quotes around `"BME280 Sensor"`. Without them, your terminal would treat `BME280` and `Sensor` as two separate arguments. Now try adding it again:

```bash
python parts_bin.py add "BME280 Sensor" --category sensor --quantity 3 --price 8.50 --shelf B --bin 3
```

```
'BME280 Sensor' is already in the parts bin. Use restock instead.
```

---

## Step 4 - using and restocking parts

This is the command you'll use most. It has one important rule: **you can't use more parts than you have**.

```python
def cmd_use(parts, args):
    if args.count < 1:
        print("Count must be 1 or more.")
        return

    updated = parts.find_one_and_update(
        {"name": args.name, "quantity": {"$gte": args.count}},
        {
            "$inc": {"quantity": -args.count},
            "$set": {"last_updated": datetime.now(timezone.utc)},
        },
        return_document=ReturnDocument.AFTER,
    )

    if updated:
        print(f"Used {args.count} x {args.name}. {updated['quantity']} left.")
        return

    # Nothing matched - work out why so we can say something helpful
    part = parts.find_one({"name": args.name})
    if part is None:
        print(f"No part called '{args.name}'. Try: find {args.name.split()[0]}")
    else:
        print(f"Not enough stock: only {part['quantity']} x {args.name} left.")
```

Look closely at the filter: `{"name": args.name, "quantity": {"$gte": args.count}}`. It only matches the part **if there are at least that many in stock**. If there aren't, nothing matches and nothing changes. The check and the change happen in one single step on the server, so stock can never go below zero - even if two people use the app at the same moment.

**`find_one_and_update`** is a sibling of `update_one`. It does the update *and* hands back the document, and `ReturnDocument.AFTER` asks for the version after the change, so we can show how many are left. If nothing matched it returns `None`, and we do a quick `find_one` to tell "no such part" apart from "not enough stock".

Restocking is the same idea, without the guard:

```python
def cmd_restock(parts, args):
    if args.count < 1:
        print("Count must be 1 or more.")
        return

    updated = parts.find_one_and_update(
        {"name": args.name},
        {
            "$inc": {"quantity": args.count},
            "$set": {"last_updated": datetime.now(timezone.utc)},
        },
        return_document=ReturnDocument.AFTER,
    )

    if updated:
        print(f"Restocked {args.name}. Now {updated['quantity']} in stock.")
    else:
        print(f"No part called '{args.name}'.")
```

```python
    p = sub.add_parser("use", help="take parts out of stock")
    p.add_argument("name")
    p.add_argument("--count", type=int, default=1)
    p.set_defaults(func=cmd_use)

    p = sub.add_parser("restock", help="put parts back into stock")
    p.add_argument("name")
    p.add_argument("count", type=int)
    p.set_defaults(func=cmd_restock)
```

Let's put it through its paces:

```bash
python parts_bin.py use "SG90 Micro Servo" --count 2
```

```
Used 2 x SG90 Micro Servo. 10 left.
```

```bash
python parts_bin.py use "DRV8833 Motor Driver" --count 5
```

```
Not enough stock: only 2 x DRV8833 Motor Driver left.
```

```bash
python parts_bin.py use "SG90 Servo"
```

```
No part called 'SG90 Servo'. Try: find SG90
```

```bash
python parts_bin.py restock "WS2812B LED Strip 1m" 2
```

```
Restocked WS2812B LED Strip 1m. Now 2 in stock.
```

---

## Step 5 - the low stock report

```python
def cmd_low(parts, args):
    print(f"Parts with fewer than {args.threshold} in stock:\n")
    low = parts.find({"quantity": {"$lt": args.threshold}})
    print_parts(low.sort([("quantity", 1), ("name", 1)]))
```

```python
    p = sub.add_parser("low", help="show parts that are running low")
    p.add_argument("--threshold", type=int, default=3)
    p.set_defaults(func=cmd_low)
```

This is the `quantity < 3` query from lesson 07, with the 3 turned into a setting. Sorting by quantity puts the most urgent parts at the top, and sorting by name second keeps parts with the same quantity in a predictable order.

```bash
python parts_bin.py low
```

```
Parts with fewer than 3 in stock:

Name                           Category    Qty   Price  Bin
-----------------------------------------------------------
Raspberry Pi 5 4GB             board         1   55.00  C2
DRV8833 Motor Driver           driver        2    4.50  D3
PCA9685 Servo Driver           driver        2    5.99  D4
WS2812B LED Strip 1m           lighting      2    7.50  E2

4 part(s)
```

The LED strip is now 2 rather than 0, thanks to our restock. Try `--threshold 5` for a longer shopping list.

---

## Step 6 - the value report

The last command reuses the pipelines from [lesson 12](12_aggregation_basics) exactly, just written with Python's quoted keys and `None` instead of `null`:

```python
def cmd_value(parts, args):
    # The per-category pipeline from lesson 12
    by_category = [
        {"$group": {"_id": "$category",
                    "value": {"$sum": {"$multiply": ["$quantity", "$price"]}}}},
        {"$project": {"value": {"$round": ["$value", 2]}}},
        {"$sort": {"value": -1}},
    ]
    # And the whole-workshop total, also from lesson 12
    totals = [
        {"$group": {"_id": None,
                    "total_value": {"$sum": {"$multiply": ["$quantity", "$price"]}},
                    "total_items": {"$sum": "$quantity"}}},
    ]

    print(f"{'Category':<12} {'Value':>9}")
    print("-" * 22)
    for row in parts.aggregate(by_category):
        print(f"{row['_id']:<12} {row['value']:>9.2f}")
    print("-" * 22)

    summary = list(parts.aggregate(totals))
    if summary:
        total = summary[0]
        print(f"{'Total':<12} {total['total_value']:>9.2f}")
        print(f"\n{total['total_items']} items in the workshop")
```

```python
    p = sub.add_parser("value", help="what is all this stock worth?")
    p.set_defaults(func=cmd_value)
```

`aggregate()` returns a cursor, just like `find()`. The totals pipeline gives back one document, so we turn it into a list and take the first item. The `if summary:` check covers an empty collection, where there would be nothing to total.

```bash
python parts_bin.py value
```

```
Category         Value
----------------------
board            86.50
sensor           56.26
servo            52.96
driver           29.95
motor            25.60
power            21.00
lighting         15.00
hardware         12.00
----------------------
Total           299.27

59 items in the workshop
```

On the original seed data this reads 263.77 and 56 items, matching lesson 12. Ours has gone up because we added three BME280s and two LED strips, and used two servos.

---

## The finished app

Here is the whole of `parts_bin.py` in one piece:

```python
"""Parts Bin - a command line stock tracker for your workshop."""

import argparse
import os
import re
import sys
from datetime import datetime, timezone

from pymongo import MongoClient, ReturnDocument
from pymongo.errors import DuplicateKeyError, ServerSelectionTimeoutError

MONGODB_URI = os.environ.get("MONGODB_URI", "mongodb://localhost:27017")
DB_NAME = "workshop"


def get_parts_collection():
    """Connect to MongoDB and return the parts collection."""
    client = MongoClient(MONGODB_URI, serverSelectionTimeoutMS=2000)
    try:
        client.admin.command("ping")
    except ServerSelectionTimeoutError:
        print(f"Could not reach MongoDB at {MONGODB_URI}")
        print("Is the server running? Try: docker start mongodb")
        sys.exit(1)
    return client[DB_NAME]["parts"]


def print_parts(found):
    """Print a list of part documents as a neat table."""
    found = list(found)
    if not found:
        print("No parts found.")
        return

    print(f"{'Name':<30} {'Category':<10} {'Qty':>4} {'Price':>7}  Bin")
    print("-" * 59)
    for part in found:
        bin_label = f"{part['location']['shelf']}{part['location']['bin']}"
        print(
            f"{part['name']:<30} {part['category']:<10} "
            f"{part['quantity']:>4} {part['price']:>7.2f}  {bin_label}"
        )
    print(f"\n{len(found)} part(s)")


def cmd_list(parts, args):
    query = {}
    if args.category:
        query["category"] = args.category
    print_parts(parts.find(query).sort([("location.shelf", 1), ("location.bin", 1)]))


def cmd_find(parts, args):
    pattern = {"$regex": re.escape(args.text), "$options": "i"}
    query = {"$or": [{"name": pattern}, {"tags": pattern}]}
    print_parts(parts.find(query).sort("name", 1))


def cmd_add(parts, args):
    new_part = {
        "name": args.name,
        "category": args.category,
        "quantity": args.quantity,
        "price": args.price,
        "location": {"shelf": args.shelf, "bin": args.bin},
        "tags": args.tag,
        "specs": {},
        "used_in": [],
        "added": datetime.now(timezone.utc),
    }
    try:
        result = parts.insert_one(new_part)
    except DuplicateKeyError:
        print(f"'{args.name}' is already in the parts bin. Use restock instead.")
        return
    print(f"Added {args.name} with _id {result.inserted_id}")


def cmd_use(parts, args):
    if args.count < 1:
        print("Count must be 1 or more.")
        return

    updated = parts.find_one_and_update(
        {"name": args.name, "quantity": {"$gte": args.count}},
        {
            "$inc": {"quantity": -args.count},
            "$set": {"last_updated": datetime.now(timezone.utc)},
        },
        return_document=ReturnDocument.AFTER,
    )

    if updated:
        print(f"Used {args.count} x {args.name}. {updated['quantity']} left.")
        return

    # Nothing matched - work out why so we can say something helpful
    part = parts.find_one({"name": args.name})
    if part is None:
        print(f"No part called '{args.name}'. Try: find {args.name.split()[0]}")
    else:
        print(f"Not enough stock: only {part['quantity']} x {args.name} left.")


def cmd_restock(parts, args):
    if args.count < 1:
        print("Count must be 1 or more.")
        return

    updated = parts.find_one_and_update(
        {"name": args.name},
        {
            "$inc": {"quantity": args.count},
            "$set": {"last_updated": datetime.now(timezone.utc)},
        },
        return_document=ReturnDocument.AFTER,
    )

    if updated:
        print(f"Restocked {args.name}. Now {updated['quantity']} in stock.")
    else:
        print(f"No part called '{args.name}'.")


def cmd_low(parts, args):
    print(f"Parts with fewer than {args.threshold} in stock:\n")
    low = parts.find({"quantity": {"$lt": args.threshold}})
    print_parts(low.sort([("quantity", 1), ("name", 1)]))


def cmd_value(parts, args):
    # The per-category pipeline from lesson 12
    by_category = [
        {"$group": {"_id": "$category",
                    "value": {"$sum": {"$multiply": ["$quantity", "$price"]}}}},
        {"$project": {"value": {"$round": ["$value", 2]}}},
        {"$sort": {"value": -1}},
    ]
    # And the whole-workshop total, also from lesson 12
    totals = [
        {"$group": {"_id": None,
                    "total_value": {"$sum": {"$multiply": ["$quantity", "$price"]}},
                    "total_items": {"$sum": "$quantity"}}},
    ]

    print(f"{'Category':<12} {'Value':>9}")
    print("-" * 22)
    for row in parts.aggregate(by_category):
        print(f"{row['_id']:<12} {row['value']:>9.2f}")
    print("-" * 22)

    summary = list(parts.aggregate(totals))
    if summary:
        total = summary[0]
        print(f"{'Total':<12} {total['total_value']:>9.2f}")
        print(f"\n{total['total_items']} items in the workshop")


def build_parser():
    parser = argparse.ArgumentParser(
        prog="parts_bin.py",
        description="Keep track of the parts in your workshop.",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("list", help="list every part")
    p.add_argument("--category", help="only show this category")
    p.set_defaults(func=cmd_list)

    p = sub.add_parser("find", help="search names and tags")
    p.add_argument("text", help="text to search for")
    p.set_defaults(func=cmd_find)

    p = sub.add_parser("add", help="add a new part")
    p.add_argument("name")
    p.add_argument("--category", required=True)
    p.add_argument("--quantity", type=int, required=True)
    p.add_argument("--price", type=float, required=True)
    p.add_argument("--shelf", required=True)
    p.add_argument("--bin", type=int, required=True)
    p.add_argument("--tag", action="append", default=[], help="repeat for more tags")
    p.set_defaults(func=cmd_add)

    p = sub.add_parser("use", help="take parts out of stock")
    p.add_argument("name")
    p.add_argument("--count", type=int, default=1)
    p.set_defaults(func=cmd_use)

    p = sub.add_parser("restock", help="put parts back into stock")
    p.add_argument("name")
    p.add_argument("count", type=int)
    p.set_defaults(func=cmd_restock)

    p = sub.add_parser("low", help="show parts that are running low")
    p.add_argument("--threshold", type=int, default=3)
    p.set_defaults(func=cmd_low)

    p = sub.add_parser("value", help="what is all this stock worth?")
    p.set_defaults(func=cmd_value)

    return parser


def main():
    args = build_parser().parse_args()
    parts = get_parts_collection()
    args.func(parts, args)


if __name__ == "__main__":
    main()
```

And the help page argparse wrote for us:

```bash
python parts_bin.py --help
```

```
usage: parts_bin.py [-h] {list,find,add,use,restock,low,value} ...

Keep track of the parts in your workshop.

positional arguments:
  {list,find,add,use,restock,low,value}
    list                list every part
    find                search names and tags
    add                 add a new part
    use                 take parts out of stock
    restock             put parts back into stock
    low                 show parts that are running low
    value               what is all this stock worth?

options:
  -h, --help            show this help message and exit
```

Each command has its own help too, such as `python parts_bin.py add --help`.

**How your data has changed:** you now have 14 parts. The BME280 Sensor is new, the SG90 servos are down to 10, the LED strip is up to 2, and those two parts have a `last_updated` date. Keep it - the next lesson backs it up.

---

## Try it Yourself

1. Add a `delete NAME` command that removes a part with `delete_one` and reports whether anything was deleted.
2. Make `list` print only the parts used in one project, with a `--project SMARS` option.
3. Add a `--shelf` option to `low`, so you can check one shelf at a time.

<details><summary>Show answers</summary>

1. One function and three parser lines:

```python
def cmd_delete(parts, args):
    result = parts.delete_one({"name": args.name})
    if result.deleted_count:
        print(f"Deleted {args.name}.")
    else:
        print(f"No part called '{args.name}'.")
```

```python
    p = sub.add_parser("delete", help="remove a part completely")
    p.add_argument("name")
    p.set_defaults(func=cmd_delete)
```

2. Add `p.add_argument("--project")` to the `list` parser, then in `cmd_list` add `if args.project: query["used_in"] = args.project`. A plain equality matches any item in the `used_in` list.

3. Add `p.add_argument("--shelf")` to the `low` parser, then build the filter in steps: start with `query = {"quantity": {"$lt": args.threshold}}` and add `query["location.shelf"] = args.shelf` when a shelf was given.

</details>

---

## Common Issues

**Problem:** `parts_bin.py: error: unrecognized arguments: Micro Servo`  
**Solution:** Put quotes around names with spaces: `python parts_bin.py use "SG90 Micro Servo"`.  
**Why:** Your terminal splits what you type at every space, so without quotes argparse receives three separate words and only expects one name.

**Problem:** `add` happily adds the same part twice.  
**Solution:** Recreate the unique index in `mongosh`: `db.parts.createIndex({ name: 1 }, { unique: true })`. You'll need to delete the duplicate first, or the index can't be built.  
**Why:** The duplicate check is done by the index. Dropping the collection to reset it removes its indexes too.

**Problem:** `use` says "No part called ..." for a part you can see in `list`.  
**Solution:** Copy the name exactly, including capitals - or use `find` to check the spelling.  
**Why:** `use` and `restock` match the name exactly, so "sg90 micro servo" isn't the same as "SG90 Micro Servo". Only `find` ignores case.

**Problem:** `KeyError: 'location'` when running `list`.  
**Solution:** Give the offending part a location with `updateOne` and `$set` in `mongosh`.  
**Why:** A part added by hand without a `location` field breaks `print_parts`, which expects every part to have one. Schema validation from [lesson 11](11_designing_your_data) can make `location` required.

---

## Recap

- `argparse` subcommands turn one script into a whole toolbox, with free `--help` pages.
- f-string padding like `:<30` and `:>7.2f` lines up tables neatly.
- A unique index plus `DuplicateKeyError` stops duplicate parts without any extra queries.
- Putting `"quantity": {"$gte": count}` in the filter makes "use" safe in one step on the server.
- `aggregate()` runs the same pipelines from `mongosh`, with quoted keys and `None`.

Next, let's make sure all that hard-won data is safe - with backups and a locked front door.
