---
title: Python and PyMongo
description: Connect to MongoDB from Python with PyMongo and read, add, change and delete parts from a script
layout: lesson
type: page
cover: /learn/mongodb/assets/banners/14_python_and_pymongo.jpg
date_updated: 2026-10-18
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

So far we have talked to MongoDB by typing into `mongosh`. That is brilliant for exploring, but nobody wants to type `db.parts.updateOne(...)` every time they pull a servo out of a drawer. The real power comes when a **program** talks to the database for you.

In this lesson we connect to the Parts Bin from **Python**, using a library called **PyMongo**. By the end you will be able to find, add, change and delete parts from a script - which is everything we need to build the full Parts Bin app in the next lesson.

If Python is new to you, the [Python for beginners course](/learn/python/) covers everything used here. If your data has drifted from the original 13 parts, reset it first (`db.parts.drop()` then re-run the `insertMany` from [lesson 05](05_inserting_documents)).

---

## What is PyMongo?

A **library** is a bundle of ready-made code you can use in your own programs. **PyMongo** is the official MongoDB library for Python. It is a **driver** - the bit of software that knows how to speak MongoDB's language over the network, so your Python code can act as a **client** to the MongoDB **server**, exactly like `mongosh` does.

---

## Setting up a virtual environment

A **virtual environment** is a private folder of Python libraries for one project, so installing things for this project never breaks another one. Let's make a project folder and a virtual environment inside it:

```bash
mkdir parts_bin
cd parts_bin
python3 -m venv venv
source venv/bin/activate
```

On Windows, the last line is `venv\Scripts\activate` instead. Your prompt now starts with `(venv)`, which tells you the environment is switched on.

Now install PyMongo:

```bash
pip install pymongo
```

And check it worked:

```bash
python -c "import pymongo; print(pymongo.version)"
```

```
4.19.0
```

Your number may be newer - anything starting with 4 is fine.

---

## Connecting to the server

Create a file called `connect.py`:

```python
from pymongo import MongoClient

client = MongoClient("mongodb://localhost:27017")

result = client.admin.command("ping")
print(result)
print(client.list_database_names())
```

`MongoClient` is the object that holds the connection to the server. The text `"mongodb://localhost:27017"` is a **connection string** - an address that says "MongoDB, on this computer, on port 27017". The `ping` command is the same one we used in [lesson 03](03_installing_mongodb).

```bash
python connect.py
```

```
{'ok': 1.0}
['admin', 'config', 'local', 'workshop']
```

There is our `workshop` database. Python is talking to MongoDB.

---

## Databases and collections are like dictionaries

In Python, a **dictionary** is a collection of `key: value` pairs, and you look things up with square brackets: `person["name"]`. PyMongo borrows that idea. You get a database from the client, and a collection from the database, with square brackets:

```python
db = client["workshop"]
parts = db["parts"]
```

You will also see `client.workshop.parts`, which does exactly the same thing. Square brackets are safer because they work with any name, even ones containing a dash.

---

## Reading one document with find_one

`find_one()` is Python's version of `findOne()`. Create `first_part.py`:

```python
from pymongo import MongoClient

client = MongoClient("mongodb://localhost:27017")
db = client["workshop"]
parts = db["parts"]

part = parts.find_one({"name": "SG90 Micro Servo"})
print(part)
print(type(part))
print(part["quantity"])
print(part["location"]["shelf"])
```

```
{'_id': ObjectId('6ad523460f65ec38569405a8'), 'name': 'SG90 Micro Servo', 'category': 'servo', 'quantity': 12, 'price': 2.5, 'location': {'shelf': 'A', 'bin': 1}, 'tags': ['servo', '5v', 'smars'], 'specs': {'torque_kg_cm': 1.8, 'voltage': '4.8-6V'}, 'used_in': ['SMARS', 'BurgerBot'], 'added': datetime.datetime(2026, 1, 12, 0, 0)}
<class 'dict'>
12
A
```

This is the best bit of PyMongo: **a MongoDB document comes back as a plain Python dictionary**. Nested documents become nested dictionaries, arrays become Python lists, and dates become Python `datetime` objects. No translation needed. If nothing matches, `find_one()` returns `None`.

---

## Reading many documents with find

`find()` does not hand you a list straight away. It gives you a **cursor** - a pointer that fetches the matching documents from the server as you loop over them. Create `list_drivers.py`:

```python
from pymongo import MongoClient

client = MongoClient("mongodb://localhost:27017")
parts = client["workshop"]["parts"]

cursor = parts.find({"category": "driver"})
print(cursor)

for part in cursor:
    print(part["name"], "-", part["quantity"], "in stock")

print(parts.count_documents({"category": "driver"}))
```

```
<pymongo.synchronous.cursor.Cursor object at 0x7ff792688590>
L298N Motor Driver - 3 in stock
DRV8833 Motor Driver - 2 in stock
PCA9685 Servo Driver - 2 in stock
3
```

Printing the cursor itself just shows the object, not your data - you have to loop over it. Notice `count_documents` uses an underscore. PyMongo method names use Python's `snake_case` style, so `findOne` becomes `find_one`, `insertOne` becomes `insert_one`, and so on.

Projections and sorting work just like in `mongosh`. Here is the low stock query from [lesson 07](07_query_operators):

```python
low = parts.find(
    {"quantity": {"$lt": 3}},
    {"_id": 0, "name": 1, "quantity": 1},
).sort("quantity", 1)

for part in low:
    print(part)
```

```
{'name': 'WS2812B LED Strip 1m', 'quantity': 0}
{'name': 'Raspberry Pi 5 4GB', 'quantity': 1}
{'name': 'DRV8833 Motor Driver', 'quantity': 2}
{'name': 'PCA9685 Servo Driver', 'quantity': 2}
```

One thing that catches everyone: **a cursor can only be looped over once**. After that it is used up:

```python
cursor = parts.find({"category": "servo"})
print(len(list(cursor)))
print(len(list(cursor)))
```

```
2
0
```

If you need the results more than once, turn them into a list first with `list(parts.find(...))`.

---

## Adding a part with insert_one

Let's add a new sensor. Create `add_part.py`:

```python
from datetime import datetime, timezone
from pymongo import MongoClient

client = MongoClient("mongodb://localhost:27017")
parts = client["workshop"]["parts"]

new_part = {
    "name": "BME280 Sensor",
    "category": "sensor",
    "quantity": 3,
    "price": 8.50,
    "location": {"shelf": "B", "bin": 3},
    "tags": ["sensor", "i2c", "temperature"],
    "specs": {"voltage": "3.3V"},
    "used_in": [],
    "in_stock": True,
    "notes": None,
    "added": datetime.now(timezone.utc),
}

result = parts.insert_one(new_part)
print(result.inserted_id)
print(new_part["_id"])
```

```
6ad5236f07ad9efcd12fa090
6ad5236f07ad9efcd12fa090
```

`insert_one` returns a result object, and `result.inserted_id` is the new `ObjectId`. PyMongo also quietly adds the `_id` to your dictionary, which is why the second line prints the same thing.

Look at the Python values: `True`, `None` and a `datetime`. In `mongosh` those were `true`, `null` and `new Date()`. If we peek at the part in `mongosh`, PyMongo has translated them for us:

```javascript
db.parts.findOne({ name: "BME280 Sensor" }, { _id: 0, in_stock: 1, notes: 1, added: 1 })
```

```
{
  in_stock: true,
  notes: null,
  added: ISODate('2026-10-18T19:52:15.655Z')
}
```

---

## Changing a part with update_one

Using a sensor means taking one off the quantity, just like `$inc` in [lesson 08](08_updating_documents). Create `use_part.py`:

```python
from pymongo import MongoClient

client = MongoClient("mongodb://localhost:27017")
parts = client["workshop"]["parts"]

result = parts.update_one(
    {"name": "BME280 Sensor"},
    {"$inc": {"quantity": -1}},
)
print("matched:", result.matched_count)
print("modified:", result.modified_count)

part = parts.find_one({"name": "BME280 Sensor"})
print(part["quantity"])
```

```
matched: 1
modified: 1
2
```

Always check `matched_count`. Here is the same update with a lower case "s" in "sensor":

```
matched: 0
modified: 0
```

No error at all - MongoDB simply found nothing to change. Your program has to notice that and tell the user.

---

## Looking up by _id with ObjectId

An `_id` is not a string, it is an `ObjectId`. If you copy one out of your terminal as text, you must wrap it before searching. Create `by_id.py`, pasting in the id your `add_part.py` printed:

```python
from bson import ObjectId
from pymongo import MongoClient

client = MongoClient("mongodb://localhost:27017")
parts = client["workshop"]["parts"]

part_id = "6ad5236f07ad9efcd12fa090"

print(parts.find_one({"_id": part_id}))
print(parts.find_one({"_id": ObjectId(part_id)})["name"])
```

```
None
BME280 Sensor
```

The plain string finds nothing. `ObjectId` comes from `bson`, a module that is installed as part of PyMongo. **BSON** is the format MongoDB stores documents in, which we met in [lesson 02](02_documents_and_collections).

---

## Removing a part with delete_one

Finally, let's tidy up so the data is back to the original 13 parts. Create `remove_part.py`:

```python
from pymongo import MongoClient

client = MongoClient("mongodb://localhost:27017")
parts = client["workshop"]["parts"]

result = parts.delete_one({"name": "BME280 Sensor"})
print("deleted:", result.deleted_count)
print(parts.count_documents({}))
```

```
deleted: 1
13
```

---

## Python versus mongosh

Most of what you learnt in `mongosh` carries straight over. The differences are small but they trip everyone up at least once:

| mongosh | Python | Why |
|---|---|---|
| `{ name: "SG90" }` | `{"name": "SG90"}` | Python dictionary keys must be in quotes |
| `{ $inc: { quantity: -1 } }` | `{"$inc": {"quantity": -1}}` | Operators are just keys, so they need quotes too |
| `true` / `false` / `null` | `True` / `False` / `None` | Python spellings, with capital letters |
| `new Date()` | `datetime.now(timezone.utc)` | Import `datetime` and `timezone` from `datetime` |
| `ObjectId("...")` | `ObjectId("...")` | Import it with `from bson import ObjectId` |
| `findOne`, `insertOne`, `countDocuments` | `find_one`, `insert_one`, `count_documents` | Python uses snake_case names |
| `.sort({ quantity: 1 })` | `.sort("quantity", 1)` | One field as two arguments, or a list of pairs for several |
{:class="table table-single"}

---

## Handling a server that is not running

Creating a `MongoClient` does not actually connect. PyMongo waits until the first real command, and if the server is not there it keeps trying for **30 seconds** before giving up. That feels like your program has frozen. Let's make it fail fast, with a friendly message. Create `safe_connect.py`:

```python
import os
import sys

from pymongo import MongoClient
from pymongo.errors import ServerSelectionTimeoutError

MONGODB_URI = os.environ.get("MONGODB_URI", "mongodb://localhost:27017")

client = MongoClient(MONGODB_URI, serverSelectionTimeoutMS=2000)

try:
    client.admin.command("ping")
except ServerSelectionTimeoutError:
    print(f"Could not reach MongoDB at {MONGODB_URI}")
    print("Is the server running? Try: docker start mongodb")
    sys.exit(1)

print("Connected!")
```

`serverSelectionTimeoutMS=2000` means "give up after 2000 milliseconds" (two seconds). The `try` / `except` block catches the error and prints something helpful instead of a wall of red text.

```bash
python safe_connect.py
```

```
Connected!
```

### Keeping the connection string in an environment variable

The line `os.environ.get("MONGODB_URI", "mongodb://localhost:27017")` reads an **environment variable** - a named setting that lives in your terminal session rather than in your code. If `MONGODB_URI` is not set, we fall back to the local server.

This matters for **Atlas** users, whose connection string contains a username and password. Passwords should never be typed into your code, because code gets shared and uploaded to GitHub. Instead, set the variable in the terminal:

```bash
export MONGODB_URI="mongodb+srv://kev:yourpassword@cluster0.abcde.mongodb.net/"
python safe_connect.py
```

On Windows PowerShell use `$env:MONGODB_URI = "mongodb+srv://..."` instead. We can see the error handling working by pointing it at a port where nothing is listening:

```bash
MONGODB_URI="mongodb://localhost:27018" python safe_connect.py
```

```
Could not reach MongoDB at mongodb://localhost:27018
Is the server running? Try: docker start mongodb
```

---

## Try it Yourself

1. Write a script that prints the name and price of every part on shelf "D", most expensive first.
2. Add the BME280 again, then write a script that uses `update_one` with `"$push"` to add `"Weather Station"` to its `used_in` list. Delete it again when you are done.
3. Print `type(part["added"])` for any part. What kind of Python object is it?

<details><summary>Show answers</summary>

1. Dot notation goes in quotes, just like every other key:

```python
for part in parts.find({"location.shelf": "D"}).sort("price", -1):
    print(part["name"], part["price"])
```

2. Operators are quoted keys:

```python
parts.update_one({"name": "BME280 Sensor"}, {"$push": {"used_in": "Weather Station"}})
```

3. `<class 'datetime.datetime'>` - a normal Python date and time you can compare and format.

</details>

---

## Common Issues

**Problem:** `SyntaxError: invalid syntax` pointing at a `$`.  
**Solution:** Put quotes around the operator: `{"$inc": {"quantity": -1}}`, not `{$inc: ...}`.  
**Why:** In Python an operator is just a dictionary key, and keys that are text must be quoted.

**Problem:** `NameError: name 'true' is not defined. Did you mean: 'True'?`  
**Solution:** Use `True`, `False` and `None` in Python.  
**Why:** `true`, `false` and `null` are JavaScript spellings that only work in `mongosh`.

**Problem:** `find_one({"_id": "6ad5..."})` returns `None` even though the id is right.  
**Solution:** Wrap it: `{"_id": ObjectId("6ad5...")}` after `from bson import ObjectId`.  
**Why:** An ObjectId is its own type. A string with the same characters is not equal to it.

**Problem:** `ModuleNotFoundError: No module named 'pymongo'`.  
**Solution:** Activate your virtual environment (`source venv/bin/activate`) and run `pip install pymongo` again.  
**Why:** PyMongo was installed into the virtual environment, so Python can only see it while that environment is switched on.

**Problem:** Dates come back without a time zone, like `datetime.datetime(2026, 1, 12, 0, 0)`.  
**Solution:** Create the client with `MongoClient(uri, tz_aware=True)` if you need time zone aware dates.  
**Why:** MongoDB always stores dates in UTC. PyMongo hands them back without a time zone label unless you ask.

---

## Recap

- PyMongo is the official Python driver. Install it with `pip install pymongo` inside a virtual environment.
- `client["workshop"]["parts"]` gets you a collection, and documents come back as Python dictionaries.
- `find()` returns a cursor you loop over once; `find_one()` returns a dictionary or `None`.
- Operators need quotes, and Python uses `True`, `False`, `None`, `datetime` and `ObjectId`.
- Use `serverSelectionTimeoutMS` and an environment variable for a fast, safe connection.

Next up, we put all of this together and build the Parts Bin command line app.
