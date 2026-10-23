---
layout: lesson
title: Keeping Your Data Safe
author: Kevin McAleer
type: page
cover: /learn/mongodb/assets/banners/16_keeping_your_data_safe.jpg
date: 2026-10-18
previous: 15_building_the_parts_bin_app.html
next: 17_summary.html
description: Back up and restore the Parts Bin with mongodump and mongorestore, export
  it to spreadsheets, and lock MongoDB down with users and passwords
percent: 85
duration: 12
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

Our Parts Bin now holds real information about real stock. That makes it worth protecting from two very different dangers:

1. **Losing it** - a mistyped `deleteMany({})`, a dead SD card, or a Docker volume removed by accident.
2. **Someone else getting at it** - a database left open on the internet with no password.

In this lesson we'll fix both. We'll make **backups** (a copy of your data you can put back later), export the parts to a spreadsheet, and then add a username and password to MongoDB.

We'll work with the data as lesson 15 left it: 14 parts, including the BME280 Sensor, plus the indexes from lesson 13.

---

## The MongoDB Database Tools

MongoDB comes with a set of small command line programs called the **Database Tools**. We need four of them:

| Tool | What it does |
|---|---|
| `mongodump` | Copies a database into backup files |
| `mongorestore` | Puts a backup back into the database |
| `mongoexport` | Writes a collection out as JSON or CSV |
| `mongoimport` | Reads JSON or CSV into a collection |
{:class="table table-single"}

These are run from your normal terminal, **not** inside `mongosh`.

- **Docker users:** the `mongo:8.0` image already includes the tools, so we run them inside the container with `docker exec`.
- **Native install or Atlas users:** download the "MongoDB Database Tools" from the [MongoDB download centre](https://www.mongodb.com/try/download/database-tools), or on macOS run `brew install mongodb-database-tools`.

---

## Backing up with mongodump

`mongodump` connects to the server like any other client and saves everything in a database to files. Let's back up the workshop:

```bash
mongodump --db workshop --out backup
```

```
2026-10-18T20:02:43.691+0000	writing `workshop.parts` to `backup/workshop/parts.bson`
2026-10-18T20:02:43.693+0000	done dumping `workshop.parts` (14 documents)
```

That made a folder called `backup`, with one folder per database inside:

```
backup/
  workshop/
    parts.bson
    parts.metadata.json
    prelude.json
```

`parts.bson` holds the documents themselves, in the same **BSON** format MongoDB uses internally, so dates and ObjectIds survive perfectly. `parts.metadata.json` remembers the collection's settings, including all its indexes - so the unique index on `name` is backed up too.

### Backing up from Docker

Inside Docker there are two neat ways to do it. The easiest is to ask for a single compressed file and send it straight out of the container to your computer:

```bash
docker exec mongodb mongodump --db workshop --archive --gzip > workshop-backup.gz
```

`--archive` with no file name writes the whole backup as one stream of data to the screen, `--gzip` squashes it, and `>` catches that stream and saves it in a file on your computer. The progress messages still appear in your terminal, but they don't end up in the file. Our 14 parts make a file of under 2 KB.

The other way is to make a backup folder inside the container, then copy it out with `docker cp`:

```bash
docker exec mongodb mongodump --db workshop --out /data/backup
docker cp mongodb:/data/backup ./backup
```

Either way, the important part is that the backup ends up **outside** the container and its volume. A backup that lives in the same place as the data disappears with it.

---

## Restoring with mongorestore

A backup you've never restored is just a hopeful guess. Let's practise a disaster. In `mongosh`:

```javascript
db.parts.drop()
db.parts.countDocuments()
```

```
true
0
```

Gone. Now, back in your terminal:

```bash
mongorestore --drop backup/
```

```
2026-10-18T20:02:45.069+0000	preparing collections to restore from
2026-10-18T20:02:45.069+0000	don't know what to do with file `backup/workshop/prelude.json`, skipping...
2026-10-18T20:02:45.069+0000	reading metadata for `workshop.parts` from `backup/workshop/parts.metadata.json`
2026-10-18T20:02:45.076+0000	restoring `workshop.parts` from `backup/workshop/parts.bson`
2026-10-18T20:02:45.087+0000	finished restoring `workshop.parts` (14 documents, 0 failures)
2026-10-18T20:02:45.087+0000	restoring indexes for collection `workshop.parts` from metadata
...
2026-10-18T20:02:45.107+0000	14 document(s) restored successfully. 0 document(s) failed to restore.
```

We've trimmed three long lines describing each index. The "don't know what to do with prelude.json" message is harmless - that file just records which version made the backup. Check in `mongosh` and all 14 parts are back, with their indexes.

**`--drop`** tells `mongorestore` to drop each collection before restoring it, so you get exactly what was in the backup. Without it, `mongorestore` only adds documents, and any whose `_id` already exists fail with a duplicate key error.

To restore the single-file Docker backup, send the file *into* the container. The `-i` lets `docker exec` read from your keyboard or, here, from a file:

```bash
docker exec -i mongodb mongorestore --archive --gzip --drop < workshop-backup.gz
```

---

## Exporting to a spreadsheet with mongoexport

Backups are for MongoDB to read. Sometimes you want something *people* can read - like a stock list to open in a spreadsheet. That's what `mongoexport` is for.

**CSV** (comma separated values) is a plain text table that every spreadsheet program can open. Because a CSV is flat, we have to say which fields we want, using dot notation for the nested ones:

```bash
mongoexport --db workshop --collection parts --type csv --fields name,category,quantity,price,location.shelf,location.bin --out parts.csv
```

```
2026-10-18T19:55:33.455+0000	connected to: mongodb://localhost/
2026-10-18T19:55:33.457+0000	exported 14 records
```

Here's `parts.csv`:

```
name,category,quantity,price,location.shelf,location.bin
SG90 Micro Servo,servo,10,2.5,A,1
MG996R Servo,servo,4,6.99,A,2
HC-SR04 Ultrasonic Sensor,sensor,6,1.8,B,1
VL53L0X Time-of-Flight Sensor,sensor,4,4.99,B,2
Raspberry Pi Pico W,board,5,6.3,C,1
Raspberry Pi 5 4GB,board,1,55,C,2
N20 Gear Motor,motor,8,3.2,D,1
L298N Motor Driver,driver,3,2.99,D,2
DRV8833 Motor Driver,driver,2,4.5,D,3
PCA9685 Servo Driver,driver,2,5.99,D,4
18650 Battery,power,6,3.5,E,1
WS2812B LED Strip 1m,lighting,2,7.5,E,2
M3 Screws (pack of 100),hardware,3,4,F,1
BME280 Sensor,sensor,3,8.5,B,3
```

Leave off `--type csv` and you get JSON instead, one document per line, with special markers like `{"$oid": ...}` and `{"$date": ...}` so the ObjectIds and dates aren't lost. Docker users can leave off `--out` too, and catch the output with `>` just like the archive backup:

```bash
docker exec mongodb mongoexport --db workshop --collection parts --type csv --fields name,category,quantity,price,location.shelf,location.bin > parts.csv
```

### Importing with mongoimport

It works the other way too. Say your supplier order is in a spreadsheet, saved as `new_parts.csv`:

```
name,category,quantity,price,location.shelf,location.bin
TCRT5000 Line Sensor,sensor,10,0.90,B,4
SSD1306 OLED Display,display,2,3.99,G,1
```

```bash
mongoimport --db workshop --collection parts --type csv --headerline --file new_parts.csv
```

```
2026-10-18T19:55:44.464+0000	connected to: mongodb://localhost/
2026-10-18T19:55:44.466+0000	2 document(s) imported successfully. 0 document(s) failed to import.
```

`--headerline` means "the first line holds the field names". MongoDB turns numbers into numbers, and headings like `location.shelf` into proper nested documents. Let's check in `mongosh`:

```javascript
db.parts.findOne({ name: "TCRT5000 Line Sensor" }, { _id: 0 })
```

```
{
  name: 'TCRT5000 Line Sensor',
  category: 'sensor',
  quantity: 10,
  price: 0.9,
  location: { shelf: 'B', bin: 4 }
}
```

Your Parts Bin now holds 16 parts. Imported parts have no `tags` or `used_in`, but our app copes fine with that. For Docker, use `docker exec -i mongodb mongoimport --db workshop --collection parts --type csv --headerline < new_parts.csv`.

---

## Locking the front door

Out of the box, a local MongoDB has **no password at all**. Anyone who can reach port 27017 can read, change or delete everything. On your own laptop that's fine. On the internet, it's a disaster - automated bots scan for open MongoDB servers all day, wipe them, and leave a ransom note.

So here are the rules, from most to least important.

### 1. Never expose port 27017 to the internet

Don't forward port 27017 on your router. With Docker, you can make the port reachable only from your own computer by adding `127.0.0.1:` to the `-p` option:

```bash
docker run -d --name mongodb -p 127.0.0.1:27017:27017 -v mongodb_data:/data/db mongo:8.0
```

Our original `-p 27017:27017` listens on **every** network your computer is connected to, and Docker's port publishing can slip past firewalls like `ufw`. A native install already listens only on localhost unless you change `bindIp` in its config file.

### 2. Turn on authentication

**Authentication** means the server asks "who are you?" and wants a username and password before it does anything. MongoDB keeps its users inside the database itself, so we create them first, then switch authentication on.

In `mongosh`, create an admin user in the special `admin` database. The `root` **role** - a named bundle of permissions - lets this user do anything:

```javascript
use admin
db.createUser({ user: "kev", pwd: "a-long-admin-password", roles: [ { role: "root", db: "admin" } ] })
```

```
{ ok: 1 }
```

Then create a less powerful user for the Parts Bin app. The `readWrite` role on `workshop` lets it read and change parts, but not touch any other database or create users:

```javascript
use workshop
db.createUser({ user: "parts_bin_app", pwd: "a-different-long-password", roles: [ { role: "readWrite", db: "workshop" } ] })
```

```
{ ok: 1 }
```

Pick your own long passwords, of course. Now restart MongoDB with authentication switched on. With Docker, remove the container (your data is safe in the `mongodb_data` volume, and you have a backup) and start it again with `--auth` on the end:

```bash
docker stop mongodb
docker rm mongodb
docker run -d --name mongodb -p 127.0.0.1:27017:27017 -v mongodb_data:/data/db mongo:8.0 --auth
```

Native installs add `security:` and `authorization: enabled` to the `mongod.conf` file, then restart the service.

Now connecting without a password gets you nowhere:

```
workshop> db.parts.countDocuments()
Uncaught MongoServerError[Unauthorized]: Command aggregate requires authentication
```

To log in, put the username and password in the connection string. The database at the end is where the user was created:

```bash
mongosh "mongodb://parts_bin_app:a-different-long-password@localhost:27017/workshop"
```

```
workshop> db.parts.countDocuments()
16
```

The admin user lives in `admin`, so log in with `docker exec -it mongodb mongosh -u kev -p --authenticationDatabase admin`. Leaving `-p` empty makes `mongosh` ask for the password, so it isn't saved in your terminal history.

The Database Tools take the same connection string with `--uri`, for example `mongodump --uri "mongodb://parts_bin_app:a-different-long-password@localhost:27017/workshop" --out backup`.

**Starting fresh?** On a brand new Docker setup you can skip the `createUser` step for the admin. Pass `-e MONGO_INITDB_ROOT_USERNAME=kev -e MONGO_INITDB_ROOT_PASSWORD=...` to `docker run` and the image creates a root user and turns authentication on for you. These only work when the data volume is **empty** - on our existing `mongodb_data` volume they are silently ignored.

### 3. Atlas users

Atlas always has authentication on, which is why your connection string has a username and password in it. The other lock is the **IP access list** under Network Access. Only add your own IP address. `0.0.0.0/0` means "allow from anywhere" - avoid it unless you really need it.

---

## Keeping secrets out of your code

Our `parts_bin.py` already reads `MONGODB_URI` from the environment, so the password never has to appear in the code:

```bash
export MONGODB_URI="mongodb://parts_bin_app:a-different-long-password@localhost:27017/workshop"
python parts_bin.py low
```

```
Parts with fewer than 3 in stock:

Name                           Category    Qty   Price  Bin
-----------------------------------------------------------
Raspberry Pi 5 4GB             board         1   55.00  C2
DRV8833 Motor Driver           driver        2    4.50  D3
PCA9685 Servo Driver           driver        2    5.99  D4
SSD1306 OLED Display           display       2    3.99  G1
WS2812B LED Strip 1m           lighting      2    7.50  E2

5 part(s)
```

The OLED display we imported from the CSV has already made the shopping list.

Typing `export` every time gets old. A popular alternative is a **`.env` file** - a small text file in your project folder that holds your settings:

```
MONGODB_URI=mongodb://parts_bin_app:a-different-long-password@localhost:27017/workshop
```

Install the `python-dotenv` library with `pip install python-dotenv`, then add two lines near the top of `parts_bin.py`:

```python
from dotenv import load_dotenv
```

```python
load_dotenv()
MONGODB_URI = os.environ.get("MONGODB_URI", "mongodb://localhost:27017")
```

`load_dotenv()` reads `.env` and sets its values as environment variables before we use them. Most importantly, **never commit `.env` to Git**. Add it to your `.gitignore` file:

```bash
echo ".env" >> .gitignore
```

---

## Try it Yourself

1. Make a backup file with today's date in its name, like `workshop-2026-10-18.gz`.
2. Restore your backup into a *different* database called `workshop_copy`, so you can check it without touching the real one.
3. Export only the parts with fewer than 3 in stock to `shopping_list.csv`. Hint: `mongoexport` has a `--query` option that takes a filter.

<details><summary>Show answers</summary>

1. Let the terminal fill in the date:

```bash
mongodump --db workshop --archive=workshop-$(date +%F).gz --gzip
```

2. `--nsFrom` and `--nsTo` rename the database as it is restored (`ns` is short for **namespace**, MongoDB's name for `database.collection`):

```bash
mongorestore --archive=workshop-backup.gz --gzip --nsFrom='workshop.*' --nsTo='workshop_copy.*'
```

Then `use workshop_copy` and `db.parts.countDocuments()`. Tidy up with `db.dropDatabase()`.

3. The filter is the same one we used in `mongosh`, written as JSON in single quotes:

```bash
mongoexport --db workshop --collection parts --type csv --fields name,quantity --query '{"quantity": {"$lt": 3}}' --out shopping_list.csv
```

</details>

---

## Common Issues

**Problem:** `mongodump: command not found`  
**Solution:** Docker users run it inside the container: `docker exec mongodb mongodump ...`. Everyone else installs the MongoDB Database Tools.  
**Why:** The tools are a separate download from the server and from `mongosh`.

**Problem:** `mongorestore` reports `E11000 duplicate key error` and `0 document(s) restored successfully`.  
**Solution:** Add `--drop` to replace the collection with the backup.  
**Why:** The documents are still there, and every `_id` in the backup already exists.

**Problem:** `Failed: CSV mode requires a field list`  
**Solution:** Add `--fields name,category,quantity` (or whichever fields you want).  
**Why:** A CSV file is a flat table, so MongoDB needs to know which columns to make.

**Problem:** `MongoServerError: Authentication failed.`  
**Solution:** Check the username, password and the database at the end of the connection string. The admin user needs `?authSource=admin` or `--authenticationDatabase admin`.  
**Why:** A user belongs to the database it was created in, and MongoDB looks for it there.

**Problem:** `InvalidURI: Username and password must be escaped according to RFC 3986`  
**Solution:** Encode the password first: `python -c "from urllib.parse import quote_plus; print(quote_plus('p@ss/word'))"` gives `p%40ss%2Fword`.  
**Why:** Characters like `@`, `/` and `:` have special meanings in a connection string.

**Problem:** `parts_bin.py` crashes with `OperationFailure: Command find requires authentication`.  
**Solution:** Set `MONGODB_URI` with the username and password, as above.  
**Why:** Our `ping` check passes without logging in, because the server answers pings from anyone. The first real query is where it says no.

---

## Recap

- `mongodump` and `mongorestore --drop` back up and restore a whole database, indexes included. Keep backups outside Docker.
- `mongoexport` and `mongoimport` move data to and from JSON or CSV for spreadsheets.
- Never expose port 27017 to the internet. Bind Docker to `127.0.0.1`.
- Turn on authentication, and give your app a `readWrite` user that can only touch `workshop`.
- Keep passwords in environment variables or a `.env` file, never in your code or Git.

Next, let's look back at everything we've built in the course summary.
