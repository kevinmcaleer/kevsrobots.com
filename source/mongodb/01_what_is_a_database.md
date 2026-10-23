---
title: What is a Database?
description: What a database actually is, why it beats a spreadsheet for some jobs, and where MongoDB fits in
layout: lesson
type: page
cover: /learn/mongodb/assets/banners/01_what_is_a_database.jpg
date_updated: 2026-10-18
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Picture the parts drawer in your workshop. Somewhere in there are three SG90 servos, a motor driver you bought "just in case", and a bag of M3 screws that is definitely nearly empty. You know roughly what is in there, but answering a simple question like "how many servos do I have?" means tipping the whole thing out on the bench.

A database is the cure for that. Let's look at what one actually is before we start typing commands.

---

## So what is a database?

A **database** is an organised collection of information, plus the software that looks after it. That second part matters. The software:

- **Stores** your data safely on disk, so it survives a reboot or a power cut
- **Finds** things quickly, even when there are millions of them
- **Changes** things without mangling them, even when several programs write at the same time
- **Answers questions** you send it, like "which parts are running low?"

A question you send to a database is called a **query**. You will be writing a lot of those in this course.

---

## Why not just use a spreadsheet?

Fair question. Plenty of makers track their parts in a spreadsheet or a text file, and for a dozen items that works fine. Things start to creak as soon as you want more:

| | Text file | Spreadsheet | Database |
|---|---|---|---|
| Easy to start | Yes | Yes | A little setup |
| Fast with thousands of items | No | Slows down | Yes |
| Several programs using it at once | Risky | Risky | Built for it |
| Your own code can ask it questions | You write the searching | Awkward | Yes, that is the point |
| Each item can have different details | Messy | Every row has the same columns | Yes, with MongoDB |
{:class="table table-single"}

The big one for makers is the fourth row. A robot logging sensor readings, a web page showing your stock, and a Python script reordering parts can all talk to the same database at the same time without tripping over each other.

---

## Servers and clients

Databases like MongoDB work as a **server** and a **client**.

- The **server** is a program that runs quietly in the background, holding all the data. It sits and listens for requests on a network **port** (a numbered door on your computer). MongoDB's port is **27017**.
- A **client** is any program that connects to the server and sends it requests. In this course our clients will be `mongosh` (MongoDB's command line shell) and, later, a Python script.

Think of a café. The kitchen (server) has all the food. Customers (clients) walk up to the counter and order. Lots of customers can order at once, and none of them need to know how the kitchen is laid out.

The server and client can be on the same computer, which is how we will start, or on opposite sides of the world.

---

## What is MongoDB?

**MongoDB** is a popular, free-to-use database first released in 2009. The name comes from "humongous", because it was designed to handle huge amounts of data. We will use the free **Community Edition** (and optionally the free tier of MongoDB's cloud service, Atlas).

You will often hear MongoDB described with two phrases.

### NoSQL

**SQL** (often said "sequel") is the language used by **relational databases**, which store data in tables of rows and columns, a bit like a strict spreadsheet. **NoSQL** means "not only SQL" and is a catch-all name for databases that store data some other way. If you are curious about the table-based world, our [SQLite course](/learn/sqlite3/) is a gentle introduction, but you don't need it for this course.

### Document database

MongoDB is a **document database**. Instead of spreading a part across rows and columns, it stores each part as one self-contained **document**, a bit like an index card with everything about that part written on it:

```javascript
{
  name: "SG90 Micro Servo",
  category: "servo",
  quantity: 12,
  tags: ["servo", "5v", "smars"]
}
```

Everything about the servo, including its list of tags, lives together on one card. We will pick this apart properly in the next lesson.

---

## Where is MongoDB used?

All over the place, because documents map so neatly onto the objects programs already work with:

- **Web and mobile apps** - user profiles, shopping baskets, product catalogues
- **Internet of Things** - streams of sensor readings from devices
- **Games** - player saves, inventories and leaderboards
- **Content sites** - articles, comments and settings

For us, it is a great fit for a workshop full of parts that are all a little bit different. A servo has a torque rating, a battery has a capacity, and a screw has neither.

---

## What we'll build

Throughout this course we will build the **Parts Bin**, a database that tracks every part in a maker's workshop: servos, sensors, boards, motor drivers, batteries and screws.

- The database will be called `workshop`
- The parts will live in a collection called `parts`
- In the final project lesson it becomes a Python command line app, `parts_bin.py`, that can add parts, find them, mark them as used, restock them and warn you when you are running low

Every example in the course uses this same data, so by the end you will have a genuinely useful tool for your own bench.

---

## Try it Yourself

No commands yet, just a bit of thinking.

1. Name three things in your workshop, home or job that would make a good database. What questions would you want to ask each one?
2. In the café analogy, what plays the part of the menu, the order and the meal?
3. Write down, in plain English, every detail you would want to record about one part in your own parts drawer.

<details><summary>Show answers</summary>

1. Anything with lots of similar items works: your parts, your 3D printer filament, your books, sensor readings from a weather station. Questions might be "what is running low?", "what did I buy in March?" or "what was the hottest day?"
2. The menu is what the server can do (its commands), the order is your **query**, and the meal is the **result** the server sends back to the client.
3. There is no wrong answer. Typical ones are name, category, how many you have, price, where it lives (shelf and bin), and which projects use it. Keep your list - it is very close to what we will store.

</details>

---

## Common Issues

**Problem:** "Do I need to learn SQL first?"  
**Solution:** No. MongoDB does not use SQL at all. Everything in this course is self-contained.  
**Why:** MongoDB has its own way of writing queries, using the same document shape as the data itself.

**Problem:** "Is the server a separate computer I need to buy?"  
**Solution:** No. The server is just a program. We will run it on the computer you already have.  
**Why:** "Server" describes a role (the thing that answers requests), not a type of hardware.

---

## Recap

- A database stores information safely and answers questions (queries) about it quickly
- A database server holds the data and a client sends it requests - MongoDB listens on port 27017
- MongoDB is a NoSQL document database: each item is stored as one self-contained document
- Our project is the Parts Bin: a `workshop` database with a `parts` collection

Next up, we will look closely at documents and collections and learn to read the format MongoDB uses.
