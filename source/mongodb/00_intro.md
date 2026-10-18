---
title: Introduction
description: Learn MongoDB from scratch - no database experience needed - and build a Parts Bin app to track every part in your workshop
layout: lesson
type: page
cover: /learn/mongodb/assets/banners/00_intro.jpg
date_updated: 2026-10-18
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Ahoy there makers! How many SG90 servos have you got right now? Not roughly. Exactly. And which shelf are they on?

If you are anything like me, the honest answer is "some" and "somewhere near the soldering iron". I have bought the same motor driver three times because I could not find the first two. That is the problem we are going to fix in this course - and along the way you will learn one of the most popular databases in the world.

Never created a database before? Perfect. This course assumes you have not. We start with what a database actually *is*, and we take it one small step at a time from there.

---

## Overview

[MongoDB](https://www.mongodb.com) is a **document database**. Instead of squeezing your information into rows and columns like a spreadsheet, it stores each thing as a little bundle of labelled information that looks a lot like this:

```javascript
{
  name: "SG90 Micro Servo",
  category: "servo",
  quantity: 12,
  price: 2.50,
  location: { shelf: "A", bin: 1 },
  tags: ["servo", "5v", "smars"]
}
```

If you can read that, you can already read MongoDB data. Each one of those bundles is called a **document**, and that is the whole idea of the course in one picture.

We will use that idea to build **Parts Bin** - a database of every part in a maker's workshop: servos, sensors, boards, motor drivers, batteries and screws. Every lesson adds to it. By the end you will turn it into a Python command line app that can answer questions like:

```bash
python parts_bin.py find servo      # what servos have I got, and where are they?
python parts_bin.py use "SG90 Micro Servo" --count 2   # I just used two
python parts_bin.py low             # what do I need to reorder?
python parts_bin.py value           # how much is all this stuff worth?
```

---

## Course Content

- What a database is, and why it beats a spreadsheet or a text file
- JSON, documents, collections and databases, explained in plain English
- Installing MongoDB with Docker, on MongoDB Atlas (free, nothing to install), or natively
- Your first steps in `mongosh`, the MongoDB shell
- Inserting documents, and what that mysterious `_id` field is for
- Finding documents, and narrowing your search with query operators
- Updating and deleting documents - safely
- Working with nested documents and lists
- Designing your data: when to keep things together and when to split them up
- Aggregation - turning raw data into reports and totals
- Indexes, and how they make queries fast
- Using MongoDB from Python with `pymongo`
- Building the finished Parts Bin app
- Backing up your data and keeping your database secure

---

## Key Results

After completing this course, you will:

- Understand what a database is and when you need one
- Be able to install and connect to MongoDB on your own computer or in the cloud
- Create, read, update and delete data - the four jobs every database does
- Write queries that find exactly the documents you are after
- Know how to design documents for your own projects
- Be able to produce summaries and totals with the aggregation pipeline
- Have a working Python app, backed by MongoDB, that tracks your workshop parts
- Know how to back up your data and lock your database down

---

## What you'll need

- **A computer** running Windows, macOS or Linux. A **Raspberry Pi 5** with 64-bit Raspberry Pi OS works too (older Pis need the cloud option - lesson 3 explains why)
- **Docker** (recommended), or a free **MongoDB Atlas** account if you would rather not install anything. We cover both, plus a normal install, in lesson 3
- **Python 3.10 or newer** for the last part of the course
- **A terminal** - the Command Prompt, PowerShell, Terminal on a Mac, or a Linux shell. You do not need to be an expert, I will tell you exactly what to type

No electronics needed. Everything here is software, although your own parts drawer is welcome to join in.

---

## Prerequisites

You do not need any database experience at all. That is the point.

You will get more out of the Python lessons near the end if you know a little Python - variables, lists, dictionaries and functions. If that is new to you, these courses are a great place to start:

- [Learn Python](/learn/python/) - the fundamentals
- [Docker](/learn/docker/) - handy if you want to understand what the install commands are doing

If you enjoy this course, the [SQLite3](/learn/sqlite3/) and [DuckDB](/learn/duckdb/) courses show you the other big family of databases, and the [FastAPI](/learn/fastapi/) course shows you how to put a database behind a web app.

---

## How the course works

Each lesson introduces one new idea and then lets you try it straight away on the Parts Bin data. Commands you type into the MongoDB shell look like this:

```javascript
db.parts.find({ category: "servo" })
```

Commands you type into your computer's terminal look like this:

```bash
docker exec -it mongodb mongosh
```

And what the computer prints back is shown in a plain block, so you can check you got the same thing:

```
{ ok: 1 }
```

Every example in the course has been run against a real MongoDB 8.0 server, so if your output looks different, something is worth a second look - and the **Common Issues** section at the end of most lessons will probably tell you what.

Most lessons also finish with a **Try it Yourself** section: a few small challenges, with the answers tucked away so you can check your work once you have had a go.

Right then. Let's find out what a database actually is.
