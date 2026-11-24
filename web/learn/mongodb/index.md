---
layout: course_overview
title: MongoDB - Your First Database
author: Kevin McAleer
cover: /learn/mongodb/assets/cover.jpg
date: 2026-10-18
description: Never made a database before? Start here. Learn what a database is, install
  MongoDB, and store, find, update and organise your data - then build Parts Bin,
  a Python app that keeps track of every servo, sensor and screw in your workshop.
level: beginner
course_duration: 3h 12m
lesson_count: 18
first_lesson: 00_intro.html
quiz_total: 64
outcomes:
- Explain what a document database is, and read and write MongoDB documents
- Start a MongoDB server with Docker, Atlas or a native install and connect with mongosh
- Insert, find, update and delete documents, using query and update operators safely
- Design documents with embedding, references and schema validation
- Summarise data with the aggregation pipeline and speed up queries with indexes
- Build a Python app with PyMongo, then back up and secure the database
prerequisites:
- No database experience needed
- Helpful but not required, basic Python (variables, lists, dictionaries and functions)
  for the last part
you_need:
- A computer running Windows, macOS or Linux, or a Raspberry Pi 5 with 64-bit Raspberry
  Pi OS
- Docker, or a free MongoDB Atlas account, or a native MongoDB install
- Python 3.10 or newer for the PyMongo lessons
- A terminal
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
