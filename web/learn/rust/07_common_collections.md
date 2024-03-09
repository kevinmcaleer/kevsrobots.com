---
layout: lesson
title: Common Collections in Rust
author: Kevin McAleer
type: page
cover: assets/7.png
date: 2024-03-09
previous: 06_error_handling.html
next: 08_concurrency.html
description: Understand how to use Rust's common collections such as vectors, strings,
  and hash maps to store and manage data efficiently.
percent: 49
duration: 2
navigation:
- name: Introduction to Rust
- content:
  - section: Overview
    content:
    - name: Introduction to Rust
      link: 01_intro.html
  - section: Setting Up a Rust Environment
    content:
    - name: 'Rust vs. Python: Understanding the Differences'
      link: 02_rust_and_python.html
  - section: Basics
    content:
    - name: Rust Basics
      link: 03_rust_basics.html
    - name: Ownership and Borrowing
      link: 04_ownership_and_borrowing.html
    - name: Structs and Enums
      link: 05_structs_and_enums.html
    - name: Error Handling in Rust
      link: 06_error_handling.html
    - name: Common Collections in Rust
      link: 07_common_collections.html
  - section: Tools and Ecosystem
    content:
    - name: Concurrency in Rust
      link: 08_concurrency.html
    - name: Rust Ecosystem and Tools
      link: 09_ecosystem_and_tools.html
    - name: 'Project: Building a Simple Web Application'
      link: 10_example.html
  - section: Rust for Pico
    content:
    - name: Rust for Raspberry Pi Pico
      link: 11_rust_for_pico.html
    - name: Summary and Resources
      link: 12_summary_and_resources.html
    - name: 'Bonus Lesson: Creating and Using Cargo'
      link: 13_cargo.html
---


![cover image]({{page.cover}}){:class="cover"}

## Introduction

Rust provides several collections for storing data. Unlike arrays, these collections can grow or shrink in size. This lesson will cover three fundamental collections: vectors, strings, and hash maps, exploring how to use them to manage groups of data.

---

## Learning Objectives

- Learn how to use vectors to store lists of values in Rust.
- Understand how to manipulate strings, Rust's collection for storing text.
- Discover how to use hash maps to associate keys with values.

---

### Vectors

Vectors in Rust allow you to store more than one value in a single data structure that puts all the values next to each other in memory. Vectors are similar to arrays but can grow and shrink in size.

```rust
let mut v: Vec<i32> = Vec::new(); // create a new, empty vector
v.push(5); // add the value 5 to the end of the vector
v.push(6); // add the value 6 to the end of the vector
```

### Strings

The `String` type in Rust is a collection of characters. It's not just a simple array of chars due to Rust’s support for UTF-8 encoding, which means that it can store more than just ASCII.

```rust
let mut s = String::new(); // creates a new empty string
s.push_str("hello"); // appends "hello" to the string
```

### Hash Maps

Hash maps allow you to store keys associated with values. This structure is useful when you want to look up data based on a key rather than an index like in vectors.

```rust
use std::collections::HashMap;

let mut scores = HashMap::new();
scores.insert(String::from("Blue"), 10);
scores.insert(String::from("Yellow"), 50);
```

---

## Summary

In this lesson, you've learned about the use of common collections in Rust, including vectors, strings, and hash maps. Understanding these collections is crucial for effective data management and manipulation in Rust applications.

---
