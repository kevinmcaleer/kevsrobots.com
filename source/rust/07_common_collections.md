---
title: Common Collections in Rust
description: Understand how to use Rust's common collections such as vectors, strings, and hash maps to store and manage data efficiently.
layout: lesson
cover: /learn/scrawly_wally/assets/7.png
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
