---
layout: lesson
title: Structs and Enums
author: Kevin McAleer
type: page
cover: assets/5.png
date: 2024-03-09
previous: 04_ownership_and_borrowing.html
next: 06_error_handling.html
description: Learn how to organize data using Rust's structs and enums, and understand
  how pattern matching works with these types.
percent: 35
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

Structs and enums are the building blocks for creating new types in Rust’s type system. While structs allow you to create custom types that group related data together, enums enable you to define a type by enumerating its possible variants. This lesson explores how to define and use structs and enums, and introduces pattern matching.

---

## Learning Objectives

- Understand how to define and use structs in Rust.
- Learn about enums and their role in Rust.
- Discover how pattern matching works in conjunction with enums.

---

### Defining Structs

In Rust, a struct is a way to group related data together into a meaningful composite type. Each piece of data in the struct is called a 'field'. Structs are similar to tuples, but each field has a name, providing more clarity.

```rust
struct User {
    username: String,
    email: String,
    sign_in_count: u64,
    active: bool,
}
```

### Enums and Pattern Matching

Enums in Rust are types that can encompass different 'variants'. Enums are particularly useful for defining a type which could be one of a few different variants. Combined with Rust's pattern matching, enums are powerful for handling different scenarios.

```rust
enum Message {
    Quit,
    Move { x: i32, y: i32 },
    Write(String),
    ChangeColor(i32, i32, i32),
}

let msg = Message::Write(String::from("hello"));
```

Pattern matching in Rust allows you to compare a value against a series of patterns and execute code based on which pattern matches. It can be used with enums to handle different scenarios:

```rust
match msg {
    Message::Quit => {
        println!("The Quit variant has no data to destructure.");
    },
    Message::Move { x, y } => {
        println!("Move in the x direction {} and in the y direction {}", x, y);
    },
    Message::Write(text) => {
        println!("Text message: {}", text);
    },
    Message::ChangeColor(r, g, b) => {
        println!("Change the color to red {}, green {}, and blue {}", r, g, b);
    },
}
```

---

## Summary

In this lesson, you learned about struct and enum types in Rust, how to define them, and how to use them in conjunction with pattern matching to handle different kinds of data and control flow in a type-safe way.

---
