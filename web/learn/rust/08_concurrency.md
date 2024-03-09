---
layout: lesson
title: Concurrency in Rust
author: Kevin McAleer
type: page
cover: assets/8.png
date: 2024-03-09
previous: 07_common_collections.html
next: 09_ecosystem_and_tools.html
description: Discover Rust's approach to safe and efficient concurrency, including
  threads and the principles of fearless concurrency.
percent: 56
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

Concurrency in Rust is designed to be safe and efficient, preventing common pitfalls found in other languages. This lesson introduces Rust's concurrency model, focusing on threads and the concept of fearless concurrency.

---

## Learning Objectives

- Understand how to create and manage threads in Rust.
- Learn about Rust’s ownership and type systems' role in enabling fearless concurrency.

---

### Threads

Threads in Rust allow you to perform different tasks concurrently. Creating new threads is done using the `thread::spawn` function, which takes a closure containing the code to be executed in the new thread.

```rust
use std::thread;
use std::time::Duration;

thread::spawn(|| {
    for i in 1..10 {
        println!("hi number {} from the spawned thread!", i);
        thread::sleep(Duration::from_millis(1));
    }
});
```

### Fearless Concurrency

Rust’s memory safety guarantees extend to concurrency, making it easier to write safe and concurrent code. This is known as fearless concurrency. Rust achieves this through:

- **Ownership and Types**: The ownership system (along with types and borrowing) ensures data races are caught at compile time.
- **Send and Sync traits**: These marker traits define whether a type is safe to send or share across threads, respectively.

---

## Summary

This lesson covered the basics of implementing concurrency in Rust. By leveraging Rust’s strong type system and ownership rules, you can write concurrent code that is both safe and efficient, avoiding the pitfalls typically associated with threading and data races.

---
