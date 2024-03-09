---
title: Concurrency in Rust
description: Discover Rust's approach to safe and efficient concurrency, including threads and the principles of fearless concurrency.
layout: lesson
cover: assets/8.png
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
