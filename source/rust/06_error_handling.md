---
title: Error Handling in Rust
description: Explore Rust's approach to error handling through the Option and Result types, and learn about error propagation.
layout: lesson
cover: /learn/scrawly_wally/assets/6.png
---

![cover image]({{page.cover}}){:class="cover"}

## Introduction

Error handling is a critical part of programming. Rust handles errors through its type system, specifically with the `Option` and `Result` types, rather than exceptions. This lesson explains how these types work and how they are used in Rust for error handling and propagation.

---

## Learning Objectives

- Understand the `Option` and `Result` types and their purposes in Rust.
- Learn how to use these types for handling potential errors.
- Explore error propagation in Rust to create robust and safe applications.

---

### Option and Result

The `Option` type is used in Rust when a value could be something or nothing (`None`). This type is used as a safer alternative to null references in other languages.

```rust
let some_number = Some(5);
let no_number: Option<i32> = None;
```

The `Result` type is used when operations could fail. It returns `Ok(value)` if the operation is successful, or `Err(e)` if it fails. This approach makes you explicitly handle errors, leading to more robust code.

```rust
fn divide(numerator: f64, denominator: f64) -> Result<f64, &'static str> {
    if denominator == 0.0 {
        Err("Cannot divide by zero")
    } else {
        Ok(numerator / denominator)
    }
}
```

### Error Propagation

Instead of handling errors immediately, sometimes it's more appropriate to propagate them to the calling code. Rust makes error propagation concise and safe with the `?` operator.

```rust
fn get_divided_result(numerator: f64, denominator: f64) -> Result<f64, &'static str> {
    let result = divide(numerator, denominator)?;
    Ok(result)
}
```

The `?` operator automatically propagates the error if the operation fails, simplifying error handling especially when multiple operations could fail.

---

## Summary

In this lesson, you've learned about Rust's approach to error handling using the `Option` and `Result` types and explored how these can be used to handle and propagate errors in a safe, explicit manner. Understanding and utilizing these concepts are fundamental in writing robust Rust applications.

---
