---
layout: lesson
title: Rust Basics
author: Kevin McAleer
type: page
cover: /learn/scrawly_wally/assets/3.png
date: 2024-03-09
previous: 02_rust_and_python.html
next: 04_ownership_and_borrowing.html
description: Learn the fundamentals of Rust programming including variables, data
  types, and control flow.
percent: 21
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

Building on the differences between Rust and Python, this lesson covers the basic building blocks of Rust programming. You'll learn about variables, data types, and control flow in Rust.

---

## Learning Objectives

- Understand Rust's variable declaration and data types.
- Grasp Rust's control flow mechanisms.
- Compare these basic concepts with Python to aid understanding for those with Python experience.

---

### Variables and Mutability

In Rust, variables are immutable by default, meaning once a value is bound to a name, you cannot change that value. To make a variable mutable, you must use the `mut` keyword. This approach contrasts with Python, where variables are mutable by default.

```rust
let x = 5; // immutable
let mut y = 5; // mutable
y += 1; // this is allowed only because y is mutable
```

### Data Types

Rust is a statically typed language, requiring explicit type definitions at compile time. However, the compiler can usually infer what type each variable is, so you don't always have to write type annotations. This is different from Python, which is dynamically typed.

```rust
let x: i32 = 5; // x is an integer
let y: bool = true; // y is a boolean
```

### Control Flow

Rust's control flow constructs are similar to those in other C-like languages and include `if`, `else`, `while`, and `for`. Unlike Python, Rust uses braces `{}` to define scope instead of indentation.

```rust
let number = 6;

if number % 2 == 0 {
    println!("number is even");
} else {
    println!("number is odd");
}
```

---

> ### Python Comparisons
>
> - **Variables**: In Python, variables can be reassigned to different types without declaring them as mutable or immutable.
>
> - **Data Types**: Python does not require explicit type annotations due to its dynamic typing.
>
> - **Control Flow**: Python uses indentation, not braces, to define scopes and does not require parentheses for conditions in control flow statements.

---

## Summary

This lesson covered the basics of Rust, including variable mutability, data types, and control flow structures. Understanding these fundamental differences between Rust and Python is crucial for transitioning from Python to Rust programming efficiently.

---
