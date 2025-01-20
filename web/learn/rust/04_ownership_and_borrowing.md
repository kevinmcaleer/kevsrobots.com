---
layout: lesson
title: Ownership and Borrowing
author: Kevin McAleer
type: page
cover: /learn/scrawly_wally/assets/4.png
date: 2024-03-09
previous: 03_rust_basics.html
next: 05_structs_and_enums.html
description: 'Dive into Rust''s unique features: ownership, references, and borrowing,
  and understand their impact on memory safety and concurrency.'
percent: 28
duration: 3
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

One of Rust's distinguishing features is its ownership system, which is a set of rules that the compiler checks at compile time. It does not slow down your program while running. This lesson will cover the concepts of ownership, references, and borrowing, as well as lifetimes, which are all central to understanding how Rust manages memory.

---

## Learning Objectives

- Comprehend Rust's ownership principles and how they contribute to memory safety.
- Understand references and the borrowing rules in Rust.
- Learn about lifetimes and their role in managing references.

---

### Understanding Ownership

Ownership in Rust means that each value in Rust has a variable that's called its owner. There can only be one owner at a time, and when the owner goes out of scope, the value will be dropped.

The rules of ownership are:

1. Each value in Rust has a variable that’s called its ‘owner’.
2. There can only be one owner at a time.
3. When the owner goes out of scope, the value will be dropped.

This system prevents memory leaks and allows for more efficient memory management.

### References and Borrowing

Instead of transferring ownership, Rust uses references to give you access to values without taking ownership of them. This process is called 'borrowing'. As with ownership, there are rules for borrowing:

1. You may have either one mutable reference or any number of immutable references to a value at a time.
2. References must always be valid.

Borrowing allows you to use values without taking ownership, enabling multiple parts of your code to access data without duplicating it.

### Lifetimes

Lifetimes are Rust's way of ensuring that all borrows are valid. They are a part of the type system and describe the scope for which a reference is valid. In many cases, lifetimes are implicit and inferred, just like most types are, but there are situations where you need to annotate them explicitly.

Lifetimes ensure that references do not outlive the data they refer to, preventing dangling references.

---

> ### Python Comparisons
>
> - In Python, memory management is handled by garbage collection, so the concepts of ownership, borrowing, and lifetimes are not directly applicable.
> - Python allows you to reference objects multiple times without explicit rules, relying on reference counting and garbage collection to manage memory, which contrasts with Rust's compile-time checks for memory safety and access rules.

---

## Summary

In this lesson, you learned about Rust's ownership, borrowing, and lifetimes. Understanding these concepts is fundamental to writing safe and efficient Rust code, as they enable Rust to manage memory safety without a garbage collector.

---
