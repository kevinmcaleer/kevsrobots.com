---
title: Introduction to Rust
description: >-
    This lesson introduces you to the Rust programming language, guiding you through installation and your first Rust program.
layout: lesson
cover: /learn/scrawly_wally/assets/cover.png
---

![cover image]({{page.cover}}){:class="cover"}

## Introduction

`Rust` is a system programming language focused on safety, especially safe concurrency, supporting functional and imperative-procedural paradigms. This lesson is an introduction to Rust programming, especially tailored for programmers with a background in Python or MicroPython.

---

## The Origins of Rust

### Background and Creation

Rust is a programming language that focuses on speed, memory safety, and parallelism. It is designed to be a safer, concurrent, practical language, supporting pure-functional, imperative-procedural, and object-oriented styles.

The development of Rust began as a personal project by Graydon Hoare in 2006. Mozilla sponsored the project in 2009 and announced it in 2010. The language steadily grew in popularity and saw its first stable release, Rust 1.0, on May 15, 2015. Since then, it has rapidly evolved, driven by an active community and an open development process.

### Name Origin

The name "Rust" is a play on words related to the rust fungus, which is a parasite that grows on fast, distributed systems (a nod to the speed and concurrency aims of the language). Additionally, rust (the corrosion) is seen as a slower, more relentless process that breaks down old systems, aligning with the language's goal to provide a safer alternative to older system programming languages like C and C++.

### Goals of the Project

The goals of Rust are defined by its design choices, which aim to create a language that:

1. **Memory Safety**: Eliminates common bugs found in systems programming, such as null or dangling pointers and buffer overflows, without requiring a garbage collector.
2. **Concurrency Without Fear**: Provides powerful abstractions to make concurrent programming safe and efficient.
3. **Abstraction Without Overhead**: Allows developers to create abstractions without incurring performance penalties, making Rust competitive with C and C++ in terms of runtime performance.
4. **Error Messages and Developer Experience**: Focuses on providing clear and helpful error messages and a positive overall development experience.
5. **Community and Open Development**: Rust development is community-driven, aiming to be welcoming, inclusive, and open for contributions from people regardless of their background.

Rust's design is intended to support highly concurrent and highly safe systems, and "safe code" can be as fast as "unsafe code" in languages like C and C++. This approach comes from the language's unique blend of low-level control over memory with high-level language features.

### Origin story

The origins of Rust are deeply rooted in the desire to improve system programming through memory safety, concurrency, and performance. Created by Graydon Hoare and developed with the backing of Mozilla, Rust has grown into a widely used language that seeks to change how developers think about safe, efficient, and concurrent code. The Rust programming community continues to build on these goals, pushing the boundaries of what's possible in system-level programming.

---

## Learning Objectives

- Understand the benefits and fundamentals of Rust.
- Successfully install Rust on your computer.
- Write and execute your first Rust program.

---

### Why Rust?

Rust offers memory safety without garbage collection, and concurrency without data races. It's designed to help you write fast, reliable software. Here are some reasons to consider Rust:

1. **Memory Safety**: Rust's ownership system guarantees memory safety without needing a garbage collector.
1. **Concurrency**: Rust's type system and ownership model enable safe concurrency.
1. **Performance**: Rust provides control over low-level details similar to C and C++, but with higher-level ergonomics.
1. **Tooling**: The Rust ecosystem includes great tooling, including the integrated package manager and build tool, Cargo.

---

### Installing Rust

To get started with Rust:

1. Visit the [official Rust website](https://www.rust-lang.org/learn/get-started).
1. Follow the instructions to install Rust and its package manager, Cargo.
1. Once installed, open a terminal/command prompt and type `rustc --version` to verify the installation.

---

### Your First Rust Program

Let's write your first Rust program, a classic 'Hello, World!':

```rust
fn main() {
    println!("Hello, World!");
}
```

To run this program:

1. Open your text editor or IDE.
1. Copy and paste the code into a new file.
1. Save the file with a `.rs` extension, for example, `hello_world.rs`.
1. Open a terminal/command prompt.
1. Navigate to the directory containing hello_world.rs.
1. Type rustc `hello_world.rs` and press enter to compile the program.
1. Run the program by typing `./hello_world` (or `hello_world.exe` on Windows).

You should see Hello, World! printed in the terminal.

---

## Summary

In this lesson, you've been introduced to the Rust programming language. You've learned why Rust can be a powerful tool in your programming arsenal, how to install it, and how to write and execute your first Rust program.

---
