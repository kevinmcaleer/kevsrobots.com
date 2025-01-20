---
layout: lesson
title: 'Bonus Lesson: Creating and Using Cargo'
author: Kevin McAleer
type: page
cover: /learn/scrawly_wally/assets/13.png
date: 2024-03-09
previous: 12_summary_and_resources.html
description: Learn how to create, manage, and build Rust projects using Cargo, Rust's
  package manager and build system.
percent: 100
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

Cargo is not just a package manager; it's also a Rust project’s build system. It manages building your code, downloading the libraries your code depends on, and building those libraries. This bonus lesson will guide you through creating a new Rust project with Cargo, managing dependencies, and building projects.

---

## Learning Objectives

- Understand how to create new Rust projects using Cargo.
- Learn how to manage project dependencies.
- Explore building and running Rust projects with Cargo.

---

### Creating a New Rust Project with Cargo

To start a new Rust project with Cargo, you can use the following command:

```bash
cargo new project_name
```

This command creates a new directory called `project_name` containing a `Cargo.toml` file (the manifest file for Rust projects) and a `src` directory with a `main.rs` file. The `Cargo.toml` file will include basic metadata about your project and a list of dependencies.

### Managing Dependencies

Add dependencies to your project by listing them under `[dependencies]` in your `Cargo.toml` file. For example, to add the `serde` library:

```toml
[dependencies]
serde = "1.0"
```

When you build your project, Cargo will automatically download and compile your dependencies and all of their dependencies.

### Building and Running Projects

To build your project, run the following command in your project directory:

```bash
cargo build
```

This command compiles your project and all of its dependencies. If the build is successful, Cargo places the executable in `target/debug/project_name`.

To build and run your project in one step, you can use:

```bash
cargo run
```

If your project compiles successfully, Cargo will then run the resulting executable.

---

## Summary

In this bonus lesson, you learned how to create a new Rust project using Cargo, manage your project's dependencies, and build and run your project. Cargo is a powerful tool that simplifies many aspects of Rust development, making it easier to manage large projects and their dependencies.

---
