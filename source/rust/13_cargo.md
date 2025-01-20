---
title: "Bonus Lesson: Creating and Using Cargo"
description: >- 
    Learn how to create, manage, and build Rust projects using Cargo, Rust's package manager and build system.
layout: lesson
cover: /learn/scrawly_wally/assets/13.png
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
