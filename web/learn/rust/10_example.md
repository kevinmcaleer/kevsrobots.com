---
layout: lesson
title: 'Project: Building a Simple Web Application'
author: Kevin McAleer
type: page
cover: /learn/scrawly_wally/assets/10.png
date: 2024-03-09
previous: 09_ecosystem_and_tools.html
next: 11_rust_for_pico.html
description: Apply what you've learned in Rust by building a simple web application.
  This project will cover setup, logic implementation, and finalization.
percent: 70
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

Now that you're familiar with Rust's fundamentals, its concurrency model, error handling, and ecosystem, it's time to put your knowledge into practice by building a simple web application. This project will guide you through setting up your project environment, implementing the application logic, and finalizing your project.

---

## Learning Objectives

- Set up a Rust project for a web application.
- Implement basic web server logic using Rust.
- Finalize and run your web application.

---

### Setting up

First, create a new Rust project:

```bash
cargo new rust_web_app
cd rust_web_app
```

Then, add necessary dependencies in your `Cargo.toml`:

```toml
[dependencies]
warp = "0.3"
tokio = { version = "1", features = ["full"] }
serde = { version = "1.0", features = ["derive"] }
```

We'll use `warp` for the web server framework, `tokio` for asynchronous runtime, and `serde` for serializing and deserializing the JSON data.

### Implementing Logic

Create a new file in the `src` directory called `server.rs`. Here, you will define your routes and request handlers. Start with a simple health check endpoint:

```rust
use warp::Filter;

async fn health_check() -> Result<impl warp::Reply, warp::Rejection> {
    Ok("OK")
}

pub fn routes() -> impl Filter<Extract = impl warp::Reply, Error = warp::Rejection> + Clone {
    warp::path!("health").and(warp::get()).and_then(health_check)
}
```

In your `main.rs`, set up the server to run:

```rust
use warp::Filter;

mod server;

#[tokio::main]
async fn main() {
    let routes = server::routes();

    warp::serve(routes).run(([127, 0, 0, 1], 3030)).await;
}
```

### Finalizing the Project

Before running your application, review the code to ensure it follows best practices and that you've handled potential errors. Run your project:

```bash
cargo run
```

Test your application by visiting `http://127.0.0.1:3030/health` in a web browser or using a tool like `curl`:

```bash
curl http://127.0.0.1:3030/health
```

---

## Summary

In this project, you've built a simple web application using Rust. You set up the project environment, implemented a basic web server, and learned how to run and test your Rust web application. This project serves as a foundation for building more complex web applications with Rust.

---
