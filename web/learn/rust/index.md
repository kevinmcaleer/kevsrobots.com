---
layout: course_overview
title: Introduction to Rust
author: Kevin McAleer
cover: /learn/rust/assets/cover.png
date: 2024-03-09
description: Learn the Rust programming language from scratch, with a Python and MicroPython
  maker in mind. Write and run real programs on your computer, build a small web server,
  then set up Rust to blink an LED on the Raspberry Pi Pico.
level: intermediate
course_duration: 1h 7m
lesson_count: 14
first_lesson: 00_intro.html
quiz_total: 42
video: M9LekkoQyf0
outcomes:
- Install Rust and build, run and test programs with Cargo
- Explain ownership and borrowing, and read the compiler errors they produce
- Model robot data with structs and enums, and handle failures with Option and Result
- Use vectors, strings, hash maps and threads safely
- Build a small JSON web server with warp
- Set up the Rust toolchain for the Raspberry Pi Pico and flash an LED blinker
prerequisites:
- Some programming experience, ideally in Python or MicroPython, such as the Learn
  MicroPython course
- Comfort using a terminal or command prompt
you_need:
- A computer running Windows, macOS or Linux, with an internet connection
- A text editor, such as Visual Studio Code with the rust-analyzer extension
- For the final lesson only, an original Raspberry Pi Pico (RP2040, not Pico W or
  Pico 2) and a micro-USB data cable
navigation:
- name: Introduction to Rust
- content:
  - section: Getting Started
    content:
    - name: Introduction to Rust
      link: 00_intro.html
    - name: Installing Rust and Your First Program
      link: 01_install-and-hello.html
    - name: 'Rust vs. Python: Understanding the Differences'
      link: 02_rust_and_python.html
  - section: The Language
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
