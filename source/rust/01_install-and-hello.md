---
title: Installing Rust and Your First Program
description: >-
    Install Rust with rustup, check it works, and build and run your first program with rustc and Cargo.
layout: lesson
type: page
cover: /learn/rust/assets/banners/01_install-and-hello.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to install Rust, and build and run a program with both `rustc` and `cargo`.

---

## Installing with rustup

Rust is installed with a small tool called **rustup**. It installs the compiler (`rustc`), the build tool and package manager (`cargo`), and lets you update them later with one command. Visit [rust-lang.org/learn/get-started](https://www.rust-lang.org/learn/get-started) and follow the steps for your system. On Linux and macOS that is one command pasted into a terminal. On Windows you download an installer, which may also ask you to install the Visual Studio C++ build tools: say yes, because Rust uses their linker.

When it finishes, open a **new** terminal and check:

```bash
rustc --version
cargo --version
```

Each prints a version number. Your numbers will differ from mine because Rust releases a new stable version every six weeks. If you see a version, you are ready.

## Your first program with rustc

Create a file called `hello.rs` and type in:

```rust
fn main() {
    println!("Hello, robot!");
}
```

Every Rust program starts at a function called `main`. `println!` prints a line. The exclamation mark tells you it is a **macro**, not an ordinary function. Compile and run it:

```bash
rustc hello.rs
./hello
```

On Windows run `hello.exe` instead. You will see:

```text
Hello, robot!
```

`rustc` turned your text into a real machine-code program. That is the big difference from Python: there is a **compile** step between writing and running.

```mermaid
flowchart LR
    A["hello.rs source code"] -->|"rustc"| B["hello program"]
    B -->|"run"| C["Hello, robot!"]
```

## The Cargo way

Real projects use Cargo, which creates the folders for you and runs the compiler. From a terminal:

```bash
cargo new hello_robot
cd hello_robot
cargo run
```

Cargo makes a folder with a `Cargo.toml` file (the project's settings) and `src/main.rs` (where your code lives). The generated program prints `Hello, world!`. `cargo run` compiles and runs it in one step, putting the results in a `target` folder. Open `src/main.rs`, change the message to `Hello, robot!` and run `cargo run` again: Cargo rebuilds only what changed.

We use Cargo for the rest of the course. Lesson 13 goes deeper into what it can do.

## Why this matters

The compile step feels slow after Python, but it is where Rust earns its reputation. Many mistakes are found before the program runs. For a robot that matters: it is far better to hear about a bug at your desk than when the robot is halfway across the table.

> **Teachers**: installing Rust needs permission to write to your home folder, and on Windows may need administrator rights for the C++ build tools. Test the install on one school machine before the lesson. The install is a sizeable download, so a whole class installing at once on school Wi-Fi can be slow. Have learners install before the lesson, or share a pre-installed machine.

## Try it Yourself

1. Change the message to print your own robot's name, then rebuild and run it.
2. Add a second `println!` line so your program prints two lines.
3. Challenge: delete the closing brace `}`, run `cargo run`, and read the error message. Then put it back.

## Common Issues

- **Problem**: `rustc: command not found` (or `'rustc' is not recognized`).
- **Solution**: Close the terminal, open a new one, and try again. If it still fails, rerun the rustup installer and check that Cargo's `bin` folder is on your `PATH`.
- **Why**: The installer changes your `PATH` setting, and terminals that were already open do not see the change.

- **Problem**: On Windows, building fails saying `link.exe` was not found.
- **Solution**: Install the Visual Studio C++ build tools and run rustup again.
- **Why**: Rust turns your code into a program using the system linker, and Windows does not include one by default.

## Check your understanding

{% quiz %}
? Which tool installs and updates the Rust compiler and Cargo?
- [ ] pip
- [x] rustup
- [ ] elf2uf2
> rustup installs `rustc` and `cargo` and can update them later.

? What does the exclamation mark in `println!` tell you?
- [ ] The line is an error
- [x] It is a macro, not an ordinary function
- [ ] The text will be printed in capitals
> Names ending in `!` are macros.

? Which command creates a new project folder with `Cargo.toml` and `src/main.rs`?
- [ ] `cargo run`
- [x] `cargo new`
- [ ] `rustc new`
> `cargo new name` generates the project. `cargo run` builds and runs it.
{% endquiz %}
