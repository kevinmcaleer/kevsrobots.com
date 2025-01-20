---
layout: lesson
title: Rust for Raspberry Pi Pico
author: Kevin McAleer
type: page
cover: /learn/scrawly_wally/assets/11.png
date: 2024-03-09
previous: 10_example.html
next: 12_summary_and_resources.html
description: Learn how to set up Rust for development on the Raspberry Pi Pico, including
  environment setup, installation, debugging, and a sample program.
percent: 77
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

The Raspberry Pi Pico is a popular microcontroller for various projects. This lesson will guide you through setting up Rust for development on the Pico, how to install your Rust programs on the device, techniques for debugging, and walking through a sample program.

---

## Learning Objectives

- Set up the Rust build environment for the Raspberry Pi Pico.
- Understand how to install Rust programs onto the Raspberry Pi Pico.
- Learn debugging techniques for Rust programs on the Pico.
- Implement and understand an example Rust program on the Pico.

---

### Setting up the Build Environment

To develop Rust programs for the Raspberry Pi Pico, you'll need to set up your build environment:

1. Install the standard Rust toolchain via [rustup](https://rustup.rs/).
2. Install the `cross` tool for cross-compiling: `cargo install cross`.
3. Configure the Rust toolchain for cross-compiling to the ARM Cortex-M0+ processor used by the Pico.

```bash
rustup target add thumbv6m-none-eabi
```

4. Set up your development environment to access the USB port of the Raspberry Pi Pico (this process varies between operating systems).

### How to Install on Pico

After developing your Rust application, you'll need to compile it for the Pico and upload it:

1. Build your project for the ARM Cortex-M0+ architecture:

```bash
cargo build --release --target thumbv6m-none-eabi
```

2. Convert the compiled program to UF2 format, which is necessary for the Raspberry Pi Pico bootloader.

3. Push the RESET button on your Pico while holding the BOOTSEL button, release the BOOTSEL after the device is connected. It should mount as a Mass Storage Device.

4. Copy your `.uf2` file to the Raspberry Pi Pico mass storage device.

### Debugging

Debugging microcontroller applications can be challenging. Here are some techniques for debugging Rust programs on the Pico:

- Use `println!` debugging to send output to a connected serial terminal.
- Use an in-circuit debugger (ICD) like the Raspberry Pi Pico's built-in SWD port for real-time debugging.
- Employ unit tests for parts of your code that can be tested on the host machine.

### Example Program

Here's a simple Rust program that blinks an LED on the Raspberry Pi Pico:

```rust
#![no_std]
#![no_main]

use panic_halt as _;
use rp_pico::hal::{prelude::*, Timer};
use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    let mut pac = rp_pico::pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = rp_pico::hal::clocks::init_clocks_and_plls(
        rp2040::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = Timer::new(pac.TIMER, &mut pac.RESETS);

    let sio = Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();
    
    loop {
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}
```

This program initializes the board's peripherals, sets up an LED pin, and then blinks the LED on and off every 500 milliseconds.

---

## Summary

In this lesson, you've learned how to set up a Rust development environment for the Raspberry Pi Pico, how to install Rust programs on the device, strategies for debugging, and you've walked through an example program that blinks an LED.

---
