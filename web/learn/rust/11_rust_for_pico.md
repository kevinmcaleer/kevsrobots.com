---
layout: lesson
title: Rust for Raspberry Pi Pico
author: Kevin McAleer
type: page
cover: /learn/rust/assets/banners/11_rust_for_pico.jpg
date: 2024-03-09
previous: 10_example.html
next: 12_summary_and_resources.html
description: Set up the Rust toolchain for the Raspberry Pi Pico, understand a no_std
  program, and build and flash an LED blinker.
percent: 84
duration: 7
quiz_total: 42
date_updated: 2026-11-24
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


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to set up a Rust project for the original Raspberry Pi Pico, explain what `#![no_std]` means, and flash a blinking LED program.

---

## Which board?

This lesson targets the **original Raspberry Pi Pico**, which has the RP2040 chip, a Cortex-M0+ processor, and an LED wired straight to a pin. Two close relatives need different set-up:

- The **Pico W** has its LED on the wireless chip, not on a normal pin, so the blink below will not light it.
- The **Pico 2** uses a different chip, the RP2350, with its own crate (`rp235x-hal`) and set-up. Check that crate's documentation.

The crates here were checked in November 2026: `rp-pico` 0.9.0 (which builds on `rp2040-hal` 0.10), `cortex-m` 0.7.7, `cortex-m-rt` 0.7.7, `embedded-hal` 1.0.0 and `panic-halt` 1.0.0.

## How a Rust program reaches the Pico

```mermaid
flowchart LR
    A["main.rs"] -->|"cargo build, target thumbv6m-none-eabi"| B["ELF file"]
    B -->|"elf2uf2-rs"| C["UF2 file"]
    C -->|"copied to the RPI-RP2 drive"| D["Pico runs it"]
```

Your computer is not a Cortex-M0+, so Cargo **cross-compiles**: it builds for another processor. You tell it the target, `thumbv6m-none-eabi`. The Pico's bootloader accepts UF2 files, so a tool converts and copies the result.

## Step 1: install the tools

```bash
rustup target add thumbv6m-none-eabi
cargo install elf2uf2-rs --locked
```

The first adds the standard-library pieces for the Pico's processor. The second installs `elf2uf2-rs`, which converts the program to UF2 and copies it to a Pico in bootloader mode. You do not need the `cross` tool.

## Step 2: create the project

```bash
cargo new pico_blinky
cd pico_blinky
```

Edit `Cargo.toml` so it reads:

```toml
[package]
name = "pico_blinky"
version = "0.1.0"
edition = "2021"

[dependencies]
cortex-m = "0.7.7"
cortex-m-rt = "0.7.7"
embedded-hal = "1.0.0"
panic-halt = "1.0.0"
rp-pico = "0.9.0"
```

Create `.cargo/config.toml` (a `.cargo` folder inside the project) with the build settings used by the rp-hal project:

```toml
[build]
target = "thumbv6m-none-eabi"

[target.thumbv6m-none-eabi]
rustflags = [
    "-C", "link-arg=--nmagic",
    "-C", "link-arg=-Tlink.x",
    "-C", "no-vectorize-loops",
]
runner = "elf2uf2-rs -d"
```

Create `memory.x` in the project root. It tells the linker how the Pico's memory is laid out: 256 bytes of second-stage bootloader, then flash up to 2 MB, then 256 KB of RAM.

```text
MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
    RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}

EXTERN(BOOT2_FIRMWARE)

SECTIONS {
    /* ### Boot loader */
    .boot2 ENTRY(BOOT2_FIRMWARE) :
    {
        KEEP(*(.boot2));
    } > BOOT2
} INSERT BEFORE .text;
```

## Step 3: the blink program

Replace `src/main.rs` with this program, adapted from the `pico_blinky` example in the `rp-pico` crate:

```rust
#![no_std]
#![no_main]

use rp_pico::entry;
use embedded_hal::digital::OutputPin;
use panic_halt as _;
use rp_pico::hal::prelude::*;
use rp_pico::hal::pac;
use rp_pico::hal;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let sio = hal::Sio::new(pac.SIO);
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

## Reading the program

- `#![no_std]` means no standard library. The Pico has no operating system, so there are no files, threads or `println!`. You get the `core` library instead, which includes `Option`, `Result`, iterators and the rest of the language you learned.
- `#![no_main]` and `#[entry]` replace the usual `main`. The start-up code from `cortex-m-rt` calls your function, which returns `!` (never) because firmware must not end.
- `panic_halt` decides what a panic does: stop forever. This is the same panic as `unwrap()` in lesson 6.
- `Peripherals::take().unwrap()` hands you the chip's hardware exactly once. That is ownership applied to hardware: no two parts of the program can own the same peripheral.
- The clock set-up, `Pins::new` and `pins.led` are the board crate telling you which pin the LED is on.

## Step 4: flash it

Hold the **BOOTSEL** button, plug the Pico in with a USB cable, then release the button. A drive called `RPI-RP2` appears. Then run:

```bash
cargo run --release
```

The runner converts the program and copies it to the drive, and the LED starts blinking once a second. If the drive is not detected, copy the UF2 file across by hand.

## Debugging without println

With no standard library there is no `println!`. You have three options: blink the LED in patterns; use a USB serial example (the `rp-pico` crate has `pico_usb_serial`); or use a debug probe, a second Pico running probe firmware, or Raspberry Pi's Debug Probe, wired to the SWD pins, with `probe-rs`. The Pico has no built-in debugger, so a probe is separate hardware. The rp-hal project's own config shows the matching runner for it: `probe-rs run --chip RP2040`, used in place of the `elf2uf2-rs` runner.

> **Teachers**: you need one original Pico per learner or pair, plus data-capable USB cables (some cables charge only). School computers may block the `RPI-RP2` drive or stop you running `cargo install`, so test one machine first. Avoid Pico W boards for this lesson, because their LED is not on a normal pin. If hardware is short, learners can read the program and the diagram and still complete the quiz.

## Try it Yourself

1. Change both delays to 100 and flash again. How does the blink change?
2. Make a double blink: on, off, on, off, then a longer pause.
3. Remove `panic_halt` and read the compiler error about a missing panic handler.
4. Challenge: read the `pico_pwm_blink` example in the `rp-pico` crate and make the LED fade.

## Common Issues

- **Problem**: `can't find crate for core`.
- **Solution**: Run `rustup target add thumbv6m-none-eabi`.
- **Why**: The core library for the Pico's processor has to be installed before you can build for it.

- **Problem**: The linker cannot find `memory.x` or `link.x`.
- **Solution**: Put `memory.x` in the project root and check the `-Tlink.x` flag is in `.cargo/config.toml`.
- **Why**: The linker script comes from `cortex-m-rt` and reads your `memory.x` to know where to put the code.

- **Problem**: `elf2uf2-rs` cannot find the Pico.
- **Solution**: Unplug, hold BOOTSEL while plugging in, and wait for the `RPI-RP2` drive.
- **Why**: The Pico only accepts new programs while in bootloader mode.

- **Problem**: The program flashes but nothing blinks on a Pico W.
- **Solution**: Use an original Pico, or wire an external LED and resistor to a normal pin.
- **Why**: On the Pico W the on-board LED is controlled through the wireless chip.

## Check your understanding

{% quiz %}
? What does `#![no_std]` mean?
- [ ] The program has no variables
- [x] The standard library is not used, because there is no operating system
- [ ] The program cannot panic
> Firmware uses the `core` library instead of `std`, so there is no `println!`.

? Which target do you build for on the original Pico?
- [ ] `x86_64-unknown-linux-gnu`
- [x] `thumbv6m-none-eabi`
- [ ] `wasm32-unknown-unknown`
> The RP2040's Cortex-M0+ processor needs the `thumbv6m-none-eabi` target.

? How does a UF2 file get onto the Pico?
- [ ] Over Wi-Fi
- [x] Copied to the RPI-RP2 drive while the Pico is in BOOTSEL mode
- [ ] Typed into the REPL
> `elf2uf2-rs -d` converts the build and copies it to the mounted drive.
{% endquiz %}
