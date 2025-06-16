---
title: Setting Up the Toolchain
description: Learn how to install and configure the tools you need to write, compile, and upload C code to the Raspberry Pi Pico.
layout: lesson
type: page
cover: assets/6.jpg
date_updated: 2025-06-15
---

![Cover]({{page.cover}}){:class="cover"}

---

To program your Raspberry Pi Pico in C, you’ll need a **toolchain** — the set of software tools that compile your code and upload it to the board.

This lesson walks you through installing the official **Pico SDK** and CMake-based toolchain.

---

## What You’ll Need

- A Raspberry Pi Pico (or Pico W)
- A computer running **Linux**, **macOS**, or **Windows (WSL recommended)**
- A micro-USB cable
- Some patience for first-time setup!

---

## Step 1: Install Prerequisites

### On Linux (Debian/Ubuntu)

```bash
sudo apt update
sudo apt install cmake gcc-arm-none-eabi build-essential libnewlib-arm-none-eabi git
````

### On macOS

Install Homebrew, then:

```bash
brew tap ArmMbed/homebrew-formulae
brew install cmake gcc-arm-embedded
```

### On Windows

Use **WSL** with Ubuntu for best results. Inside WSL, use the Linux steps above.

---

## Step 2: Download the Pico SDK and Examples

```bash
mkdir -p ~/pico
cd ~/pico
git clone -b master https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init
```

Now clone the example projects:

```bash
cd ~/pico
git clone -b master https://github.com/raspberrypi/pico-examples.git
```

---

## Step 3: Set Environment Variables

Add the SDK path to your shell config:

```bash
echo "export PICO_SDK_PATH=~/pico/pico-sdk" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 4: Build a Sample Project

```bash
cd ~/pico/pico-examples/blink
mkdir build
cd build
cmake ..
make
```

You should now have a file called `blink.uf2` — this is the binary you’ll upload to the Pico.

---

## Step 5: Upload to the Pico

1. Hold down the **BOOTSEL** button on the Pico
2. Plug it into USB
3. A new USB drive called **RPI-RP2** will appear
4. Drag and drop the `blink.uf2` file onto the drive

Your Pico will reboot and start blinking the onboard LED!

---

## Summary

You now have:

* A working C toolchain with the Raspberry Pi Pico SDK
* The ability to compile and build `.uf2` firmware files
* A method for flashing code onto your Pico

---

Next up: [Your First Program in C](07_first_program), where you'll write a custom C program from scratch and upload it to the Pico.

---
