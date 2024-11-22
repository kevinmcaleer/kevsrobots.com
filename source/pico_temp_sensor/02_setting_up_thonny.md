---
title: Setting Up Thonny
description: Get your environment ready by installing and configuring the Thonny IDE to program your Raspberry Pi Pico with MicroPython.
layout: lesson
type: page
---

## Introduction

Before you can start programming your Raspberry Pi Pico, you need an Integrated Development Environment (IDE) that supports MicroPython. Thonny is a beginner-friendly IDE that makes it easy to write and execute Python scripts on your Pico.

---

## What You Will Do in This Lesson

- Install Thonny IDE on your computer.
- Connect your Raspberry Pi Pico.
- Configure Thonny to program with MicroPython.

---

## Step 1: Install Thonny

1. Visit [thonny.org](https://thonny.org) to download the latest version of Thonny for your operating system.
2. Follow the installation instructions for your platform:
   - **Windows:** Run the installer and follow the prompts.
   - **MacOS:** Drag the Thonny app to your Applications folder.
   - **Linux:** Use your package manager (e.g., `sudo apt install thonny` on Debian-based systems).

---

## Step 2: Connect the Raspberry Pi Pico

1. Plug your Raspberry Pi Pico into your computer using a USB cable.
2. Hold down the **BOOTSEL** button while connecting the USB cable to enter the Pico’s bootloader mode.
3. The Pico should appear as a removable drive on your computer.

---

## Step 3: Install MicroPython on the Pico

1. Open Thonny.
2. From the menu, go to **Tools > Options** and select the **Interpreter** tab.
3. Choose **MicroPython (Raspberry Pi Pico)** as the interpreter.
4. If prompted, install the MicroPython firmware on your Pico.

---

## Step 4: Verify the Setup

1. In Thonny’s editor, type:

   ```python
   print("Hello, Pico!")
   ```

2. Save the script to your Pico and run it. You should see the output in Thonny’s console.

You're now ready to start programming your Pico!

---
