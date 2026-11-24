---
title: Setting Up Thonny
description: Get your environment ready by installing and configuring the Thonny IDE to program your Raspberry Pi Pico with MicroPython.
layout: lesson
type: page
cover: /learn/pico_temp_sensor/assets/banners/02_setting_up_thonny.jpg
---

![Course Cover Image]({{page.cover}}){:class="cover"}

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
   - **macOS:** Open the downloaded `.pkg` file and follow the installer.
   - **Linux:** Use your package manager (e.g., `sudo apt install thonny` on Debian-based systems), or the install script from thonny.org for the newest version.
   - **Raspberry Pi OS:** Thonny is already installed.

---

## Step 2: Connect the Raspberry Pi Pico

1. Use a USB cable that carries data, not a charge-only cable. A charge-only cable powers the Pico but your computer will never see it.
2. **If your Pico already has MicroPython on it**, just plug it in. It shows up as a serial port, not as a drive.
3. **If your Pico is brand new (or you are not sure)**, hold down the **BOOTSEL** button while you plug in the USB cable, then let go. The Pico appears as a removable drive called `RPI-RP2` (or `RP2350` on a Pico 2). This bootloader mode is only needed to install firmware.

---

## Step 3: Choose the MicroPython Interpreter

1. Open Thonny.
2. From the menu, go to **Tools > Options** and select the **Interpreter** tab (or click the interpreter name in the bottom-right corner of the Thonny window).
3. Choose **MicroPython (RP2040)** as the interpreter. Older versions of Thonny call it **MicroPython (Raspberry Pi Pico)**. Pick the Pico’s port in the **Port** list, or leave it on **Try to detect port automatically**.
4. If your Pico is in BOOTSEL mode, click **Install or update MicroPython** in the same dialog, choose your board, and click **Install**. The next lesson shows how to do this by hand instead.

---

## Step 4: Verify the Setup

1. In Thonny’s editor, type:

   ```python
   print("Hello, Pico!")
   ```

2. Save the script to your Pico and run it. You should see the output in Thonny’s console.

You're now ready to start programming your Pico!

---
