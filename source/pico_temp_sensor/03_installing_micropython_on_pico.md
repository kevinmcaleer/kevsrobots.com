---
title: Installing MicroPython on the Pico
description: Learn how to install MicroPython on your Raspberry Pi Pico to enable programming and communication through Thonny.
layout: lesson
type: page
cover: assets/micropython_install_cover.png
---

## Introduction

MicroPython is a lightweight version of Python specifically designed for microcontrollers. By installing MicroPython on your Raspberry Pi Pico, you can use Python to control hardware and create exciting projects.

---

## Step 1: Download MicroPython Firmware

1. Visit the [official MicroPython download page](https://micropython.org/download/rp2-pico/).
2. Download the latest `.uf2` firmware file for the Raspberry Pi Pico.

---

## Step 2: Enter Bootloader Mode

1. Hold down the **BOOTSEL** button on your Pico.
2. While holding the button, connect the Pico to your computer using a USB cable.
3. The Pico will appear as a removable storage device.

---

## Step 3: Flash the Firmware

1. Drag and drop the downloaded `.uf2` file onto the Pico’s drive.
2. The Pico will reboot and appear as a MicroPython device.

---

## Step 4: Verify the Installation

1. Open Thonny and select **MicroPython (Raspberry Pi Pico)** in the interpreter settings.
2. In the console, type:

   ```python
   print("MicroPython is ready!")
   ```

3. You should see the message displayed in the console.

Congratulations, MicroPython is now installed on your Raspberry Pi Pico!

---
