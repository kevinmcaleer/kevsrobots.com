---
title: Installing MicroPython on the Pico
description: Learn how to install MicroPython on your Raspberry Pi Pico to enable programming and communication through Thonny.
layout: lesson
type: page
cover: /learn/pico_temp_sensor/assets/banners/03_installing_micropython_on_pico.jpg
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

## Introduction

MicroPython is a lightweight version of Python specifically designed for microcontrollers. By installing MicroPython on your Raspberry Pi Pico, you can use Python to control hardware and create exciting projects.

---

## Step 1: Download MicroPython Firmware

1. Visit the official MicroPython download page for your board. Each board needs its own firmware:
   - Raspberry Pi Pico: [micropython.org/download/RPI_PICO](https://micropython.org/download/RPI_PICO/)
   - Raspberry Pi Pico W: [micropython.org/download/RPI_PICO_W](https://micropython.org/download/RPI_PICO_W/)
   - Raspberry Pi Pico 2: [micropython.org/download/RPI_PICO2](https://micropython.org/download/RPI_PICO2/)
2. Download the latest stable `.uf2` firmware file (not a preview build).

---

## Step 2: Enter Bootloader Mode

1. Hold down the **BOOTSEL** button on your Pico.
2. While holding the button, connect the Pico to your computer using a USB cable.
3. Let go of the button. The Pico will appear as a removable storage device called `RPI-RP2` (`RP2350` on a Pico 2).

---

## Step 3: Flash the Firmware

1. Drag and drop the downloaded `.uf2` file onto the Pico’s drive.
2. The Pico copies the firmware, reboots, and the drive disappears. That is normal: the Pico is now running MicroPython and shows up as a serial port instead of a drive.

---

## Step 4: Verify the Installation

1. Open Thonny and select **MicroPython (RP2040)** in the interpreter settings (called **MicroPython (Raspberry Pi Pico)** in older versions of Thonny).
2. You should see a `>>>` prompt in Thonny’s Shell. Type:

   ```python
   print("MicroPython is ready!")
   ```

3. Press Enter. You should see the message displayed in the Shell.

Congratulations, MicroPython is now installed on your Raspberry Pi Pico!

---
