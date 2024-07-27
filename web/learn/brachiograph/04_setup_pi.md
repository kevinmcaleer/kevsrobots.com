---
layout: lesson
title: Setup Raspberry Pi
author: Kevin McAleer
type: page
cover: assets/cover.png
date: 2024-07-26
previous: 03_wire.html
next: 05_code.html
description: null
percent: 48
duration: 3
navigation:
- name: BrachioGraph Tutorial
- content:
  - section: Introduction
    content:
    - name: BrachioGraph Pen Plotter
      link: 01_intro.html
  - section: Build the plotter
    content:
    - name: Building the Plotter
      link: 02_build.html
    - name: Wiring up the BrachioGraph
      link: 03_wire.html
  - section: Programming
    content:
    - name: Setup Raspberry Pi
      link: 04_setup_pi.html
    - name: Code
      link: 05_code.html
    - name: Draw
      link: 06_draw.html
    - name: Vectorize Images
      link: 07_vectorize.html
  - section: Summary
    content:
    - name: Summary and Key Takeaways
      link: 08_summary.html
---


In this lesson you will learn how to setup a Raspberry Pi to run the BrachioGraph Python code. The Raspberry Pi is a small single-board computer that can be used to control the BrachioGraph plotter. By installing the Raspberry Pi OS and the required Python libraries, you can run the BrachioGraph code to create drawings and designs.

## Step 1: Install Raspberry Pi OS

1. Download the Raspberry Pi Imager from the [Raspberry Pi website](https://www.raspberrypi.org/software/).

1. Install the Raspberry Pi Imager on your computer.

1. Insert the microSD card into your computer.

1. Open the Raspberry Pi Imager and select the Raspberry Pi OS.

1. Select the microSD card as the storage location.

1. Click on "Write" to install the Raspberry Pi OS on the microSD card.

1. Once the installation is complete, remove the microSD card from your computer.

---

## Step 2: Boot Raspberry Pi

1. Insert the microSD card into the Raspberry Pi.

1. Connect the Raspberry Pi to a monitor, keyboard, and mouse.

1. Connect the power supply to the Raspberry Pi.

1. The Raspberry Pi will boot up and display the Raspberry Pi OS desktop.

---

## Step 3: Connect to Wi-Fi

1. Click on the Wi-Fi icon in the top right corner of the screen.

1. Select your Wi-Fi network from the list of available networks.

1. Enter the Wi-Fi password and click on "Connect".

1. The Raspberry Pi will connect to the Wi-Fi network.

---

## Step 4: Update Raspberry Pi

1. Open the Terminal from the Raspberry Pi desktop.

1. Run the following commands to update the Raspberry Pi:

    ```bash
    sudo apt update
    sudo apt upgrade
    ```

1. Wait for the updates to complete.

---

## Step 5: Install Python

1. Run the following command to install Python:

    ```bash
    sudo apt install python3 python3-pip
    ```

---

## Step 6: Install BrachioGraph

1. Clone the code repository from GitHub:

    ```bash
    git clone https://www.github.com/evildmp/brachiograph.git
    ```

1. Create a virtual environemnt:

    ```bash
    python3 -m venv venv
    ```

1. Activate the virtual environment:

    ```bash
    source venv/bin/activate
    ```

1. Run the following command to install the required Python libraries:

    ```bash
    sudo pigpiod
    cd BrachioGraph
    pip install -r requirements.txt
    ```

> ## What is Pigpiod?
>
> **Note:** The `pigpiod` service is required to control the servos using the GPIO pins on the Raspberry Pi.

---

## Step 7: Run BrachioGraph

1. Run the following command to start the BrachioGraph code:

    ```bash
    python3
    ```

    Python with then load and in the next lesson you will learn how to use the BrachioGraph to create drawings and designs.

---
