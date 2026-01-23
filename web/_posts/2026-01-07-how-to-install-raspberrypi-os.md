---
title: "How to install RaspberryPi OS"
layout: showcase
cover: /assets/img/blog/install_pios/cover.jpg
hero: /assets/img/blog/install_pios/hero.png
mode: light
short_title: How to install RaspberryPi OS
short_description: A step-by-step guide to installing Raspberry Pi OS.
description: A detailed tutorial on how to install Raspberry Pi OS.
date: 2026-01-07
author: Kevin McAleer
excerpt: >
    A step-by-step guide to installing Raspberry Pi OS.

tags:
 - raspberry_pi

groups:
 - raspberrypi
---

The Raspberry Pi Imager tool got a maker over in 2025, and is now more *wizard*-like than ever before. This guide will walk you through the steps to install Raspberry Pi OS on your Raspberry Pi.

## Step 1: Install Raspberry Pi OS on an SD Card

1. Download the Raspberry Pi Imager from the [Raspberry Pi website](https://www.raspberrypi.org/software/).

    {% include gallery.html images="/assets/img/blog/install_pios/piimager.jpg" cols="2" titles="Raspberry Pi Imager" links="https://www.raspberrypi.org/software/" use-links=true %}

---

1. Install the Raspberry Pi Imager on your computer

1. Insert the microSD card into your computer

1. Open the Raspberry Pi Imager and select the Raspberry Pi OS.

1. Choose the Raspberry Pi device from the list

1. Choose the OS you want to install (Raspberry Pi OS Desktop is recommended for most users)

1. Select the storage location from the available list of drives

1. Make customisations, such as enabling SSH, setting Wi-Fi credentials, and locale settings
 
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
