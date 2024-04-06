---
title: Setting Up Your Raspberry Pi
description: >- 
    This lesson guides you through the initial setup of your Raspberry Pi, preparing it for robotics projects with the PCA9685 and Python.
layout: lesson
type: page
cover: assets/2.png
---

![Raspberry Pi Setup]({{ page.cover }}){:class="cover"}

## Introduction to Raspberry Pi Setup

Welcome to the second lesson of our course, where we'll walk you through setting up your Raspberry Pi. This powerful, small computer is the brain of our robot arm project, and getting it ready is our first step towards bringing our robot to life.

## Equipment Needed

Before we start, ensure you have the following items:

- Raspberry Pi (any model with GPIO pins)
- MicroSD card (16GB or larger recommended)
- MicroSD card reader
- Power supply for Raspberry Pi
- Monitor, keyboard, and mouse
- An internet connection

## Installing the Operating System

1. **Download Raspberry Pi OS**: Visit the [Raspberry Pi OS download page](https://www.raspberrypi.org/software/) and download the Raspberry Pi Imager for your operating system.
2. **Prepare the MicroSD Card**: Insert your MicroSD card into the card reader and connect it to your computer. Launch the Raspberry Pi Imager, select the OS version (Raspberry Pi OS Full is recommended for beginners), and choose your MicroSD card as the target.
3. **Write the OS to the MicroSD Card**: Click the "Write" button to start the process. Once complete, eject the card safely from your computer.

## Initial Raspberry Pi Setup

1. **Connect Your Raspberry Pi**: Insert the MicroSD card into your Raspberry Pi, connect it to a monitor, keyboard, and mouse, and finally, plug in the power supply.
2. **Follow the Setup Wizard**: Upon booting, you'll be greeted by a setup wizard. Follow the instructions to set your locale, change the default password, connect to WiFi, and update the software.
3. **Enable SSH and I2C**: SSH allows remote access, and I2C is necessary for communicating with the PCA9685 board. To enable these, open a terminal and run `sudo raspi-config`. Navigate to "Interfacing Options", and enable both SSH and I2C.

## Updating and Upgrading

Ensure your Raspberry Pi is up to date with the latest packages and security updates:

```shell
sudo apt update
sudo apt full-upgrade
```

---

## Setting Up a Virtual Environment

A virtual environment is a self-contained directory that contains a Python installation for a particular version of Python, plus a number of additional packages. Using a virtual environment allows you to manage dependencies for different projects, avoiding conflicts between package versions.

To create a virtual environment, run the following commands in your terminal:

```bash
python -m venv venv
```

To activate the virtual environment, use:

- On Windows:

```bash
venv\Scripts\activate.bat
```

- On Unix or MacOS:

```bash
source venv/bin/activate
```

---

## Installing Python and Required Libraries

Python comes pre-installed on Raspberry Pi OS, but we'll need to install some additional libraries for our project:

```shell
sudo apt install python3-pip
pip3 install Adafruit-PCA9685
```

The [Adafruit](https://www.adafruit.com/) PCA9685 library is essential for controlling many servos with the PCA9685 board. It provides an easy-to-use interface for setting PWM signals to control servo motors.

---

## Conclusion

Your Raspberry Pi is now set up and ready for robotics projects! You've installed the operating system, configured the necessary settings, and installed Python along with the libraries needed for servo control. In the next lesson, we'll introduce the PCA9685 board and how to connect it to your Raspberry Pi.

## Lesson Assignment

Familiarize yourself with the Raspberry Pi terminal and try out some basic Python commands. Experiment with enabling and disabling SSH and I2C from the `raspi-config` menu to get comfortable with the interface.

---
