---
title: Installing the Software
description: Guide to installing the necessary software for the wall drawing robot.
layout: lesson
type: page
cover: assets/software.png
date_updated: 2024-08-02
---

## Installing the Software

The software for this project is avialable on GitHub - <https://www.github.com/kevinmcaleer/scrawly-wally.git>.

To install this on your Raspberry Pi, follow these steps:

1. **Install Python and Required Libraries**:
   - Open a terminal and run:

     ```sh
     sudo apt-get install python3 python3-pip
     pip3 install RPi.GPIO pigpio
     ```

     Note - `pigpio` is a library for controlling the GPIO pins on the Raspberry Pi; it needs to be run as a daemon process. You can start it by running:

     ```sh
      sudo pigpiod
      ```

1. **Clone the Software**:
   - To clone the software, Run:

     ```sh
     git clone https://www.github.com/kevinmcaleer/scrawly-wally.git
     ```

---

The software is now installed. Let's move on to writing the control code.

> **Note**: The software for this project is a work in progress and may be updated. Check the GitHub repository for the latest version and updates.
>
> To update the software, navigate to the cloned repository and run `git pull` to fetch the latest changes.

---
