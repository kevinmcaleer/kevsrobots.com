---
title: Setting Up the Raspberry Pi
description: Instructions to set up and configure the Raspberry Pi.
layout: lesson
type: page
cover: assets/setup_pi.png
date_updated: 2024-08-02
---

![Setup Pi](assets/pi_setup01.png){:class="cover"}

---

## Setting Up the Raspberry Pi

Setup the Pi for the Scrawly Wally robot:

Item | Action
--- | ---
**Install Raspberry Pi OS** Download and install the latest version of Raspberry Pi OS on a microSD card. Insert the microSD card into the Raspberry Pi and boot it up. <br /><br />**Connect to WiFi**: Follow the on-screen instructions to connect your Raspberry Pi to your WiFi network. <br /><br />**Update the System**: Open a terminal and run: `sudo apt-get update && sudo apt-get upgrade`<br /><br />**Enable I2C, SPI, and Serial Interfaces**: Run `sudo raspi-config` and enable the necessary interfaces. | ![Setup Pi](assets/pi_setup01.png){:class="w-100 img-fluid rounded-3 card-shadow card-hover"}
{:class="table table-striped"}

---

Your Raspberry Pi is now set up. Next, we'll install the software needed for the robot.

---
