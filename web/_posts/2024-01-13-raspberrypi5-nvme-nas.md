---
title: How to install an NVMe drive on a Raspberry Pi 5
description: >- 
    Learn how to install an NVMe drive on a Raspberry Pi 5 and setup a NAS server.
layout: project
date: 2024-01-13
cover: /assets/img/blog/nvme/nvme.jpg
excerpt: >-
    In this article we'll take a quick look at what NVMe is all about, and how to install an NVMe drive on a Raspberry Pi 5.
author: Kevin McAleer
difficulty: beginner
groups:
    - raspberrypi
tags:
    - NAS
    - Raspberry Pi
    - Samba
    - NVMe
---

## Overview

In this article we'll take a quick look at what `NVMe` is all about, and how to install an NVMe drive on a Raspberry Pi 5. Then we'll look at how to setup a NAS server using Samba.

---

## What does NVMe stand for?

[`NVMe`](/resources/glossary#nvme) stands for Non-Volatile Memory Express.

* `NVM` refers to memory that retains its stored data even when the power is turned off. Unlike volatile memory (like RAM), which loses its data without power, non-volatile memory keeps the data intact
* `Express` - This term is borrowed from "PCI Express" (Peripheral Component Interconnect Express), a high-speed interface commonly used in computers
* `NVMe` - was specifically designed to unleash the potential of solid-state drives, which were being bottlenecked by older interfaces like SATA that were designed for slower spinning hard drives

---

## What do the NVMe sizes mean?

Regarding sizes, NVMe SSDs commonly come in a few different form factors:

* **M.2** - Most common form factor. M.2 SSDs are small and compact, making them ideal for laptops and small form factor PCs. They come in various lengths, typically labeled as 2280, 2260, 2242, etc., where the first two digits represent the width in millimeters (22mm) and the last two or three digits represent the length (e.g., 80mm, 60mm, 42mm).
* **U.2** - Formerly known as SFF-8639, U.2 NVMe SSDs are larger and often used in enterprise or server environments. They are physically similar to traditional 2.5-inch SATA drives but with a different connector to support NVMe’s higher performance.
* **PCIe Add-In Card (AIC)** - These are full-sized cards that slot directly into a PCIe slot on a motherboard. They offer higher capacities and often better performance but are larger and require a desktop PC with available PCIe slots.
* **NF1** - NF1 (Next-generation Form Factor 1) is another form factor mainly used in servers and data centers. It's designed for high density and offers a higher capacity than M.2.

Each form factor is tailored for different use cases and system configurations, from ultra-portable laptops to high-end servers and workstations.

![A slide showing the different NVMe form factors](/assets/img/blog/nvme/naming.jpg){:class="img-fluid w-100 shadow-lg rounded"}

---

## Installing an NVMe Drive

1. First of all, you'll need to get your hands on an NVMe drive. I bought a 1TB drive from Amazon for £80.
1. Next you'll need a Hat or Base board to connect the NVMe drive to the Raspberry Pi 5.
1. The boot order in the EEProm news to be changed to boot from the NVMe Drive
1. Samba needs to be installed to share the files on your network

Two great options are the:

* [Pimoroni NVMe M.2 Base](https://shop.pimoroni.com/products/nvme-base?variant=41219587178579)
* [Pineberry Pi HatDrive](https://pineberrypi.com/)

There are also two approaches to adding the NVMe drive to your Raspberry Pi 5; via a HAT or via a Base board that fits underneath the Raspberry Pi 5.

---

### How to Install a Pimoroni NVMe Base

![A Slide showing the installation steps described above](/assets/img/blog/nvme/install.jpg){:class="img-fluid w-100 shadow-lg rounded"}

The NVMe drive is installed by pushing it at a slight angle into the NVMe board and then securing it with a small screw.

![A slide showing how to insert the nvme drive](/assets/img/blog/nvme/insert.jpg){:class="img-fluid w-100 shadow-lg rounded"}

![A Slide showing the installation steps described above](/assets/img/blog/nvme/install02.jpg){:class="img-fluid w-100 shadow-lg rounded"}

---

### Screw the NVMe Base to the Raspberry Pi 5

![A Slide showing the installation steps described above](/assets/img/blog/nvme/install03.jpg){:class="img-fluid w-100 shadow-lg rounded"}

---

### Push the cable into the NVMe Base and then into the Raspberry Pi 5

![A Slide showing the installation steps described above](/assets/img/blog/nvme/cable.jpg){:class="img-fluid w-100 shadow-lg rounded"}

---

## How to install Raspberry Pi OS on an NVMe drive

This guide assumes you have a Raspberry Pi, a Pimroni NVMe Base and a NVMe drive. The Raspberry Pi 5 is already running Raspberry Pi OS via an SD Card.

Next we need to install Raspberry Pi OS on the NVMe drive. This is done by using the Raspberry Pi Imager.

1. On the Raspberry pi 5, open the Raspberry Pi Imager

    ![A Slide showing the installation steps described above](/assets/img/blog/nvme/rpiimager.png){:class="img-fluid w-100 shadow-lg rounded"}

1. Click the `Choose Device` button

    ![A Slide showing the installation steps described above](/assets/img/blog/nvme/rpiimager_home.png){:class="img-fluid w-100 shadow-lg rounded"}

1. Select the Raspberry Pi 5 option

    ![A Slide showing the installation steps described above](/assets/img/blog/nvme/choose_os.png){:class="img-fluid w-100 shadow-lg rounded"}

1. Select the Raspberry Pi OS 64bit option

    ![A Slide showing the installation steps described above](/assets/img/blog/nvme/pios64.png){:class="img-fluid w-100 shadow-lg rounded"}

1. Click the `Choose Storage` button

1. Choose the NVMe drive

1. Click the `Next` button

1. Once the Raspberry Pi Imager has finished writing the image to the NVMe drive, turn the Raspberry Pi off by pressing the on/off button

1. Finally Remove the SD Card from the Raspberry Pi 5 and reboot the Raspberry Pi 5

---

## How to boot a Raspberry Pi 5 from NVMe

Next we need to change the boot order of our Raspberry Pi 5. This is done by editing the EEProm on the Raspberry Pi 5 using the `rpi-eeprom-config` command.

```bash
sudo rpi-eeprom-config -e
```

The add the following line to the file:

```bash
BOOT_ORDER=0xf416
```

Then save the file and reboot the Raspberry Pi 5.

---

## What does NAS stand for?

[`NAS`](/resources/glossary#nas) means Network Attached Storage. This is a file-level computer data storage server connected to a computer network providing data access to a heterogeneous group of clients. NAS is specialized for serving files either by its hardware, software, or configuration.

---

## How to setup Samba on Raspberry Pi 5

1. Install Samba

    ```bash
    sudo apt install samba samba-common-bin
    ```

1. Create a directory to share

    ```bash
    mkdir /home/pi/share
    ```

1. Edit the Samba config file

    ```bash
    sudo nano /etc/samba/smb.conf
    ```

1. Add the following lines to the bottom of the file

    ```bash
    [share]
    Comment = Pi shared folder
    Path = /home/pi/share
    Browseable = yes
    Writeable = Yes
    only guest = no
    create mask = 0777
    directory mask = 0777
    Public = yes
    Guest ok = yes
    ```

1. Add a Samba user

    ```bash
    sudo smbpasswd -a pi
    ```

1. Restart Samba

    ```bash
    sudo systemctl restart smbd
    ```

1. Finally, on your computer, open a file browser and type in the IP address of your Raspberry Pi 5, followed by the name of the share folder.

---

## Conclusion

In this article we looked at what NVMe is, how to install an NVMe drive on a Raspberry Pi 5 and how to setup a NAS server using Samba.
