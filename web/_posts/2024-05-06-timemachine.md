---
title: Raspberry Pi Time machine
description: >-
    How to setup a Raspberry Pi as a Time Machine backup server for your Mac.
excerpt:
    This is a simple project to setup a Raspberry Pi as a Time Machine backup server for your Mac. This will allow you to backup your Mac automatically to the Raspberry Pi over your local network.
layout: showcase
date: 2024-05-06
author: Kevin McAleer
difficulty: beginner
cover: /assets/img/blog/timemachine/cover.png
hero: /assets/img/blog/timemachine/hero.png
mode: light
tags: 
 - raspberry_pi
 - time_machine
 - backup
 - mac
groups:
 - raspberrypi
videos:
 - 4ZM1kD03ISQ
---

## What is Time Machine?

Time Machine is a backup software application distributed with the Apple macOS computer operating system. The software is designed to work with external storage devices such as USB drives, SSDs, or network-attached storage (NAS) devices. Time Machine creates incremental backups of files that can be restored at a later date. The software is designed to be user-friendly and requires minimal configuration to set up.

---

## Why use a Raspberry Pi as a Time Machine server?

Using a Raspberry Pi as a Time Machine server has several advantages:

1. **Cost-effective**: Raspberry Pi devices are inexpensive and can be repurposed for various projects.
1. **Energy-efficient**: Raspberry Pi devices consume very little power compared to traditional computers.
1. **Always-on**: Raspberry Pi devices can be left running 24/7, allowing for continuous backups.
1. **Network-attached storage**: Raspberry Pi devices can be connected to your local network, making them accessible to multiple devices.

---

## Prerequisites

Before you begin, you will need the following:

1. A Raspberry Pi with Raspbian installed
1. An external USB drive, or for Raspberry Pi 5, NVMe SSD (with an NVMe HAT)
1. A Mac computer running macOS
1. A local network connection

---

## Setting up the Raspberry Pi

### Step 1: Install Raspbian

If you haven't already, install Raspbian on your Raspberry Pi. You can download the latest version of Raspbian from the [Raspberry Pi website](https://www.raspberrypi.org/software/).

### Step 2: Update the system

Once Raspbian is installed, update the system by running the following commands:

```bash
sudo apt update
sudo apt upgrade
```

### Step 3: Install Samba

Samba is a software suite that provides file and print services for Windows, macOS, and Linux systems. Install Samba on your Raspberry Pi by running the following command:

```bash
sudo apt install samba samba-common
```

---

## Configuring Samba

### Step 1: Create a shared folder

Create a folder on your Raspberry Pi to store the Time Machine backups. You can create the folder using the following command:

```bash
sudo mkdir /mnt/timemachine
sudo chmod 777 -R /mnt/timemachine
```

### Step 2: Configure Samba

Edit the Samba configuration file to allow access to the shared folder. Open the configuration file in a text editor by running the following command:

```bash
sudo nano /etc/samba/smb.conf
```

Add the following lines to the end of the file:

```bash
[timemachine]
   path = /mnt/timemachine
   browseable = yes
   read only = no
   guest ok = no
   create mask = 0600
   directory mask = 0700
   comment = Raspberry Pi Time Capsule
   writeable = yes
   valid users = kev # change this to your user account
   write list = kev  # change this to your user account
   vfs objects = catia fruit streams_xattr
   fruit:aapl = yes
   fruit:time machine = yes
```

Save the file and exit the text editor.

**Note** - use `CTLR + x`, then `s` to save your file.

---

## Create an SMB account on the Pi

Create an SMB account on the Raspberry Pi by running the following command:

```bash
sudo smbpasswd -a kev # change this to your user account
```

---

## Setup Avahi (Bonjour) on the Raspberry Pi

### Step 1: Create a new service file

Create a new service file for Avahi by running the following command:

```bash
sudo nano /etc/avahi/services/samba.service
```

Add the following lines to the file:

```xml
# /etc/avahi/services/samba.service

<?xml version="1.0" standalone='no'?><!--*-nxml-*-->
<!DOCTYPE service-group SYSTEM "avahi-service.dtd">
<service-group>
  <name replace-wildcards="yes">%h</name>
  <service>
    <type>_smb._tcp</type>
    <port>445</port>
  </service>
  <service>
    <type>_device-info._tcp</type>
    <port>9</port>
    <txt-record>model=TimeCapsule8,119</txt-record>
  </service>
  <service>
    <type>_adisk._tcp</type>
    <port>9</port>
    <txt-record>dk0=adVN=backups,adVF=0x82</txt-record>
    <txt-record>sys=adVF=0x100</txt-record>
  </service>
</service-group>
```

**Note** - use `CTLR + x`, then `s` to save your file.

---

### Step 2: Restart Samba

Restart the Samba service to apply the changes by running the following command:

```bash
sudo systemctl restart smbd
```

---

## Setting up Time Machine on your Mac

### Step 1: Open the System Preferences

![Time Machine System Preferences](/assets/img/blog/timemachine/timemachine.png){:class="img-fluid w-100 rounded-3 card-shadow"}

Open the System Preferences on your Mac and search for "Time Machine."

---

### Step 2: Add a new backup disk

Click the Plus (+) button to add a new backup disk.

The Raspberry Pi should appear in the list of available disks. Select the Raspberry Pi and click "Use Disk."

---

### Step 3: Enter your SMB credentials

Enter the SMB credentials you created earlier on the Raspberry Pi.

---

### Step 4: Start the backup

Click "Back Up Now" to start the backup process. (You can find this in the System menu at the top right of your screen under the Time Machine icon.)

The first back up will take several hours, depending on the size of your Mac's hard drive. Subsequent backups will be much faster as Time Machine only backs up changes.

---
