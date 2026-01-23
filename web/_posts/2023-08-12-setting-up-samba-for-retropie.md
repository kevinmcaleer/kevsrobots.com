---
layout: project
title: RetroPie Samba Setup 
description: "Setting up Samba for RetroPie"
difficulty: Beginner
short_title: RetroPie Samba Setup 
short_description: "Setting up Samba for RetroPie"
date: 2023-08-12
author: Kevin McAleer
excerpt: >-
    Learn how to setup Samba file sharing on a Raspberry Pi based RetroPie gaming system 
cover: /assets/img/blog/retropie/retropie.jpg
tags: 
 - raspberry_pi
 - retropie
 - retro_gaming
 - gaming
groups:
 - retro
 - raspberrypi
videos:
 - mtfUUuBg1dw
---

## Prerequisites

1. Raspberry Pi running RetroPie.
1. An active network connection on the Raspberry Pi.
1. Another computer on the same network for file transfers.

---

## Steps

1. **Access RetroPie Setup**:
   - Boot up your Raspberry Pi and enter the EmulationStation interface (default interface of RetroPie).
   - Press `Start` on your controller and select `Quit` to get to the terminal/command line.
   - Type `sudo raspi-config` and press `Enter`.

1. **Enable SSH (if not already done)**:
   - In the `raspi-config` menu, navigate to `Interfacing Options`.
   - Select `SSH` and choose `Enable`.

1. **Install Samba**:
   - At the terminal/command line, type the following commands:

     ```bash
     sudo apt-get update
     sudo apt-get install samba samba-common-bin
     ```

1. **Configure Samba for RetroPie**:
   - Edit the Samba configuration file using:

     ```bash
     sudo nano /etc/samba/smb.conf
     ```

   - Scroll to the bottom and add the following:

     ```toml
     [RetroPie]
     path = /home/pi/RetroPie/roms
     writeable=Yes
     create mask=0777
     directory mask=0777
     public=no
     ```

1. **Set a Password for Samba**:
   - While still in the terminal, set a password for the 'pi' user in Samba:

     ```bash
     sudo smbpasswd -a pi
     ```

   - Enter a password when prompted. This will be the password you'll use when accessing the file share from another computer.

1. **Restart Samba Service**:
   - Restart the Samba service to apply the changes:

     ```bash
     sudo /etc/init.d/samba restart
     ```

1. **Accessing ROMs Folder from Another Computer**:
   - **Windows**: Open File Explorer and in the address bar type `\\[RASPBERRY_PI_IP_ADDRESS]\RetroPie`. Replace `[RASPBERRY_PI_IP_ADDRESS]` with the IP address of your Raspberry Pi.
   - **Mac**: Open Finder, click on `Go` in the top menu, select `Connect to Server`, and enter `smb://[RASPBERRY_PI_IP_ADDRESS]/RetroPie`.

1. **Transfer ROMs**:
   - Once connected, you can drag and drop ROMs into the appropriate system folders.

1. **Return to EmulationStation**:
   - After transferring ROMs, type `emulationstation` in the terminal of your Raspberry Pi to return to the game interface.
   - You might need to restart EmulationStation or the Raspberry Pi to see your new games.

Remember, always ensure you're transferring ROMs you legally own. Enjoy your gaming!

---
