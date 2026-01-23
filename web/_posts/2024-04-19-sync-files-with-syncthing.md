---
title: Sync Files on your Pis, with Syncthing
description: >-
    Learn how to share files between your Raspberry Pis using Syncthing, a secure and private file synchronization tool.
layout: showcase
date: 2024-04-19
author: Kevin McAleer
difficulty: beginner
excerpt: >-
    
cover: /assets/img/blog/syncthing/cover.png
hero: /assets/img/blog/syncthing/hero.png
mode: light
tags:
  - raspberry_pi
  - file_syncing
  - syncthing
groups:
  - raspberrypi
videos:
  - 0J-YYtkWP3o

code:
 - https://gist.github.com/kevinmcaleer/a5571ae18d814ae26ba23c3801670b05

---

If you've used Raspberry Pi's for a while, particularly if you're looking at using one a as a daily driver, you may find yourself needing to share files between them (and other devices).

Most of the popular file syncing apps such as i/cloud, Dropbox, Google Drive, and OneDrive are great, but they're not always the best solution for syncing files between your Raspberry Pi's (they aren't free either, unlike syncthing which is free and open source).

Enter [Syncthing](https://syncthing.net/), a secure and private file synchronization tool that you can use to sync files between your Raspberry Pi's, and other devices.

---

## What is Syncthing?

Syncthing is a continuous file synchronization program. It synchronizes files between two or more computers in real time, safely protected from prying eyes. Your data is your data alone and you deserve to choose where it is stored, whether it is shared with some third party, and how it's transmitted over the internet.

Files are stored only on the machines that you choose - they are not stored on a central server, so you can be sure that your files are safe and secure.

---

## Installing Syncthing

![Raspberry Pi Desktop](/assets/img/blog/syncthing/install01.png){:class="w-100 img-fluid rounded-3 shadow-lg"}

We need to make sure a couple of preequisites are installed on your Raspberry Pi before we can install Syncthing.

1. Launch a terminal window on your Raspberry Pi (you can do this by clicking on the terminal icon in the Raspberry Pi menu).

      ![Terminal Icon](/assets/img/blog/syncthing/install02.png){:class="w-100 img-fluid shadow-lg rounded-3"}

      ---

1. Update our package list:

      ![Update Package List](/assets/img/blog/syncthing/install03.png){:class="w-100 img-fluid rounded-3 shadow-lg"}

      From the termial window, type the following command:

      ```bash
      sudo apt update
      ```

      ---

1. Install the `apt-transport-https` package:
      From the terminal window, type the following command:

      ```bash
      sudo apt install apt-transport-https
      ```
  
      ![Install apt-transport-https](/assets/img/blog/syncthing/install04.png){:class="w-100 img-fluid rounded-3 shadow-lg"}

      ---

1. Add the Syncthing repository key:
      From the terminal window, type the following command:

      ```bash
      curl -s https://syncthing.net/release-key.txt | gpg --dearmor | sudo tee /usr/share/keyrings/syncthing-archive-keyring.gpg >/dev/null
      ```

      ![Add Syncthing Repository Key](/assets/img/blog/syncthing/install05.png){:class="w-100 img-fluid rounded-3 shadow-lg"}
  
      ---

1. Add the Syncthing repository:
      From the terminal window, type the following command:

      ```bash
      echo "deb [signed-by=/usr/share/keyrings/syncthing-archive-keyring.gpg] https://apt.syncthing.net/ syncthing stable" | sudo tee /etc/apt/sources.list.d/syncthing.list
      ```

      ![Add Syncthing Repository](/assets/img/blog/syncthing/install06.png){:class="w-100 img-fluid rounded-3 shadow-lg"}

      ---

1. Update our package list again:
      From the terminal window, type the following command:

      ```bash
      sudo apt update
      ```

      ![Update Package List](/assets/img/blog/syncthing/install07.png){:class="w-100 img-fluid rounded-3 shadow-lg"}

      ---

1. Install Syncthing:
      From the terminal window, type the following command:

      ```bash
      sudo apt install syncthing
      ```

      ![Install Syncthing](/assets/img/blog/syncthing/install08.png){:class="w-100 img-fluid rounded-3 shadow-lg"}

      ---

1. Start Syncthing

      You can start Syncthing either by clicking on the Raspberry Pi menu > Internet > Start Syncthing, or by typing the following command in the terminal window:

      ```bash
      syncthing
      ```

      ![Start Syncthing](/assets/img/blog/syncthing/install09.png){:class="w-100 img-fluid rounded-3 shadow-lg"}

      ---

1. Launch the Web Interface

      Once Syncthing has started, you can open a web browser and navigate to `http://localhost:8384` to access the Syncthing web interface. There is also a link in the Raspberry Pi Menu > Internet > Syncthing Web UI.

      You can click `yes` on the dialog box to share anonymous usage data with the Syncthing project.

      ![Syncthing Web Interface](/assets/img/blog/syncthing/install10.png){:class="w-100 img-fluid rounded-3 shadow-lg"}

      Its recommended to set a username and password for the web interface, you can do this by clicking on the `Actions` button in the top right corner of the web interface, and then clicking on `Settings`.

      ![Syncthing Icon](/assets/img/blog/syncthing/install11.png){:class="w-100 img-fluid rounded-3 shadow-lg"}

---

## Web UI Screen

The name of this computer is shown in the menu bar, at the top left of the screen.

---

### Folders  List

There is a folders list, showing the folders that are either being shared or ready to be shared.

---

### This Device

This section shows the device ID of the Raspberry Pi (or computer, if no a Pi) you are currently using, along with stats about the number and total size of the files being shared

---

### Remote Devices

This section shows the device ID of the other Computers that you are sharing files with, along with stats about the number and total size of the files being shared.

![Syncthing Web Interface](/assets/img/blog/syncthing/install12.png){:class="w-100 img-fluid rounded-3 shadow-lg"}

---

## Adding Folders

1. To add a folder to Syncthing, you'll need to click on the `Add Folder` button.

1. Type the path to the folder you want to share with other users in the `Folder Path`, then click on the `Save` button.

> ### Note
>
> If the folder does not exist, Syncthing will create it for you.

![Add Folder](/assets/img/blog/syncthing/sharing01.png){:class="w-100 img-fluid rounded-3 shadow-lg"}

---

## Adding Devices

Adding a remote device and a folder to Syncthing will allow you to share files between your Raspberry Pi's, and other devices.

Log into the other computer you want to share files with, and install Syncthing on that computer.

Once Syncthing is installed, you can open a web browser and navigate to `http://localhost:8384` to access the Syncthing web interface.

You need the long Device ID of the computer you want to share files with. You can find the Device ID by clicking on the `Actions` button in the top right corner of the web interface, and then clicking on `Show ID`.

Copy this Device ID, and log back into the Raspberry Pi you want to share files with.

---

To add a device to Syncthing, you'll need to click on the `Add Remote Device` button.

Add the copied Device ID of the remote computer you want to share files with in the `Device ID` field.

Once you've entered the Device ID, you'll need to click on the `Save` button.

---

Switch back to the computer you want to share files with, and you should see a dialog box asking if you want to add the Raspberry Pi as a remote device, Click `Yes`.

The files should now be shared between the two computers.

---

## Gotchas and things to note

1. Syncthing is a great tool for syncing files between your Raspberry Pi's, and other devices. It's secure, private, and free, and it's easy to install and configure.

1. Be sure not to fill up the SD card on your Raspberry Pi with files, as this can cause the Raspberry Pi to slow down, and eventually fail. Syncthing will stop syncronizing files if the SD card is full, or reaches the value set in the Settings menu

1. Its best to use the Synchronised files folder as a transport area, rather than a working folder. If you are working on a file or project with in the Synchronised files folder, it can cause issues as files may be created and removed before Syncthing can detect them and the project may get out of sync.

---

## Conclusion

Syncthing is a great tool for syncing files between your Raspberry Pi's, and other devices. It's secure, private, and free, and it's easy to install and configure.

If you're looking for a way to sync files between your Raspberry Pi's, give Syncthing a try!

---
