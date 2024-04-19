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
  - raspberry pi
  - file syncing
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

To install Syncthing on your Raspberry Pi, you can use the following commands (which available as a downlodable script here: <https://gist.github.com/kevinmcaleer/a5571ae18d814ae26ba23c3801670b05>)

```bash
#!/bin/bash

echo Installing Prerequisites

sudo apt update
sudo apt install apt-transport-https

curl -s https://syncthing.net/release-key.txt | gpg --dearmor | sudo tee /usr/share/keyrings/syncthing-archive-keyring.gpg >/dev/null

echo "deb [signed-by=/usr/share/keyrings/syncthing-archive-keyring.gpg] https://apt.syncthing.net/ syncthing stable" | sudo tee /etc/apt/sources.list.d/syncthing.list

sudo apt update

echo Installing SyncThing
sudo apt install syncthing

echo copying syncthing service
sudo cp syncthing.service /lib/systemd/system/syncthing.service

echo enabling service
sudo systemctl enable syncthing

echo starting service
sudo systemctl start syncthing

echo Done.
```

Once you have Syncthings installed, you'll want to configure it to sync files between your Raspberry Pi's.

---

## Configuring Syncthing

To configure Syncthing, you'll need to open a web browser and navigate to `http://localhost:8384` on the Raspberry Pi that you've installed Syncthing on.

Note, you can also start the service by clicking on the Syncthing icon in the Raspberry Pi menu.

Once you've opened the web interface, you'll be presented with the Syncthing dashboard.

---

## Adding Devices

To add a device to Syncthing, you'll need to click on the `Add Remote Device` button.

You'll need to enter the Device ID of the device you want to sync with. You can find the Device ID by clicking on the `Actions` button on the device you want to sync with, and then clicking on `Show ID`.

Once you've entered the Device ID, you'll need to click on the `Save` button.

---

## Adding Folders

To add a folder to Syncthing, you'll need to click on the `Add Folder` button.

You'll need to enter the Folder ID of the folder you want to sync. You can find the Folder ID by clicking on the `Actions` button on the folder you want to sync, and then clicking on `Show ID`.

Once you've entered the Folder ID, you'll need to click on the `Save` button.

---

## Syncing Files

Once you've added devices and folders to Syncthing, you'll need to click on the `Start` button to start syncing files.

Syncthing will then start syncing files between the devices and folders you've added.

---

## Conclusion

Syncthing is a great tool for syncing files between your Raspberry Pi's, and other devices. It's secure, private, and free, and it's easy to install and configure.

If you're looking for a way to sync files between your Raspberry Pi's, give Syncthing a try!

---
