---
title: Setting up the Pi
description: Flash Raspberry Pi OS Lite, turn on I2C, add the FTDI latency rule and install the Open Duck Mini runtime on the v2 branch
layout: lesson
type: page
cover: /learn/open_duck_mini/assets/banners/08_setting_up_the_pi.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

The duck is built. Now let's give it a brain to run. In this lesson we'll set up the Raspberry Pi Zero 2W from a blank SD card to a working copy of the Open Duck Mini runtime. If you've done the [Linux command line course](/learn/linux_intro/), this will feel familiar.

The steps follow the [Open_Duck_Mini_Runtime README on the v2 branch](https://github.com/apirrone/Open_Duck_Mini_Runtime). If anything here differs from the README, follow the README.

---

## Step 1: flash the SD card

Use **Raspberry Pi Imager** on your computer:

1. Choose your Pi model.
2. Choose **Raspberry Pi OS Lite (64-bit)**. Lite has no desktop, which leaves more of the Pi Zero 2W's memory for the duck.
3. Before writing, open the settings and preset:
   - a hostname and a username and password
   - your **Wi-Fi** network
   - **SSH**, switched on
4. Write the card, put it in the Pi and power it up.

After a minute or two, connect from your computer:

```bash
ssh <user>@<robot ip>
```

---

## Step 2: update and install the basics

```bash
sudo apt update && sudo apt upgrade
sudo apt install git python3-pip
```

The README also suggests `python3-virtualenvwrapper`, which gives you the handy `mkvirtualenv` and `workon` commands we use below. It's optional, but this lesson assumes you have it:

```bash
sudo apt install python3-virtualenvwrapper
```

virtualenvwrapper usually needs a line or two added to your shell's startup file before `mkvirtualenv` works. Check the runtime README and the virtualenvwrapper docs for the exact lines.

---

## Step 3: turn on I2C

The IMU talks over I2C, which is off by default.

```bash
sudo raspi-config
```

Go to **Interface Options**, then **I2C**, and turn it on. Then give your user permission to use the I2C bus:

```bash
sudo usermod -aG i2c $USER
```

Log out and back in (or reboot) so the new group takes effect.

---

## Step 4: the FTDI latency rule

The servo board shows up as a USB-serial device. By default, Linux can hold on to small chunks of serial data for a short while before passing them on. That's fine for most things, but it makes servo replies arrive late, and the walking policy needs fresh readings many times a second.

The runtime fixes this with a **udev rule** that sets the latency timer to 1 ms. Create the file:

```bash
sudo nano /etc/udev/rules.d/99-usb-serial.rules
```

and put this line in it:

```text
SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
```

Save, then reboot. Note the `DRIVER=="ftdi_sio"` part: this rule only affects **FTDI** USB-serial adapters. Check which chip your servo board from the BOM uses.

---

## Step 5: a virtual environment

A virtual environment keeps the duck's Python packages separate from the system's.

```bash
mkvirtualenv -p python3 open-duck-mini-runtime
workon open-duck-mini-runtime
```

Every time you log in to work on the duck, run `workon open-duck-mini-runtime` first.

---

## Step 6: install the runtime

```bash
git clone https://github.com/apirrone/Open_Duck_Mini_Runtime
cd Open_Duck_Mini_Runtime
git checkout v2
pip install -e .
```

`git checkout v2` matters. The default branch is not the one this course uses. `pip install -e .` installs the runtime in editable mode, so if you pull updates later they take effect straight away.

---

## Using a Raspberry Pi 5

The runtime also works on a Pi 5, with one change. The Pi 5 has a new GPIO chip that the older `RPi.GPIO` library doesn't support, so swap it for `lgpio`:

```bash
pip uninstall -y RPi.GPIO
pip install lgpio
```

Run these inside the virtual environment.

---

## Try it Yourself

1. Run `git branch` inside `Open_Duck_Mini_Runtime` and check you're on `v2`.
2. Log out and back in, then run `groups`. Is `i2c` in the list?
3. Look around the repo with `ls scripts` to see the scripts you'll use in the next three lessons.

## Common Issues

**Problem**: `mkvirtualenv: command not found`.
**Solution**: Install `python3-virtualenvwrapper` and add its setup lines to your shell startup file, then open a new shell.
**Why**: virtualenvwrapper is a set of shell functions that only exist once your shell has loaded them.

**Problem**: `pip install -e .` fails.
**Solution**: Check you ran `workon open-duck-mini-runtime` first and are inside the `Open_Duck_Mini_Runtime` folder.
**Why**: Without the virtual environment, pip tries to install into the system Python.

**Problem**: You can't SSH in.
**Solution**: Check the Wi-Fi details you set in Imager, and find the Pi's IP address from your router.
**Why**: A Lite image has no screen to show you what went wrong.
