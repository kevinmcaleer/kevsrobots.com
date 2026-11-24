---
title: Installing the Operating System
description: Prepare your Raspberry Pi nodes with a lightweight, 64-bit Linux OS ready for Kubernetes and K3s.
layout: lesson
type: page
cover: /learn/k3s/assets/banners/03_os_installation.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to flash a 64-bit Raspberry Pi OS card for each node with its own hostname and SSH access, and turn on the memory cgroup that K3s needs.

---

## Which operating system?

K3s runs on several 64-bit Linux systems. In this course I use **Raspberry Pi OS Lite (64-bit)**, the headless version with no desktop, because it is the official, well-supported choice and it matches the other Pi courses on this site. Ubuntu Server for Raspberry Pi also works well, and the steps below note where it differs.

Pick the **64-bit** image. Container images for Pis are mostly published for `arm64`, and a 32-bit OS cuts you off from many of them.

```mermaid
flowchart LR
    A["Flash Pi OS Lite 64-bit<br/>with Imager settings"] --> B["Boot and ssh in"]
    B --> C["Update packages"]
    C --> D["Enable memory cgroup"]
    D --> E["Reboot and check"]
```

---

## Step 1: Flash each card

Use **Raspberry Pi Imager** from raspberrypi.com/software. For each Pi:

1. Choose your Pi model, then **Raspberry Pi OS Lite (64-bit)** (it is under Raspberry Pi OS (other)).
2. Choose the card or SSD.
3. When asked to customise, set the **hostname** (`pi-server`, `pi-node1`, `pi-node2`), a **username and password**, and **enable SSH**. Add your Wi-Fi details only if you must, though we will use Ethernet.
4. Write the card, put it in the Pi, and power it on.

Setting the hostname here means every card is unique from the start. This matters later: K3s identifies each node by its hostname, and two nodes with the same name will clash.

You may see old guides telling you to create an empty file called `ssh` on the boot partition. Imager's customisation does that job for you, so you do not need it.

---

## Step 2: First boot

Give the Pi a minute or two, then from your computer:

```bash
ssh pi@pi-server.local
```

Replace `pi` with the username you chose. If the name does not resolve, find the Pi's address in your router's device list and use that instead. Then update the system:

```bash
sudo apt update && sudo apt full-upgrade -y
```

> On Ubuntu Server the same commands work, and the first login asks you to change a default password.

---

## Step 3: Enable the memory cgroup

Kubernetes limits and measures memory with Linux *control groups* (cgroups). On Raspberry Pi OS the memory cgroup is switched off by default, and K3s will refuse to start without it, with an error asking you to add `cgroup_memory=1 cgroup_enable=memory` to the kernel command line.

The kernel command line lives in `cmdline.txt`. On current Raspberry Pi OS (Bookworm and later) it is at `/boot/firmware/cmdline.txt`. On older releases it was `/boot/cmdline.txt`. Ubuntu for the Pi also uses `/boot/firmware/cmdline.txt`.

This file must stay on **one single line**. Append the two settings to the end of that line with a space before them:

```bash
sudo sed -i '$ s/$/ cgroup_memory=1 cgroup_enable=memory/' /boot/firmware/cmdline.txt
cat /boot/firmware/cmdline.txt
```

The `cat` should show one long line ending in `cgroup_memory=1 cgroup_enable=memory`. If you see two lines, or the settings are on a line of their own, fix it with `sudo nano /boot/firmware/cmdline.txt`. Then reboot:

```bash
sudo reboot
```

After the Pi is back, check that the memory controller is enabled:

```bash
grep memory /proc/cgroups
```

The line that starts with `memory` should end with a `1` in its last column (enabled). If it ends in `0`, the setting did not take.

---

## Verify

Before moving on, on every node:

- You can `ssh` in using the hostname you chose
- `hostname` prints the right name
- `grep memory /proc/cgroups` ends in `1`

> **Teachers**: Flashing cards is the slow step in a class. Prepare the cards beforehand, one per Pi, and label each card and Pi with its hostname (a coloured sticker works well). Make sure learners write down the username and password they set, since a forgotten password means reflashing. Editing `cmdline.txt` is a good moment for a pair check, because one stray line break stops the Pi booting properly.

---

## Try it Yourself

1. Flash a card, boot it, and `ssh` in by hostname.
2. Run `uname -m` and confirm it prints `aarch64`, the sign of a 64-bit OS.
3. Challenge: run `free -h` before and after enabling the cgroup. Does available memory change much?

---

## Common Issues

- **Problem**: `ssh: Could not resolve hostname pi-server.local`.
- **Solution**: Use the Pi's IP address from your router, and check your computer supports `.local` names (mDNS).
- **Why**: `.local` names rely on multicast DNS, which some networks and operating systems block.

- **Problem**: The Pi does not boot after editing `cmdline.txt`.
- **Solution**: Put the card in your computer and make sure the file is still a single line.
- **Why**: The kernel reads the whole file as one command line, so a stray line break can split or lose settings.

- **Problem**: K3s later complains `Failed to find memory cgroup`.
- **Solution**: Check you edited `/boot/firmware/cmdline.txt`, not `/boot/cmdline.txt`, then reboot.
- **Why**: Newer releases boot from `/boot/firmware`, so edits to the old path are ignored.

---

## Check your understanding

{% quiz %}
? Where is the kernel command line file on current Raspberry Pi OS?
- [ ] /etc/cmdline.txt
- [x] /boot/firmware/cmdline.txt
- [ ] /home/pi/cmdline.txt
> Bookworm and later use /boot/firmware. Older releases used /boot.

? Why must `cmdline.txt` stay on one line?
- [ ] Imager rejects multi-line files
- [x] The kernel reads it as a single command line
- [ ] SSH needs it
> A line break splits the command line and can lose your settings.

? Why do we set a different hostname for each Pi when flashing?
- [x] K3s identifies nodes by hostname, so duplicates clash
- [ ] The Pi will not boot without one
- [ ] It makes the Pi faster
> Unique names avoid duplicate node errors when joining workers.
{% endquiz %}
