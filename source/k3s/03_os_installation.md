---
title: Installing the Operating System
description: Prepare your Raspberry Pi nodes with a lightweight, 64-bit Linux OS ready for Kubernetes and K3s.
layout: lesson
type: page
cover: assets/k3s02.jpg
date_updated: 2025-05-24
---

![Cover]({{page.cover}}){:class="cover"}

---

To run K3s reliably on Raspberry Pi, you’ll need a clean, minimal 64-bit Linux OS — with a few key configuration tweaks.

In this lesson, we’ll walk through choosing an operating system, installing it on each Pi, and preparing for clustering.

---

## 🧠 Recommended Operating Systems

| OS                            | Notes                               |
|-------------------------------|-------------------------------------|
| **Ubuntu Server 22.04+**      | Well-supported, minimal, 64-bit ARM |
| Raspberry Pi OS (64-bit Lite) | Official, stable, but less minimal  |
| DietPi or Arch Linux          | For advanced users, highly minimal  |
{:class="table table-striped"}

> ⚠️ Ensure you're using a **64-bit image**, not 32-bit — Kubernetes requires 64-bit support.

---

## 💽 Step 1: Flash the OS

### Tools You Can Use:

- [Raspberry Pi Imager](https://www.raspberrypi.com/software/)
- Balena Etcher
- `dd` (advanced)

### Process:

1. Download the 64-bit OS image (e.g., Ubuntu Server 22.04 LTS for ARM).
2. Flash the image to each SD card or SSD.
3. Enable SSH:
   - For Raspberry Pi OS: place an empty file named `ssh` in the `/boot` partition.
   - For Ubuntu Server: SSH is enabled by default.

---

## 🧑‍💻 Step 2: First Boot and Configuration

1. Insert the card/SSD into each Pi.
2. Power up and SSH in:

   ```bash
   ssh ubuntu@<pi-ip-address>
   ```

   Default password: `ubuntu` (you'll be prompted to change it)

3. Update the system:

   ```bash
   sudo apt update && sudo apt upgrade -y
   ```

4. Set a hostname for each Pi:

   ```bash
   sudo hostnamectl set-hostname pi-master
   sudo reboot
   ```

Use consistent names like `pi-master`, `pi-node1`, `pi-node2`, etc.

---

## 🌐 Step 3: Set Static IP or DHCP Reservation

On your router:

- Reserve an IP for each Raspberry Pi based on its MAC address
- This ensures stable node communication in the cluster

Or manually edit netplan (Ubuntu):

```yaml
network:
  ethernets:
    eth0:
      dhcp4: no
      addresses: [192.168.1.101/24]
      gateway4: 192.168.1.1
      nameservers:
        addresses: [1.1.1.1, 8.8.8.8]
  version: 2
```

Save to `/etc/netplan/00-installer-config.yaml` and apply:

```bash
sudo netplan apply
```

---

## 🧹 Step 4: Recommended Cleanup and Config

- Enable cgroups (required for Kubernetes):
  Check `/boot/firmware/cmdline.txt` (Ubuntu) or `/boot/cmdline.txt` (Pi OS):

  Add or confirm:

  ```text
  cgroup_enable=cpuset cgroup_memory=1 cgroup_enable=memory
  ```

- Reboot:

  ```bash
  sudo reboot
  ```

---

## 🧪 Verify

After reboot, confirm:

- You can SSH into all nodes
- Hostnames are correct
- Static IPs are assigned
- `cgroups` are enabled

---

✅ Now your Pi cluster is ready for **K3s installation**!

Next up: [Installing K3s](05_installing_k3s)

---
