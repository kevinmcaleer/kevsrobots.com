---
layout: lesson
title: Preparing Your Raspberry Pi
author: Kevin McAleer
type: page
cover: /learn/openclaw_raspberry_pi/assets/openclaw.jpg
date: 2026-01-31
previous: 03_why-docker.html
next: 05_installing-docker.html
description: Get your Pi ready for OpenClaw installation.
percent: 25
duration: 4
navigation:
- name: OpenClaw on Raspberry Pi
- content:
  - section: Getting Started
    content:
    - name: Course Overview
      link: 00_intro.html
    - name: What is OpenClaw?
      link: 01_what-is-openclaw.html
    - name: Hardware Requirements
      link: 02_hardware-requirements.html
    - name: Why Docker?
      link: 03_why-docker.html
  - section: Installation & Setup
    content:
    - name: Preparing Your Raspberry Pi
      link: 04_preparing-your-pi.html
    - name: Installing Docker
      link: 05_installing-docker.html
    - name: Docker Compose Setup
      link: 06_docker-compose-setup.html
    - name: First Access & Configuration
      link: 07_first-access.html
  - section: Configuration & Usage
    content:
    - name: Configuring OpenClaw
      link: 08_configuring-openclaw.html
    - name: Your First Agent
      link: 09_first-agent.html
    - name: Understanding Skills
      link: 10_understanding-skills.html
  - section: Management & Troubleshooting
    content:
    - name: Managing Containers
      link: 11_managing-containers.html
    - name: Monitoring Resources
      link: 12_monitoring-resources.html
    - name: Troubleshooting
      link: 13_troubleshooting.html
  - section: Advanced Topics
    content:
    - name: Security Best Practices
      link: 14_security.html
    - name: Backup and Recovery
      link: 15_backup-and-recovery.html
    - name: Updates and Upgrades
      link: 16_updates-and-upgrades.html
    - name: Production Setup
      link: 17_production-setup.html
  - section: Summary
    content:
    - name: Next Steps
      link: 18_next-steps.html
---


## Preparing Your Raspberry Pi

Before we install OpenClaw, we need to prepare the Pi. This includes installing the OS and updating the system.

### Step 1: Write Raspberry Pi OS to microSD Card

#### Get the Tools

Download **Raspberry Pi Imager** from [raspberrypi.com/software](https://www.raspberrypi.com/software).

Available for:
- Windows
- macOS
- Ubuntu/Linux

#### Insert Your microSD Card

- Use a card reader
- Connect to your computer
- Make sure you identify the correct card (this is important!)

#### Open Raspberry Pi Imager

Launch the application you just downloaded.

#### Configure Your Image

1. Click **Choose Device** → Select your Pi model (Pi 4 or Pi 5)
2. Click **Choose OS** → Select **Raspberry Pi OS (Lite)**

**Why Lite?** No GUI = less resource usage. You'll access OpenClaw through the web, not a desktop.

3. Click **Choose Storage** → Select your microSD card

**⚠️ Warning:** Triple-check this is the right card. You cannot undo this.

#### Set Up Advanced Options

Click the gear icon (⚙️) to open advanced options:

- **Set hostname:** Something memorable (e.g., `openclaw-pi`)
- **Enable SSH:** ✅ Check this box
- **Set username and password:** Change from default pi/raspberry
- **Configure wireless LAN:** Optional but useful

Example hostname: `openclaw-pi`
Example username: `kev`

#### Write the Image

Click **Write** and confirm. This takes a few minutes.

Once complete, eject the card and insert it into your Pi.

### Step 2: Power On and Boot

- Plug in your power supply
- Give it 30-60 seconds to boot
- You should see LED activity

### Step 3: Connect Over SSH

#### Find Your Pi's IP Address

**Option A: Using a network scanner**
```bash
# On your computer, scan your network
nmap -sn 192.168.1.0/24 | grep -i raspber
```

Look for a hostname like `openclaw-pi.local` or `raspberrypi.local`.

**Option B: Check your router**
Log into your router's admin interface and look for connected devices.

**Option C: Connect directly**
If you have a monitor and keyboard, connect them to your Pi and see the IP address at boot time.

#### SSH In

```bash
ssh kev@openclaw-pi.local
```

Or using IP address:
```bash
ssh kev@192.168.1.100
```

(Replace `kev` with your username and IP with yours)

You'll be prompted for your password. Enter it.

#### Verify Connection

You should see a prompt like:
```
kev@openclaw-pi:~ $
```

Great! You're in.

### Step 4: Update the System

**Critical step.** Don't skip this.

```bash
sudo apt update
sudo apt upgrade -y
```

This:
- Updates package lists
- Upgrades all installed packages
- Includes security patches

On a Pi with internet connection, this typically takes 5-15 minutes depending on how many updates are available.

**What it's doing:**
```
Setting up libssl-dev (3.0.x-y_armhf) ...
Setting up libpython3.11-dev (3.11.x-y_armhf) ...
Processing triggers for libc-bin (2.36-9+rpt2_armhf) ...
```

Let it finish. Don't interrupt it.

### Step 5: Check Disk Space

```bash
df -h
```

You should see:
```
Filesystem      Size  Used Avail Use% Mounted on
/dev/root        29G  2.8G   25G  10% /
```

**Key numbers:**
- **Size:** Total space (should be ~29G for 32GB card)
- **Used:** Currently used (~2-3G for OS)
- **Avail:** Free space (should be ~25G)
- **Use%:** Percentage used (should be <15%)

If you're below 10% available, you don't have enough space. Consider a larger card.

### Step 6: Check Memory

```bash
free -h
```

Output example:
```
               total        used        free      shared  buff/cache   available
Mem:           7.6Gi       200Mi       7.0Gi        39Mi       386Mi       7.2Gi
```

**Key number:**
- **total:** Your Pi's RAM (should match what you bought)
- **available:** Actual usable RAM (usually close to total)

If you see less than 2GB, you have a problem with your Pi.

### Step 7: Check Network

```bash
ping -c 3 8.8.8.8
```

This pings Google's DNS. If it works:
```
PING 8.8.8.8 (8.8.8.8) 56(84) bytes of data.
64 bytes from 8.8.8.8: icmp_seq=1 ttl=119 time=12.3 ms
```

If it fails, your network isn't working. Check:
- Ethernet cable is connected
- Wi-Fi password is correct
- Router is working

### Step 8: Optional - Set Static IP

If you want your Pi to always have the same IP (helpful for reliability):

```bash
sudo nano /etc/dhcpcd.conf
```

Add at the end:
```
interface eth0
static ip_address=192.168.1.100/24
static routers=192.168.1.1
static domain_name_servers=8.8.8.8
```

(Adjust IP and router IP to match your network)

Save (Ctrl+X, Y, Enter) and reboot:
```bash
sudo reboot
```

### You're Ready!

Your Pi is now prepared and ready for Docker installation. Let's move on to the next step.

---
