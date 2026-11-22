---
title: Preparing Your Raspberry Pi
description: Flash Raspberry Pi OS Lite (64-bit), connect over SSH, update the system, set the timezone, check for a 64-bit kernel and add swap on small Pis
layout: lesson
type: page
cover: /learn/openclaw_raspberry_pi/assets/banners/04_preparing-your-pi.jpg
date_updated: 2026-11-22
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

In this lesson we'll turn a blank card or SSD into a headless Pi called `openclaw-pi` that you control over SSH. By the end it will be up to date, on the right timezone, confirmed 64-bit, and ready for Docker.

---

## The plan

```mermaid
flowchart LR
    A["Flash OS with Imager"] --> B["First boot"]
    B --> C["SSH in"]
    C --> D["Update and install tools"]
    D --> E["Timezone and 64-bit check"]
    E --> F["Swap if 2 GB or less"]
```

---

## Step 1: Flash Raspberry Pi OS Lite (64-bit)

We want **Raspberry Pi OS Lite (64-bit)**. "Lite" means no desktop, which saves memory, and you'll reach everything over the network anyway. The 64-bit part is not optional: OpenClaw needs a 64-bit OS.

On **your computer**:

1. Download and open [Raspberry Pi Imager](https://www.raspberrypi.com/software/).
2. Choose your device (Pi 4 or Pi 5).
3. Choose the OS **Raspberry Pi OS Lite (64-bit)**. It may be tucked away under **Raspberry Pi OS (other)**.
4. Choose your microSD card or USB SSD as the storage. Double-check it's the right drive, because it will be wiped.
5. When Imager offers to customise the settings, fill these in:

| Setting | What to enter |
| --- | --- |
| Hostname | `openclaw-pi` |
| Username and password | A username of your choice and a strong password |
| Wi-Fi | Your network name, password and country (skip if you're using Ethernet) |
| Locale | Your timezone and keyboard layout |
| SSH | Turn it on, with password authentication |
{:class="table table-single"}

6. Write the image, then put the card or SSD in the Pi and power it up.

Throughout this course we write your username as `<user>`. Whenever you see `<user>`, type the username you just chose instead.

> **Teachers**: if several Pis share one network, give each a different hostname, such as `openclaw-pi-01`, or students will SSH into each other's Pis.

---

## Step 2: First SSH connection

Give the Pi a couple of minutes for its first boot. It resizes the file system and may restart once.

On **your computer**, open a terminal (PowerShell on Windows, Terminal on macOS or Linux) and connect:

```bash
ssh <user>@openclaw-pi.local
```

The first time, SSH asks whether you trust this computer. Type `yes`, then enter your password. When you see a prompt like this, you're in:

```text
<user>@openclaw-pi:~ $
```

If `openclaw-pi.local` can't be found, look up the Pi's IP address in your router's list of connected devices and use that instead, for example `ssh <user>@192.168.1.50`.

Everything from here is typed **on the Pi**, in this SSH session, unless it says otherwise.

---

## Step 3: Update the system and install git and curl

Let's bring every package up to date, then add the two tools we need later. `git` downloads the OpenClaw repository and `curl` fetches the Docker installer.

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y git curl
```

The upgrade can take a while on a fresh install. Let it finish. If it updated the kernel, restart and SSH back in:

```bash
sudo reboot
```

---

## Step 4: Set the timezone

Pi Butler will send you a morning brief at 7 am, so the Pi needs to know what "7 am" means where you live. Use your own IANA timezone name. Here's London:

```bash
sudo timedatectl set-timezone Europe/London
timedatectl
```

Not sure of your zone's name? List them all, then scroll or search:

```bash
timedatectl list-timezones
```

Remember the name you picked. We'll give the same one to OpenClaw in lesson 6.

---

## Step 5: Check you're on 64-bit

```bash
uname -m
```

You should see:

```text
aarch64
```

If you see `armv7l` instead, you flashed the 32-bit OS. Go back to Step 1 and choose the 64-bit Lite image.

While you're here, check memory and disk space:

```bash
free -h
df -h /
```

---

## Step 6: Add swap if your Pi has 2 GB or less

Swap is disk space the Pi borrows as extra memory when RAM runs out. It's slow, but it stops programs being killed. The OpenClaw guide says to add it on any Pi with 2 GB or less. On a 4 GB or 8 GB Pi you can skip this step.

```bash
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

The last line makes the swap file come back after a reboot. Now tell Linux to use swap only when it really needs to, which is kinder to SD cards:

```bash
echo 'vm.swappiness=10' | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

Check it worked. The `Swap` row should now show about 2 GB:

```bash
free -h
```

---

## Step 7 (optional): Stop Wi-Fi from dozing off

If your Pi is on Wi-Fi, its power saving can make it drop off the network now and then. The OpenClaw guide's fix is:

```bash
sudo iwconfig wlan0 power off
```

This lasts until the next reboot. If drop-outs keep happening, Ethernet is the reliable answer.

## Pi Butler progress

Pi Butler now has a home: a 64-bit Pi called `openclaw-pi`, up to date, on your timezone, and reachable over SSH.

---

## Try it Yourself

1. **Confirm the basics.** Run `uname -m`, `timedatectl` and `free -h`, and note the architecture, timezone and total memory.
2. **Reconnect.** Type `exit` to leave SSH, then connect again with `ssh <user>@openclaw-pi.local`. Does it ask for `yes` a second time? Why not?
3. **Challenge:** Set up SSH key login so you don't need a password. Run `ssh-keygen` on your computer, then `ssh-copy-id <user>@openclaw-pi.local`, and connect again.

---

## Common Issues

**Problem**: `ssh: Could not resolve hostname openclaw-pi.local`.
**Solution**: Wait a minute after power-on, then try again, or use the Pi's IP address from your router.
**Why**: Some networks don't pass on `.local` names, and the Pi takes a while to appear on first boot.

**Problem**: `uname -m` shows `armv7l`.
**Solution**: Re-flash with Raspberry Pi OS Lite (64-bit).
**Why**: That's a 32-bit system. OpenClaw needs 64-bit, and its Docker images are built for `arm64`.

**Problem**: SSH says `WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED!` after you re-flashed.
**Solution**: On your computer, run `ssh-keygen -R openclaw-pi.local`, then connect again.
**Why**: The fresh OS has new SSH keys, and your computer remembers the old ones.

---

## Check your understanding

{% quiz %}
? Which OS image should you flash for this course?
- [ ] Raspberry Pi OS with desktop (32-bit)
- [x] Raspberry Pi OS Lite (64-bit)
- [ ] Any OS, OpenClaw runs everywhere
> Lite skips the desktop to save memory, and OpenClaw needs a 64-bit OS.

? What should `uname -m` print on a correctly set up Pi?
- [ ] armv7l
- [ ] x86_64
- [x] aarch64
> aarch64 means a 64-bit ARM kernel, which is what OpenClaw needs.

? Why do we set the Pi's timezone?
- [ ] Docker won't install without it
- [x] So scheduled jobs like a 7 am brief run at your local 7 am
- [ ] It makes the Pi run faster
> Reminders and cron jobs depend on knowing the local time.
{% endquiz %}
