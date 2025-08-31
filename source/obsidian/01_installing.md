---
title: Installing Obsidian
description: Learn how to install Obsidian on your computer.
type: page
layout: lesson
---

# Installing Obsidian

Obsidian is available for Windows, macOS, and Linux. Visit the official website <https://obsidian.md/> and download the installer for your operating system. Follow the installation instructions to get started.

Once installed, launch Obsidian and you'll be greeted with the option to create your first vault.

---

## Installing Obsidian on a Raspberry Pi

To install Obsidian on Raspberry Pi there are two main routes:

- [flatpak](#installing-via-flatpak)
- [pi-apps](#installing-via-pi-apps)

---

### Installing via flatpak

1. Install flatpak if you haven't already:

```sh
sudo apt install flatpak
```

2. Add the Flathub repository:

```sh
flatpak remote-add --if-not-exists flathub https://flathub.org/repo/flathub.flatpakrepo
```

3. Install Obsidian:

```sh
flatpak install flathub md.obsidian.Obsidian
```

---

### Installing via Pi-Apps

1. Open the [Pi-apps](/blogs/install_pi_apps) on your Raspberry Pi.
2. Search for "Obsidian" in the store.
3. Click "Install" and follow the prompts.

---

Once installed, launch Obsidian and you'll be greeted with the option to create your first vault.

---
