---
layout: lesson
title: Installing Podman
author: Kevin McAleer
type: page
cover: assets/podman03.jpg
date: 2025-05-24
previous: 02_docker_vs_podman_architecture.html
next: 04_cli_compatibility_and_commands.html
description: Learn how to install Podman on Linux, macOS, and Windows, and verify
  your setup for development or production use.
percent: 21
duration: 3
date_updated: 2025-05-24
navigation:
- name: From Docker to Podman
- content:
  - section: Getting Started
    content:
    - name: Introduction to Podman
      link: 01_intro_to_podman.html
    - name: Docker vs Podman Architecture
      link: 02_docker_vs_podman_architecture.html
    - name: Installing Podman
      link: 03_installing_podman.html
  - section: Core Usage
    content:
    - name: CLI Compatibility and Basic Commands
      link: 04_cli_compatibility_and_commands.html
    - name: "Podman Features Docker Doesn\u2019t Have"
      link: 05_podman_unique_features.html
    - name: Managing Containers and Images
      link: 06_managing_containers_and_images.html
  - section: Advanced Concepts
    content:
    - name: Working with Pods
      link: 07_working_with_pods.html
    - name: Docker Compose vs Podman Compose
      link: 08_docker_compose_vs_podman_compose.html
    - name: Podman and Kubernetes
      link: 09_podman_and_kubernetes.html
  - section: Real-World Usage
    content:
    - name: Podman in Production
      link: 10_podman_in_production.html
    - name: Troubleshooting and Debugging
      link: 11_troubleshooting_and_debugging.html
  - section: Migration Lab
    content:
    - name: Migration Project
      link: 12_migration_project.html
  - section: Summary
    content:
    - name: Wrap-up and Future Outlook
      link: 13_wrap_up_and_future.html
---


![Cover]({{page.cover}}){:class="cover"}

---

Before using Podman, you need to get it installed and configured on your system. Podman supports Linux natively and also runs on macOS and Windows using virtual machines.

Let’s walk through how to install Podman on your platform of choice.

---

## 🐧 Installing Podman on Linux

### 📦 On Fedora, CentOS, or RHEL

```bash
sudo dnf install podman
````

### 🐧 On Ubuntu or Debian

```bash
sudo apt update
sudo apt install podman
```

### 🏗️ On Arch Linux

```bash
sudo pacman -S podman
```

> ✅ **Tip:** Use your system's package manager for the latest stable version, or check the [Podman GitHub Releases](https://github.com/containers/podman/releases) for newer versions.

---

## 🍏 Installing Podman on macOS

### Step 1: Install Podman via Homebrew

```bash
brew install podman
```

### Step 2: Initialize the VM

```bash
podman machine init
podman machine start
```

> 🛠️ Podman on macOS runs inside a lightweight Linux VM (using QEMU or Apple’s Virtualization framework).

---

## 🪟 Installing Podman on Windows

### Option 1: Use the Windows Installer

* Download from [https://podman.io](https://podman.io)
* Includes Podman Machine (QEMU backend)

### Option 2: Use WSL2 (Recommended)

```bash
wsl --install
wsl --set-default-version 2
```

Then inside WSL (Ubuntu):

```bash
sudo apt update
sudo apt install podman
```

> 🧰 **Note:** Podman Desktop is available for both macOS and Windows with a GUI interface and built-in machine management.

---

## 🍓 Install on Raspberry Pi OS
### Step 1: Update Your System

```bash
sudo apt update
sudo apt upgrade -y
```

---

### Step 2: Install Podman

```bash
sudo apt install -y podman
```

---

### Step 3: Verify Installation

```bash
podman --version
```

---

### Step 4: Configure Podman for Rootless Use

```bash
podman system migrate
```

---

### Step 5: Start Podman Machine (if using)

```bash
podman machine init
podman machine start
```

---

## ✅ Verifying Your Installation

To check that Podman is working:

```bash
podman --version
podman info
podman run hello-world
```

If everything is set up correctly, you’ll see Podman pull the image and run it just like Docker.

---

## 📦 Bonus: Installing Podman Compose

If you plan to use `docker-compose`-style workflows:

```bash
pip3 install podman-compose
```

> 🔁 Podman Compose is a community-supported tool that interprets Docker Compose files using Podman under the hood.

---

Next up: [CLI Compatibility and Basic Commands](04_cli_compatibility_and_commands)

---
