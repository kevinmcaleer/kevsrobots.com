---
layout: lesson
title: Installing Docker
author: Kevin McAleer
type: page
cover: /learn/openclaw_raspberry_pi/assets/openclaw.jpg
date: 2026-01-31
previous: 04_preparing-your-pi.html
next: 06_docker-compose-setup.html
description: Install Docker and Docker Compose on your Raspberry Pi.
percent: 30
duration: 3
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


## Installing Docker

Docker is the foundation for running OpenClaw safely and cleanly. Let's install it.

### Step 1: Install Docker

Run this command on your Pi:

```bash
curl -fsSL https://get.docker.com | sh
```

This official script:
- Detects your Pi's architecture (ARM)
- Downloads the right Docker version
- Installs Docker
- Sets up the daemon

Takes about 2-5 minutes. Let it finish.

### Step 2: Add Your User to Docker Group

By default, you need `sudo` to run Docker commands. To avoid this:

```bash
sudo usermod -aG docker kev
```

(Replace `kev` with your username)

### Step 3: Log Out and Back In

The group change requires a fresh login:

```bash
exit
```

Then SSH back in:

```bash
ssh kev@openclaw-pi.local
```

### Step 4: Verify Docker Installation

```bash
docker run hello-world
```

Expected output:
```
Hello from Docker!
This message shows that your installation appears to be working correctly.
```

If you see this, Docker is working!

### Step 5: Install Docker Compose

Check if it's already installed:

```bash
docker-compose --version
```

If not installed, install it:

```bash
sudo apt install docker-compose -y
```

Verify:

```bash
docker-compose --version
```

You should see:
```
Docker Compose version v1.29.2
```

(Version number may differ)

### Step 6: Enable Docker Daemon on Boot

So Docker starts automatically when your Pi boots:

```bash
sudo systemctl enable docker
sudo systemctl start docker
```

Verify it's running:

```bash
sudo systemctl status docker
```

You should see:
```
● docker.service - Docker Application Container Engine
     Loaded: loaded (/lib/systemd/system/docker.service; enabled; vendor preset: enabled)
```

### Troubleshooting

#### Permission Denied Errors

If you still get permission errors after adding your user to the docker group:

```bash
# Re-login completely
exit
ssh kev@openclaw-pi.local

# Or start a fresh shell
newgrp docker
```

#### Out of Memory During Installation

On Pi with 2GB RAM, installation might struggle. If it hangs:

1. Interrupt it (Ctrl+C)
2. Create a swap file:

```bash
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

3. Try again:

```bash
curl -fsSL https://get.docker.com | sh
```

### Container Runtime Information

Check your setup:

```bash
docker info
```

Look for:
- **Containers:** (should be 1 from hello-world test)
- **Memory:** (total system memory)
- **Storage Driver:** overlay2 (preferred)

### You're Ready!

Docker is installed and ready. Next, we'll set up Docker Compose to run OpenClaw.

---
