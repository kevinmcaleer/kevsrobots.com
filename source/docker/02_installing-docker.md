---
title: Installing Docker
description: Step-by-step guide to installing Docker on various operating systems.
layout: lesson
type: page

---

## Installing Docker on Your Machine

This lesson guides you through the installation process of Docker on different operating systems including Windows, MacOS, and Linux.

### Prerequisites

Before installing Docker, ensure your system meets the necessary requirements for the Docker edition you're installing. Generally, you need:

- A 64-bit processor with Second Level Address Translation (SLAT)
- 4GB system RAM
- BIOS-level hardware virtualization support must be enabled in the BIOS settings. For Windows, Hyper-V and Containers Windows features must be enabled.

### Installing Docker on Windows

To install Docker on Windows, follow these steps:

1. **System Requirements**:
   - Windows 10 64-bit: Pro, Enterprise, or Education (Build 15063 or later).
   - Hyper-V and Containers Windows features must be enabled.

2. **Download Docker Desktop for Windows**:
   - Visit the [Docker Hub](https://hub.docker.com/editions/community/docker-ce-desktop-windows/) and download the Docker Desktop installer.

3. **Installation Process**:
   - Run the installer and follow the instructions to install Docker Desktop on Windows.
   - Docker Desktop will start automatically once the installation is complete.

4. **Verifying the Installation**:
   - Open a command prompt or PowerShell, and run `docker --version` to check the Docker version.
   - Run `docker run hello-world` to verify that Docker can pull and run images.

### Installing Docker on MacOS

Follow these steps to install Docker on MacOS:

1. **System Requirements**:
   - Mac hardware must be a 2010 or newer model, with Intel’s hardware support for memory management unit (MMU) virtualization, including Extended Page Tables (EPT) and Unrestricted Mode.
   - macOS must be version 10.14 or newer.

2. **Downloading Docker Desktop for MacOS**:
   - Go to the [Docker Hub](https://hub.docker.com/editions/community/docker-ce-desktop-mac/) and download the Docker Desktop installer.

3. **Installation Instructions**:
   - Open the Docker.dmg file and drag the Docker icon to your Applications folder.
   - Run Docker from the Applications folder or Launchpad.

4. **How to Verify if Docker is Running Correctly**:
   - Open a terminal and type `docker --version` to see the installed Docker version.
   - Run `docker run hello-world` to ensure Docker can create and run containers.

### Installing Docker on Linux

Docker can be installed on various Linux distributions. Here’s a general guide:

1. **Installing Docker on Ubuntu, CentOS, Debian**:
   - Update your package index using `sudo apt-get update` (Ubuntu/Debian) or `sudo yum check-update` (CentOS).
   - Install Docker using the package manager with `sudo apt-get install docker-ce` (Ubuntu/Debian) or `sudo yum install docker-ce` (CentOS).
   - Start the Docker service with `sudo systemctl start docker`.

2. **Setting Up Docker to Run Without sudo**:
   - Add your user to the 'docker' group with `sudo usermod -aG docker $USER`.
   - Log out and log back in so that your group membership is re-evaluated.

3. **Verifying Docker Installation and Running a Test Container**:
   - Verify installation by running `docker --version`.
   - Test Docker installation by running `docker run hello-world`.

---

## How to Set Up Docker to Run Without sudo

The script below can be used to install Docker on a Raspberry Pi and configure it to run without sudo. This is useful for running Docker commands as a non-root user.

**Note** - use the `sudo` command to run the script as a superuser.

```bash
#!/bin/bash

# Define list of packages to install
pkgstoinstall=(libffi-dev libssl-dev python3 python3-pip)

# Update package cache and install packages
sudo apt-get update
sudo apt-get install -y "${pkgstoinstall[@]}"

# Download Docker convenience script if it doesn't exist
if [ ! -f /home/pi/get-docker.sh ]; then
    curl -fsSL https://get.docker.com -o /home/pi/get-docker.sh
fi

# Install Docker if not already installed
if [ ! -f /usr/bin/docker ]; then
    sh /home/pi/get-docker.sh
fi

# Add 'pi' user to the 'docker' group
sudo usermod -aG docker pi

# Unmask the Docker service
sudo systemctl unmask docker

# Fix permissions for Docker socket
sudo chmod 666 /var/run/docker.sock

# Install docker-compose if it doesn't exist
if [ ! -f /usr/local/bin/docker-compose ]; then
    sudo pip3 -v install docker-compose
fi

# Start Docker service
sudo systemctl start docker
```

This script automates the installation of Docker and its components on a Raspberry Pi, along with setting permissions to allow running Docker commands without requiring sudo access.

---
