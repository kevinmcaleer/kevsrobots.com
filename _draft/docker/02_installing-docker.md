---
title: Installing Docker
description: Step-by-step guide to installing Docker on various operating systems.
layout: lesson
type: page

---

## Installing Docker on Your Machine

This lesson guides you through the installation process of Docker on different operating systems including Windows, MacOS, and Linux.

### Prerequisites

Before installing Docker, ensure your system meets the necessary requirements for the Docker edition you're installing.

### Installing Docker on Windows

- Steps to download Docker Desktop for Windows.
- System requirements and installation process.
- Verifying the installation.

### Installing Docker on MacOS

- Downloading Docker Desktop for MacOS.
- Installation instructions and system requirements.
- How to verify if Docker is running correctly.

### Installing Docker on Linux

- Installing Docker on various Linux distributions (Ubuntu, CentOS, Debian).
- Setting up Docker to run without sudo.
- Verifying Docker installation and running a test container.

---

## How to set up Docker to run without sudo

The script below can be used to install Docker on a Raspberry Pi and configure it to run without sudo. This is useful for running Docker commands as a non-root user.

```bash
#!/bin/bash

# Elevate privileges
sudo su

# Define list of packages to install
pkgstoinstall=(libffi-dev libssl-dev python3 python3-pip)

# Update package cache and install packages
sudo apt-get update
sudo apt-get install -y "${pkgstoinstall[@]}"

# Remove the Python-configparser package
sudo apt-get remove -y python-configparser

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

---
