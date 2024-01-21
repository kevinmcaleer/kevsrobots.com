---
title: Build your own home server with Raspberry Pi 5
description: Learn how to set up a home server with Raspberry Pi 5 and Docker.
layout: project
date: 2024-01-21
cover: /assets/img/blog/homeserver/homeserver.jpg
excerpt: >-
    Learn how to set up a home server with Raspberry Pi 5 and Docker.
author: Kevin McAleer
difficulty: beginner
groups:
    - raspberry-pi
tags:
    - Docker
    - Raspberry Pi
    - Home Server
    - Home Lab
    - Clustered Pi
    - Docker Compose
    - Portainer
    - Mealie
    - NodeRed
videos:
    - NmpcKv91LLs
---

Welcome to the exciting world of Raspberry Pi, where small size doesn't mean limited power! Today, we're diving into the Raspberry Pi 5, a mighty mini-computer that's perfect for an array of projects. 

In this guide, we'll focus on turning your Raspberry Pi 5 into a versatile home server & home lab using Docker, a powerful tool for running applications in lightweight containers. We'll also explore how to enhance your setup with Docker Compose stacks from clustered-pi.com, a fantastic resource for Raspberry Pi enthusiasts.

---

> ## Homelab vs. Home Server
>
> A homelab is a collection of hardware and software used for learning and experimentation. A home server is a computer that provides services to other devices on a local network. In this guide, we'll be setting up a home server using a Raspberry Pi 5

---

## Preparing Your Raspberry Pi 5

Before diving into Docker, your Raspberry Pi 5 needs to be ready. Here’s how to set it up:

1. **SD Card Setup**: Start by flashing a Raspberry Pi OS image onto a microSD card using software like Raspberry Pi Imager.
1. **OS Installation**: Insert the microSD card into your Raspberry Pi and power it up. Follow the on-screen instructions to install the Raspberry Pi OS.
1. **Initial Configuration**: Connect your Raspberry Pi to your network. It’s advisable to set up SSH access for remote management.

---

## How to Install Docker on Raspberry Pi 5

Docker allows you to run applications in isolated environments. Here's how to get it on your Raspberry Pi:

1. **Update Your System**: Run `sudo apt update && sudo apt upgrade` to ensure your system is up-to-date.
1. **Install Docker**: Install Docker with `curl -sSL https://get.docker.com | sh`.
1. **Verify Installation**: Check Docker is installed correctly with `docker --version`.

A more robust way to install docker, which enables it to be run from a regular user account:

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

## Exploring Docker Compose

`Docker Compose` simplifies the process of running multi-container Docker applications. To install Docker Compose:

1. **Install Dependencies**: Ensure you have libffi-dev, python3-dev, and python3-pip installed; if not, run `sudo apt install libffi-dev python3-dev python3-pip`
1. **Install Docker Compose**: Use `sudo pip3 install docker-compose`
1. **Check Installation**: Verify with `docker-compose --version`

---

## Utilizing Docker Compose Stacks from Clustered-Pi.com

Clustered-pi.com is a website I created to document my Raspberry Pi Cluster; it offers pre-made Docker Compose stacks tailored for Raspberry Pi. To use them:

1. **Visit Clustered-Pi.com:** Browse the basic Docker and Raspbery Pi setup
1. **Download and Deploy**: Download the stack's docker-compose.yml file. Run docker-compose up -d in the directory containing the file to deploy the stack. The docker-compose files are available on GitHub: <https://www.github.com/kevinmcaleer/ClusteredPi> within the `stacks` folder
1. To install a stack simple change into the specific stack you want to install and run by typing:

```bash
docker-compose up -d
```

---

## Managing Your Home Server

Maintaining your server involves a few key practices:

* **Updating Containers**: Regularly update your Docker containers to get the latest features and security patches
* **Monitoring**: Keep an eye on server performance. Tools like `htop` and `docker stats` can be helpful
* **Troubleshooting**: Familiarize yourself with logs and basic troubleshooting steps for common issues

---

## Conclusion

Congratulations! You’ve turned your Raspberry Pi 5 into a powerful home server with Docker. This setup opens a world of possibilities, from media servers to personal cloud storage. Don’t hesitate to experiment and customize your server. Share your experiences, ask questions, or suggest improvements in the comments below. Happy tinkering!
