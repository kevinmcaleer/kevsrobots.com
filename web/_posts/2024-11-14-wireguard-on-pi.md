---
title: Raspberry Pi WireGuard VPN Setup with Docker
description: >-
    Step-by-step guide to setting up WireGuard on a Raspberry Pi using Docker.
excerpt:
    Learn how to set up a secure WireGuard VPN on your Raspberry Pi using Docker, allowing remote access to your home network securely and easily.
layout: showcase
date: 2024-11-14
author: Kevin McAleer
difficulty: intermediate
cover: /assets/img/blog/wireguard-pi/cover.jpg
hero: /assets/img/blog/wireguard-pi/hero.png
mode: light
tags: 
 - Raspberry Pi
 - WireGuard
 - Docker
 - VPN
 - Network Security
groups:
 - raspberrypi
---

## What is WireGuard?

WireGuard is a modern VPN protocol that’s lightweight, fast, and easy to configure. It’s designed for simplicity and efficiency, providing secure connections with minimal overhead, making it ideal for low-power devices like the Raspberry Pi.

---

## Why Use Docker on a Raspberry Pi?

Using Docker for WireGuard makes setup and management easier by:
1. **Simplifying configuration**: Docker encapsulates all dependencies.
2. **Enhancing portability**: You can replicate the setup on other devices.
3. **Ease of use**: Docker’s lightweight containerization is perfect for the Raspberry Pi’s limited resources.

---

## Prerequisites

To set up WireGuard on your Raspberry Pi, you’ll need:
1. A Raspberry Pi running Raspberry Pi OS.
2. Docker and Docker Compose installed.
3. A public IP address or Dynamic DNS (DDNS) if using this setup outside your local network.
4. Port forwarding enabled on your router for WireGuard’s UDP port (51820 by default).

---

## Installing Docker on the Raspberry Pi

### Step 1: Install Docker

If Docker isn’t already installed, you can do so with:

```bash
curl -sSL https://get.docker.com | sh
sudo usermod -aG docker $USER
```

Reboot your Raspberry Pi to finalize the installation:

```bash
sudo reboot
```

### Step 2: Install Docker Compose

Docker Compose will make it easy to manage WireGuard and its configuration. Install it with:

```bash
sudo apt update
sudo apt install -y docker-compose
```

---

## Setting Up WireGuard with Docker Compose

### Step 1: Create a Working Directory

Navigate to your preferred directory and create a folder for the WireGuard configuration:

```bash
mkdir wireguard-docker && cd wireguard-docker
```

### Step 2: Create a `docker-compose.yml` File

In this folder, create a `docker-compose.yml` file:

```yaml
version: '3.8'

services:
  wireguard:
    image: linuxserver/wireguard
    container_name: wireguard
    cap_add:
      - NET_ADMIN
      - SYS_MODULE
    environment:
      - PUID=1000
      - PGID=1000
      - TZ=Etc/UTC
      - SERVERURL=your_public_ip_or_ddns # Replace with your public IP or DDNS
      - SERVERPORT=51820
      - PEERS=1 # Number of clients
      - PEERDNS=1.1.1.1 # or your preferred DNS
    volumes:
      - ./config:/config
      - /lib/modules:/lib/modules
    ports:
      - 51820:51820/udp
    sysctls:
      - net.ipv4.conf.all.src_valid_mark=1
    restart: unless-stopped
```

Replace `your_public_ip_or_ddns` with either your public IP address or a Dynamic DNS hostname if you want access outside your network.

### Step 3: Adjusting the Permissions

To make sure Docker has the correct permissions, set the correct user and group IDs:

```bash
export PUID=$(id -u)
export PGID=$(id -g)
```

---

## Starting WireGuard

Run the following command to start the WireGuard container:

```bash
sudo docker-compose up -d
```

The container will generate WireGuard configuration files and keys for each peer, stored in the `config` folder.

---

## Configuring Your WireGuard Client

### Step 1: Locate the Peer Configuration

In the `config` folder, you’ll find configuration files for each peer (client). Open the configuration file (e.g., `peer1.conf`).

### Step 2: Import the Configuration

On your device (e.g., smartphone, laptop), download the WireGuard app and import the `.conf` file for your client.

---

## Setting Up Port Forwarding

To access your Raspberry Pi VPN outside your local network, log into your router’s admin page and forward port 51820 (UDP) to your Raspberry Pi’s local IP address.

---

## Testing the VPN Connection

1. Open the WireGuard app on your client device.
2. Toggle the connection on to test your VPN setup.

You should now have secure, encrypted access to your home network via WireGuard!

---

## Managing the WireGuard Container

### Stopping the Container

To stop the WireGuard container, run:

```bash
sudo docker-compose down
```

This will stop and remove the container but keep your configuration files.

### Restarting the Container

To restart WireGuard, simply run:

```bash
sudo docker-compose up -d
```

---

## Conclusion

With Docker, setting up a secure WireGuard VPN on a Raspberry Pi is simple and efficient. You can now enjoy safe, encrypted access to your network from anywhere in the world.

---
