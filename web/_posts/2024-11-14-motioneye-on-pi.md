---
title: Raspberry Pi MotionEye Camera Setup with Docker
description: >-
    Step-by-step guide to setting up MotionEye on a Raspberry Pi using Docker.
excerpt:
    Learn how to set up MotionEye on your Raspberry Pi with Docker to turn it into a network camera server, perfect for monitoring home security or creating a DIY surveillance system.
layout: showcase
date: 2024-11-14
author: Kevin McAleer
difficulty: intermediate
cover: /assets/img/blog/motioneye-pi/cover.jpg
hero: /assets/img/blog/motioneye-pi/hero.png
mode: light
tags: 
 - Raspberry Pi
 - MotionEye
 - Docker
 - Surveillance
 - Network Camera
groups:
 - raspberrypi
---

## What is MotionEye?

MotionEye is a web-based frontend for the motion daemon, making it easy to turn network cameras or USB cameras into a home surveillance system. It allows you to set up motion detection, alerts, and image recording in a user-friendly interface accessible through a browser.

---

## Why Use Docker on a Raspberry Pi?

Docker simplifies the setup process for applications like MotionEye by:
1. **Streamlining installation**: Docker contains all dependencies in one container.
2. **Isolating services**: MotionEye runs in a standalone container, avoiding conflicts.
3. **Portability**: Docker allows easy movement of configurations between devices.

---

## Prerequisites

For setting up MotionEye on your Raspberry Pi, you’ll need:
1. A Raspberry Pi with Raspberry Pi OS installed.
2. Docker and Docker Compose installed on the Raspberry Pi.
3. A compatible camera (USB or Pi Camera Module).
4. Network access to the Raspberry Pi for remote viewing.

---

## Installing Docker on the Raspberry Pi

### Step 1: Install Docker

If Docker isn’t already installed, add it with the following commands:

```bash
curl -sSL https://get.docker.com | sh
sudo usermod -aG docker $USER
```

Reboot the Raspberry Pi after installation:

```bash
sudo reboot
```

### Step 2: Install Docker Compose

To manage MotionEye with Docker Compose, install Docker Compose:

```bash
sudo apt update
sudo apt install -y docker-compose
```

---

## Setting Up MotionEye with Docker Compose

### Step 1: Create a Working Directory

Create a directory for MotionEye and navigate into it:

```bash
mkdir motioneye-docker && cd motioneye-docker
```

### Step 2: Create a `docker-compose.yml` File

In this directory, create a `docker-compose.yml` file for MotionEye:

```yaml
version: '3.3'

services:
  motioneye:
    image: ccrisan/motioneye:master-amd64
    container_name: motioneye
    ports:
      - "8765:8765"  # Adjust port if needed
    volumes:
      - ./motioneye:/etc/motioneye
      - /etc/localtime:/etc/localtime:ro
      - /data/motioneye:/var/lib/motioneye
    devices:
      - /dev/video0:/dev/video0  # Use the correct device for your camera
    restart: unless-stopped
```

The default port is `8765`, and `/dev/video0` is the default device for USB cameras. Adjust if using a different camera or port.

---

## Starting the MotionEye Container

Start the MotionEye container by running:

```bash
sudo docker-compose up -d
```

The container will download and set up MotionEye in detached mode, storing configurations in the `motioneye` folder.

---

## Accessing MotionEye

Once the container is running, access MotionEye by opening a browser and navigating to your Raspberry Pi’s IP address on port `8765` (e.g., `http://<Raspberry_Pi_IP>:8765`). 

The default login credentials are:
- **Username**: `admin`
- **Password**: (leave blank)

You can set an admin password in the settings after logging in.

---

## Adding Cameras to MotionEye

Once logged in, add a camera:
1. Click on **Add Camera** and choose the camera type (e.g., Local Camera for USB or Pi Camera).
2. Configure options like resolution, frame rate, and motion detection.

---

## Configuring Motion Detection and Alerts

1. **Motion Detection**: Enable motion detection in the settings for each camera, adjusting sensitivity as needed.
2. **Alerts**: Set up email or web hook notifications to receive alerts when motion is detected.

---

## Managing the MotionEye Container

### Stopping the MotionEye Container

To stop MotionEye, use:

```bash
sudo docker-compose down
```

This will stop and remove the container, keeping your configuration files in the `motioneye` folder.

### Restarting the Container

To start MotionEye again, simply run:

```bash
sudo docker-compose up -d
```

---

## Conclusion

Setting up MotionEye on your Raspberry Pi with Docker allows for easy, portable home surveillance. With MotionEye’s web-based interface, you can monitor cameras, record footage, and receive alerts, turning your Raspberry Pi into a reliable surveillance solution.

---
