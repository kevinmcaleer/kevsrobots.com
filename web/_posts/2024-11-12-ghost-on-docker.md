---
title: Raspberry Pi Ghost Setup with Docker
description: >-
    Guide to setting up a Ghost blog on a Raspberry Pi using Docker.
excerpt:
    Transform your Raspberry Pi into a Ghost blogging platform using Docker. This guide covers the setup process and configuration for a personal Ghost site.
layout: showcase
date: 2024-11-14
author: Kevin McAleer
difficulty: intermediate
cover: /assets/img/blog/ghost-pi/cover.jpg
hero: /assets/img/blog/ghost-pi/hero.png
mode: light
tags: 
 - raspberry_pi
 - ghost
 - docker
 - web_hosting
groups:
 - raspberrypi
---

## What is Ghost?

Ghost is a modern, open-source platform for creating and managing a blog or publication. Known for its minimal design and powerful editing features, Ghost offers a smooth, fast writing experience. It’s an excellent choice for those looking to set up a clean and performant blog.

---

## Why Use Docker on a Raspberry Pi?

Using Docker simplifies application management and provides:
1. **Quick setup**: Easily deploy Ghost and its dependencies.
2. **Modularity**: Run Ghost and its database in separate, managed containers.
3. **Portability**: Move and scale the setup without reconfiguring everything.

---

## Prerequisites

To set up Ghost, you'll need:
1. A Raspberry Pi (preferably Raspberry Pi 4).
2. Raspbian OS (or a variant of Raspberry Pi OS).
3. An internet connection.
4. Docker and Docker Compose installed on the Raspberry Pi.

---

## Installing Docker on the Raspberry Pi

### Step 1: Install Docker

If you haven’t installed Docker yet, do so with the following command:

```bash
curl -sSL https://get.docker.com | sh
sudo usermod -aG docker $USER
```

Reboot the Raspberry Pi to complete the installation:

```bash
sudo reboot
```

### Step 2: Install Docker Compose

Docker Compose is needed to run Ghost with its database. Install it using:

```bash
sudo apt update
sudo apt install -y docker-compose
```

---

## Creating a Docker Compose File for Ghost

### Step 1: Set Up a Working Directory

Navigate to a preferred directory and create a new folder for Ghost:

```bash
mkdir ghost-docker && cd ghost-docker
```

### Step 2: Create a `docker-compose.yml` File

Create a `docker-compose.yml` file in this folder with the following content:

```yaml
version: '3.3'

services:
  ghost:
    image: ghost:latest
    ports:
      - "2368:2368"
    environment:
      database__client: sqlite3
      database__connection__filename: "/var/lib/ghost/content/data/ghost.db"
      url: http://<Raspberry_Pi_IP>:2368
    volumes:
      - ghost_data:/var/lib/ghost/content
    restart: always

volumes:
  ghost_data:
```

Replace `<Raspberry_Pi_IP>` with your Raspberry Pi’s IP address.

---

## Starting the Ghost Container

To launch the Ghost blog, run:

```bash
sudo docker-compose up -d
```

This will download the Ghost image and start the container in detached mode.

---

## Accessing Your Ghost Blog

Once the container is up, access your blog by entering your Raspberry Pi’s IP and port 2368 in a browser, like `http://<Raspberry_Pi_IP>:2368`. You’ll be directed to the Ghost setup page.

Follow the instructions to configure your Ghost blog.

---

## Managing Docker Containers

### Stopping the Ghost Container

To stop the Ghost container, use:

```bash
sudo docker-compose down
```

This will stop and remove the container, while preserving data in the `ghost_data` volume.

### Starting the Container Again

To start Ghost again, run:

```bash
sudo docker-compose up -d
```

---

## Conclusion

With Docker, hosting Ghost on a Raspberry Pi becomes a straightforward task. You can now run a lightweight, efficient blog on this small but powerful platform.
