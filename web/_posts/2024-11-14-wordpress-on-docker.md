---
title: Raspberry Pi WordPress Setup with Docker
description: >-
    Step-by-step guide to setting up WordPress on a Raspberry Pi using Docker.
excerpt:
    Learn how to turn your Raspberry Pi into a WordPress server using Docker, allowing for an easily manageable and portable WordPress installation.
layout: showcase
date: 2024-11-14
author: Kevin McAleer
difficulty: intermediate
cover: /assets/img/blog/wordpress-pi/cover.jpg
hero: /assets/img/blog/wordpress-pi/hero.png
mode: light
tags: 
 - raspberry_pi
 - wordpress
 - docker
groups:
 - raspberrypi
---

## What is WordPress?

`WordPress` (<https://www.wordpress.org>) is a powerful and popular content management system (CMS) used worldwide for creating websites and blogs. With a vast array of themes, plugins, and an easy-to-use interface, WordPress offers both beginner-friendly and advanced customization options for building robust sites.

---

## Why Use Docker on a Raspberry Pi?

Docker allows for isolated containers, making it easy to set up, maintain, and manage applications. By using Docker, we can:

1. **Simplify installation**: Docker bundles all dependencies.
2. **Run multiple services**: Run MySQL and WordPress as separate, connected containers.
3. **Reduce configuration time**: With pre-configured Docker images, setup is straightforward.

---

## Prerequisites

Before starting, you’ll need:

1. A Raspberry Pi (Raspberry Pi 3 or later is recommended).
2. Raspbian (or any Raspberry Pi OS variant) installed.
3. A reliable internet connection.
4. Docker and Docker Compose installed on your Raspberry Pi.

---

## Setting up Docker on the Raspberry Pi

### Step 1: Install Docker

If Docker is not installed, start by installing it. Run the following commands:

```bash
curl -sSL https://get.docker.com | sh
sudo usermod -aG docker $USER
```

After installation, reboot the Raspberry Pi:

```bash
sudo reboot
```

### Step 2: Install Docker Compose

Docker Compose will help manage multiple containers. Install it using:

```bash
sudo apt update
sudo apt install -y docker-compose
```

---

## Creating a Docker Compose File for WordPress

### Step 1: Set Up Your Working Directory

Navigate to your preferred directory and create a folder for your WordPress setup:

```bash
mkdir wordpress-docker && cd wordpress-docker
```

### Step 2: Create a `docker-compose.yml` File

In this folder, create a `docker-compose.yml` file:

```yaml
version: '3.3'

services:
  db:
    image: mysql:5.7
    volumes:
      - db_data:/var/lib/mysql
    restart: always
    environment:
      MYSQL_ROOT_PASSWORD: examplepassword
      MYSQL_DATABASE: wordpress
      MYSQL_USER: wordpressuser
      MYSQL_PASSWORD: wordpresspass

  wordpress:
    image: wordpress:latest
    ports:
      - "8080:80"
    restart: always
    environment:
      WORDPRESS_DB_HOST: db:3306
      WORDPRESS_DB_USER: wordpressuser
      WORDPRESS_DB_PASSWORD: wordpresspass
      WORDPRESS_DB_NAME: wordpress
    depends_on:
      - db

volumes:
  db_data:
```

Replace `examplepassword`, `wordpressuser`, and `wordpresspass` with your own secure values.

---

## Running the WordPress and MySQL Containers

With Docker Compose configured, start the containers by running:

```bash
sudo docker-compose up -d
```

This command tells Docker Compose to start both containers in detached mode.

---

## Accessing WordPress

Once the containers are running, access WordPress by entering your Raspberry Pi’s IP address and port 8080 in a browser (e.g., `http://<Raspberry_Pi_IP>:8080`). You should see the WordPress setup screen.

Follow the prompts to set up your new WordPress site.

---

## Managing Docker Containers

### Stopping the Containers

To stop the WordPress and MySQL containers, use:

```bash
sudo docker-compose down
```

This will stop and remove the containers, preserving the data in the `db_data` volume.

### Starting the Containers Again

When you’re ready to start them again, simply run:

```bash
sudo docker-compose up -d
```

---

## Conclusion

With Docker, hosting a WordPress site on your Raspberry Pi is quick and manageable. You can now enjoy a personal WordPress site on this portable, energy-efficient platform.

---
