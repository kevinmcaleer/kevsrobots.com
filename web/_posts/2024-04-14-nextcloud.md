---
title: NextCloud
description: >-
    Share files online from your Pi with NextCloud
layout: showcase
date: 2024-04-14
author: Kevin McAleer
difficulty: intermediate
excerpt: >-
    You can host your own NextCloud server on your Raspberry Pi, and share files online with your friends and family.
cover: /assets/img/blog/nextcloud/cover.png
hero: /assets/img/blog/nextcloud/hero.png
mode: light
tags:
  - raspberry_pi
groups:
  - raspberrypi
videos:
  - ewgBgbqGt0k

code:
 - https://www.github.com/kevinmcaleer/ClusteredPi

---

## What is NextCloud?

NextCloud is a suite of client-server software for creating and using file hosting services. It is functionally similar to Dropbox, although NextCloud is free and open-source, allowing anyone to install and operate it on a private server.

You can host your own NextCloud server on your Raspberry Pi, and share files online with your friends and family.

There are even Cloud Apps for NextCloud so you can upload and edit Microsoft Office documents in your browser, and even use it as a calendar and contact manager.

---

## What you will need

- Raspberry Pi 5 is best, but a 4 will do.
- A microSD card with Raspberry Pi OS installed.
- A power supply for your Pi - it will always need to be on if you want 24/7 access to your files.
- An internet connection.
- A USB hard drive or [NVMe SSD](/blog/build-a-home-server) to store your files.
- [Docker](/learn/docker) installed; this will run NextCloud in a container.

---

## Setting up NextCloud

1. **Install Docker**: If you haven't already, install Docker on your Raspberry Pi. You can follow the instructions in my [Docker guide](/learn/docker).
1. **Clone my easy to use Docker template**: I've created a simple Docker template to get you up and running with NextCloud in minutes. You can find it on my [GitHub](https://www.github.com/kevinmcaleer/ClusteredPi). To clone the repository, run the following command from the Raspberry Pi terminal:

    ```bash
    git clone https://www.github.com/kevinmcaleer/ClusteredPi
    ```
1. **Navigate to the `ClusteredPi/stacks/nextcloud` directory**:
    
        ```bash
        cd ClusteredPi/stacks/nextcloud
        ```
1. **Run the Docker Compose file**:  and run the following command:

    ```bash
    docker-compose up -d
    ```

NextCloud will install and run in a Docker container on your Raspberry Pi. You can access it by navigating to `http://<your-pi-ip>:8080` in your web browser.

---

## Login to NextCloud

When you navigate to your Pi's IP address in your web browser (or browse to `http://localhost:8080`), you will be greeted with the NextCloud login screen. You can create an account and start uploading files to your Pi.

---

Here is a detailed guide on how to expose a Nextcloud instance running on a local server to the Internet using Cloudflare. This guide will include using a fixed IP address and an alternative method using Dynamic DNS (DDNS) if a static IP is not available.

---

## Expose Your Local Nextcloud Server to the Internet Using Cloudflare

This guide covers how to expose your Nextcloud instance, which is running on a local server, to the Internet through Cloudflare. We'll cover two approaches: using a static IP address and using Dynamic DNS.

---

### Prerequisites

- **Nextcloud setup**: Your Nextcloud server should be fully installed and running on a local network.
- **Domain Name**: You need a registered domain name.
- **Cloudflare Account**: Sign up for a free Cloudflare account at [Cloudflare](https://www.cloudflare.com/).

---

### Method 1: Using a Static IP Address

If you have a static IP address provided by your ISP, follow these steps:

#### Step 1: Configure Your Router

1. **Assign a Static IP to Your Server**: Ensure your local server has a static IP address on your local network.
2. **Port Forwarding**:
    - Login to your router’s configuration page.
    - Navigate to the Port Forwarding section.
    - Forward TCP ports 80 and 443 to the static IP of your server.

#### Step 2: Add Your Domain to Cloudflare

1. **Login to Cloudflare** and select 'Add Site' to add your domain.
2. **Update Nameservers**: Change your domain's nameservers to the ones provided by Cloudflare. This change is made at your domain registrar's website.

#### Step 3: Create DNS Records

1. **Create A Record**: In the Cloudflare dashboard, go to the DNS settings page.
2. Add an A record pointing to your public static IP address:
    - Type: `A`
    - Name: `@` (or you can use a subdomain like `cloud`)
    - IPv4 address: `Your public static IP`
    - TTL: `Auto`
    - Proxy status: `Proxied`

#### Step 4: Ensure SSL/TLS Configuration

1. **Navigate to SSL/TLS tab** on Cloudflare and set the SSL/TLS encryption mode to `Full (strict)`.

#### Step 5: Access Your Nextcloud

- Now, you can access your Nextcloud installation via your domain (e.g., `https://yourdomain.com` or `https://cloud.yourdomain.com`).

---

### Method 2: Using Dynamic DNS

If you don't have a static IP address, you can use Dynamic DNS:

#### Step 1: Choose a DDNS Provider

- Use a service like No-IP, DynDNS, or DuckDNS to create a DDNS hostname that updates with your changing IP.

#### Step 2: Configure DDNS on Your Router

- **Setup DDNS**: Most routers support DDNS. Enter your DDNS details in the router's DDNS settings.
- **Port Forwarding**: Similar to above, forward TCP ports 80 and 443 to your server’s local IP.

#### Step 3: Add Your Domain to Cloudflare

- Follow the same steps as in Method 1 to add and verify your domain with Cloudflare.

#### Step 4: Create DNS Records

- Instead of an A record, you will set up a CNAME record on Cloudflare:
    - Type: `CNAME`
    - Name: `@` or `cloud`
    - Target: `Your DDNS hostname`
    - TTL: `Auto`
    - Proxy status: `Proxied`

#### Step 5: Ensure SSL/TLS Configuration

- Set SSL/TLS encryption mode to `Full (strict)` as in Method 1.

#### Step 6: Access Your Nextcloud

- Access your Nextcloud via your domain linked to the DDNS hostname.

---

By following these steps, you can successfully expose your Nextcloud server to the Internet using Cloudflare, either with a static IP address or a DDNS service.

---

## Installing NextCloud Office Apps

NextCloud has a suite of office apps that you can install to edit and create documents online. Here's how to install them:

1. **Login to your NextCloud instance**.
1. **Click on your profile icon** in the top right corner.
1. **Select `Apps`** from the dropdown menu.
1. **Search for `Office`** in the search bar.
1. **Click on `Office`**.
1. **Click `Download and enable`**.
1. **Click `Enable`**.
1. **Repeat the process for `Calendar` and `Contacts`**.

You can now create and edit documents, spreadsheets, and presentations online using NextCloud.

---
