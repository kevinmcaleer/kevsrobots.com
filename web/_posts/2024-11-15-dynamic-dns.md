---
title: Setting Up Dynamic DNS on a Raspberry Pi for Self-Hosting
description: >-
    A step-by-step guide to setting up Dynamic DNS on a Raspberry Pi for self-hosting services like WordPress, Ghost, or an Nginx server.
excerpt:
    Learn how to configure Dynamic DNS on your Raspberry Pi to enable easy remote access and self-host your WordPress, Ghost blog, or other web services.
layout: showcase
date: 2024-11-15
author: Kevin McAleer
difficulty: intermediate
cover: /assets/img/blog/ddns-pi/cover.jpg
hero: /assets/img/blog/ddns-pi/hero.png
mode: light
tags: 
 - Raspberry Pi
 - Dynamic DNS
 - Self-Hosting
 - WordPress
 - Ghost
 - Nginx
groups:
 - raspberrypi
---

## What is Dynamic DNS?

Dynamic DNS (DDNS) is a service that maps your dynamic public IP address to a domain name. This is useful when hosting services like WordPress, Ghost, or an Nginx server at home, as it allows you to access your Raspberry Pi using a consistent domain name, even if your ISP changes your IP address.

---

## Why Use Dynamic DNS for Self-Hosting?

1. **Consistent Access**: A DDNS service keeps your domain name updated with your public IP address.
2. **Ease of Use**: No need to remember changing IP addresses.
3. **Remote Access**: Makes self-hosted services accessible from anywhere.

---

## Prerequisites

1. A Raspberry Pi running Raspberry Pi OS.
2. A registered account with a DDNS provider (e.g., No-IP, DuckDNS, DynDNS).
3. Your Raspberry Pi must be accessible from the internet via port forwarding.
4. A domain name (if required by your DDNS provider).

---

## Setting Up a DDNS Client on Raspberry Pi

### Step 1: Choose a DDNS Provider

Popular free and paid DDNS providers include:
- [No-IP](https://www.noip.com/)
- [DuckDNS](https://www.duckdns.org/)
- [DynDNS](https://dyn.com/dns/)

Create an account with your preferred provider and configure a hostname. For example, `myhome.ddns.net`.

---

### Step 2: Install a DDNS Client

Depending on your chosen provider, you’ll need to install a DDNS client. For this guide, we’ll cover No-IP and DuckDNS.

---

### Option 1: Setting Up No-IP

1. **Install the No-IP Client**:

   ```bash
   sudo apt update
   sudo apt install noip2
   ```

2. **Configure No-IP**:

   Run the following command and enter your No-IP account details when prompted:

   ```bash
   sudo noip2 -C
   ```

3. **Start the No-IP Client**:

   After configuring, start the client with:

   ```bash
   sudo noip2
   ```

4. **Check the Status**:

   Verify that the No-IP client is running:

   ```bash
   sudo noip2 -S
   ```

---

### Option 2: Setting Up DuckDNS

1. **Install DuckDNS Client**:

   DuckDNS doesn’t require a specific client. Instead, use a script. Navigate to your home directory:

   ```bash
   cd ~
   ```

2. **Download the DuckDNS Script**:

   Create a script file:

   ```bash
   nano duckdns.sh
   ```

   Add the following content, replacing `<TOKEN>` and `<DOMAIN>` with your DuckDNS token and subdomain:

   ```bash
   echo url="https://www.duckdns.org/update?domains=<DOMAIN>&token=<TOKEN>&ip=" | curl -k -o ~/duckdns.log -K -
   ```

3. **Make the Script Executable**:

   ```bash
   chmod +x duckdns.sh
   ```

4. **Automate Updates with Cron**:

   Open the crontab editor:

   ```bash
   crontab -e
   ```

   Add the following line to run the script every 5 minutes:

   ```bash
   */5 * * * * ~/duckdns.sh
   ```

---

## Configuring Your Router

### Step 1: Enable Port Forwarding

Log in to your router’s admin interface and forward the following ports to your Raspberry Pi’s local IP:
- **Port 80** (HTTP) for web servers like WordPress or Ghost.
- **Port 443** (HTTPS) for secure web access.
- Any other ports required by your application.

---

### Step 2: Test Access

Use your DDNS domain (e.g., `http://myhome.ddns.net`) to access your Raspberry Pi from an external network. If configured correctly, your DDNS domain should point to your Raspberry Pi’s public IP address.

---

## Additional Security: SSL/TLS Certificates with Let's Encrypt

1. **Install Certbot**:

   Install Certbot to obtain free SSL certificates:

   ```bash
   sudo apt update
   sudo apt install certbot python3-certbot-nginx
   ```

2. **Obtain a Certificate**:

   Run the following command, replacing `<DOMAIN>` with your DDNS domain:

   ```bash
   sudo certbot --nginx -d <DOMAIN>
   ```

3. **Renew Certificates Automatically**:

   Add a cron job to renew the certificates automatically:

   ```bash
   crontab -e
   ```

   Add this line:

   ```bash
   0 3 * * * certbot renew --quiet
   ```

---

## Conclusion

Setting up Dynamic DNS on your Raspberry Pi allows you to self-host services like WordPress, Ghost, or an Nginx server with a consistent domain name. With DDNS and port forwarding, your Raspberry Pi becomes a powerful tool for hosting accessible, secure applications.

---