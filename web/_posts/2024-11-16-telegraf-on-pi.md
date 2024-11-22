---
title: Raspberry Pi Telegraf Setup with Docker
description: "Step-by-step guide to setting up Telegraf on a Raspberry Pi using Docker."
excerpt: >- 
  "Learn how to set up Telegraf on your Raspberry Pi with Docker to monitor system metrics and integrate with popular time-series databases like InfluxDB or Prometheus."
author: Kevin McAleer  
date: 2024-11-16
date_updated: 2024-11-22
layout: showcase
difficulty: Intermediate 
cover: /assets/img/blog/telegraf-pi/cover.jpg
hero: /assets/img/blog/telegraf-pi/hero.png 
mode: light
tags: 
 - Raspberry Pi
 - Telegraf
 - Docker
 - Monitoring
 - Metrics  
groups: 
- Raspberry Pi  
---

## What is Telegraf?

Telegraf is a lightweight agent for collecting, processing, and reporting metrics and events from systems, sensors, and applications. With numerous input and output plugins, it’s perfect for monitoring Raspberry Pi metrics and integrating with databases like InfluxDB.

---

## Why Use Docker for Telegraf?

Using Docker simplifies the setup process and provides:
1. **Portability**: Easily replicate your Telegraf setup on other devices.
2. **Isolation**: Run Telegraf independently of the host system.
3. **Ease of Maintenance**: Manage Telegraf with minimal system changes.

---

## Prerequisites

To set up Telegraf on your Raspberry Pi, ensure you have:
- A Raspberry Pi running Raspberry Pi OS.
- Docker and Docker Compose installed.
- An output plugin endpoint like InfluxDB or Prometheus for metrics (optional but recommended).

---

## Installing Docker on the Raspberry Pi

### Step 1: Install Docker
If Docker is not installed, run the following commands:

```bash
curl -sSL https://get.docker.com | sh
sudo usermod -aG docker $USER
```

Reboot your Raspberry Pi to finalize the installation:

```bash
sudo reboot
```

### Step 2: Install Docker Compose
To manage Telegraf with Docker Compose, install Docker Compose:

```bash
sudo apt update
sudo apt install -y docker-compose
```

---

## Setting Up Telegraf with Docker Compose

### Step 1: Create a Working Directory
Create a folder for Telegraf and navigate into it:

```bash
mkdir telegraf-docker && cd telegraf-docker
```

---

### Step 2: Create a `docker-compose.yml` File
In the working directory, create a `docker-compose.yml` file:

```yaml
services:
  telegraf:
    image: telegraf
    volumes:
      - ./telegraf.conf:/etc/telegraf/telegraf.conf
      - /var/run/docker.sock:/var/run/docker.sock
    restart: always
    hostname: "${HOSTNAME}"
    environment:
      - HOSTNAME=${HOSTNAME}
    privileged: true
networks:
  default:
    driver: bridge
```

This configuration mounts a `telegraf.conf` file and the Docker socket inside the container.  

---

### Step 3: Start the Container Temporarily
Run the following command to start the container:

```bash
docker-compose up -d
```

This starts a temporary Telegraf container without a configuration file. We’ll generate the configuration file next.

---

### Step 4: Generate the Configuration File
Run this command to generate the default configuration file into the current directory:

```bash
docker exec telegraf telegraf config > telegraf.conf
```

- **`docker exec`** runs the `telegraf config` command inside the container.
- **`>`** redirects the output to a `telegraf.conf` file on the host.

---

### Step 5: Edit the Configuration File
Customize the `telegraf.conf` file to suit your needs. For example:

```bash
nano telegraf.conf
```

Here’s a simple example configuration file:

```conf
[[inputs.cpu]]
  percpu = false

[[inputs.disk]]

[[inputs.mem]]

[[inputs.system]]

[[outputs.influxdb]]
  urls = ["http://192.168.1.10:8086"] # Replace with your InfluxDB endpoint
  database = "telegraf"

[agent]
  hostname = "${HOSTNAME}"

[[inputs.docker]]
  endpoint = "unix:///var/run/docker.sock"
```

---

### Step 6: Restart the Container with the New Configuration
Stop the container and restart it to apply the updated configuration:

```bash
docker-compose down
docker-compose up -d
```

The container now uses the updated `telegraf.conf` file.

---

## Verifying Telegraf is Running

Check the logs to ensure Telegraf is running correctly:

```bash
docker logs telegraf
```

You should see logs indicating successful metric collection and delivery to the configured output.

---

## Integrating with InfluxDB

If you’re using InfluxDB as the output, ensure it’s set up and accessible. Here’s an example `docker-compose.yml` configuration for InfluxDB:

```yaml
services:
  influxdb:
    image: influxdb:latest
    container_name: influxdb
    ports:
      - "8086:8086"
    volumes:
      - influxdb_data:/var/lib/influxdb
    restart: unless-stopped

volumes:
  influxdb_data:
```

Set the `[outputs.influxdb]` section in `telegraf.conf` to match your InfluxDB instance.

---

## Adding Grafana for Visualization

To visualize the metrics collected by Telegraf, set up Grafana using the following `docker-compose.yml` file:

```yaml
services:
  grafana:
    image: "grafana/grafana:latest"
    restart: unless-stopped
    user: "0"
    volumes:
      - ./grafana_data:/var/lib/grafana
    ports:
      - "3000:3000"
    environment:
      - GF_AUTH_DISABLE_LOGIN_FORM=true
      - GF_AUTH_ANONYMOUS_ENABLED=true
      - GF_AUTH_ANONYMOUS_ORG_ROLE=Admin
```

Start Grafana with:

```bash
docker-compose up -d
```

Access Grafana at `http://<raspberrypi-ip>:3000` and log in with the default credentials (`admin/admin`).

---

## Conclusion

Using Docker, setting up Telegraf on a Raspberry Pi is straightforward. By leveraging Docker Compose, you can easily manage the configuration file, update plugins, and integrate with tools like InfluxDB and Grafana for comprehensive monitoring and visualization.

---
