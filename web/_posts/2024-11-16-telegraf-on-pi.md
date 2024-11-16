---
title: Raspberry Pi Telegraf Setup with Docker
description: >-
    Step-by-step guide to setting up Telegraf on a Raspberry Pi using Docker.
excerpt:
    Learn how to set up Telegraf on your Raspberry Pi with Docker to monitor system metrics and integrate with popular time-series databases like InfluxDB or Prometheus.
layout: showcase
date: 2024-11-14
author: Kevin McAleer
difficulty: intermediate
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
 - raspberrypi
---

## What is Telegraf?

Telegraf is a lightweight agent for collecting, processing, and reporting metrics and events from systems, sensors, and applications. It supports numerous input and output plugins, making it ideal for monitoring Raspberry Pi metrics and integrating with databases like InfluxDB.

---

## Why Use Docker for Telegraf?

Using Docker simplifies the setup process and provides:

1. **Portability**: Easily replicate your Telegraf configuration on other devices.
1. **Isolation**: Run Telegraf independently from the host system.
1. **Ease of Maintenance**: Manage Telegraf with minimal system changes.

---

## Prerequisites

To set up Telegraf on your Raspberry Pi, you’ll need:

1. A Raspberry Pi running Raspberry Pi OS.
1. Docker and Docker Compose installed.
1. An InfluxDB, Prometheus, or other output plugin endpoint for metrics (optional but recommended).

---

## Installing Docker on the Raspberry Pi

### Step 1: Install Docker

If Docker is not installed, you can install it with:

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

### Step 2: Create a Configuration File

Download a sample Telegraf configuration file:

```bash
curl -o telegraf.conf https://raw.githubusercontent.com/influxdata/telegraf/master/etc/telegraf.conf
```

Edit the configuration file to set up inputs and outputs. For example, to monitor system metrics:

```bash
nano telegraf.conf
```

Uncomment the `[inputs.system]` plugin and configure the `[outputs.influxdb]` or another output plugin to send metrics.

---

Here is an exaple of a `telegraf.conf` file with just the bare bones:

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

### Step 3: Create a `docker-compose.yml` File

In the same directory, create a `docker-compose.yml` file:

```yaml
services:
  telegraf:
    image: telegraf
    volumes:
      - ./telegraf.conf:/etc/telegraf/telegraf.conf
      - /var/run/docker.sock:/var/run/docker.sock
    restart: always
    networks:
      - monitoring_net
    hostname: "${HOSTNAME}"
    environment:
      - HOSTNAME=${HOSTNAME}
    privileged: true
networks:
  monitoring_net:
    driver: bridge
```

The `network_mode: "host"` is necessary for Telegraf to access system-level metrics on the Raspberry Pi.

---

### Step 4: Configure the `HOSTNAME` Environment Variable

To automatically set the `HOSTNAME` environment variable, add the following line to your /etc/profile` file:

```bash
export HOSTNAME=$(hostname)
```

---

### Step 5: Grant Docker Access to the `docker.sock` File

To allow Telegraf to access Docker metrics, add the `telegraf` user to the `docker` group:

```bash
sudo chmod 666 /var/run/docker.sock
```

---

## Starting the Telegraf Container

Start the Telegraf container using:

```bash
sudo docker-compose up -d
```

The container will start collecting metrics based on your configuration file.

---

## Verifying Telegraf is Running

Check the logs to verify Telegraf is collecting metrics:

```bash
sudo docker logs telegraf
```

You should see logs indicating successful metric collection and delivery to the configured output.

---

## Integrating with InfluxDB

If you’re using InfluxDB as the output, ensure it’s set up and accessible. For example, if running InfluxDB locally on Docker:

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

## Managing the Telegraf Container

### Stopping the Container

To stop the Telegraf container, run:

```bash
sudo docker-compose down
```

### Restarting the Container

To start Telegraf again:

```bash
sudo docker-compose up -d
```

---

## Setting up Grafana Container

To visualize the metrics collected by Telegraf, you can set up Grafana in a similar manner. Here is an example `docker-compose.yml` file for Grafana:

```yaml
version: "3.9"
services:
  grafana:
    image: "grafana/grafana:latest"
    restart: unless-stopped
    user: "0"
    volumes:
      - "${DATADIR}/grafana/data:/var/lib/grafana"
    ports:
      - 3000:3000
    environment:
      - GF_AUTH_DISABLE_LOGIN_FORM=true
      - GF_AUTH_ANONYMOUS_ENABLED=true
      - GF_AUTH_ANONYMOUS_ORG_ROLE=Admin
      - GF_SECURITY_ALLOW_EMBEDDING=true
```

To start Grafana, run:

```bash
sudo docker-compose up -d
```

---

## Logging into Grafana

Access Grafana at `http://<raspberrypi-ip>:3000` and log in with the default credentials (admin/admin). You can then set up dashboards to visualize the metrics collected by Telegraf.

Create a new Dashboard, add a new Panel, and select the data source as InfluxDB. 

### Adding a Variable

To add a variable, click on the gear icon in the top right corner, select `Variables`, and click `Add Variable`.

Create a variable with the following settings:

- Name: `servername`
- Query: `SHOW TAG VALUES WITH KEY = "host"`

Click the Multi-value checkbox and Save.

## Add a Row

To add a row to the dashboard, click the `+` icon in the top right corner and select `Add Row`.

Add a Panel to the row and select the data source as InfluxDB.

Create a query with the following settings:

- Measurement: `cpu`
- Field: `usage_idle`
- Group by: `time($__interval), "host"`
- Where: `"host" =~ /^$servername$/`
- Select: `mean()`

This will add a new panel to the dashboard showing the CPU usage for the selected server. Move this to the new row.

Select all the servers you've installed Telegraf on to see the CPU usage for each server.

Repeat for all other useful metrics you want to monitor.

---

## Conclusion

Using Docker, setting up Telegraf on a Raspberry Pi is straightforward. Telegraf provides a powerful and extensible solution for monitoring and integrating metrics from your Raspberry Pi into your preferred monitoring system.

---
