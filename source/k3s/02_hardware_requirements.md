---
title: Hardware Requirements
description: Choose the right Raspberry Pi hardware and accessories to build a reliable and scalable K3s Kubernetes cluster.
layout: lesson
type: page
cover: /learn/k3s/assets/banners/02_hardware_requirements.jpg
date_updated: 2025-05-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Before building your Kubernetes cluster, let’s go over the **hardware requirements** for running K3s effectively on Raspberry Pi.

You don’t need a datacenter — just a few small boards and some good planning.

---

## Minimum Recommended Setup

| Component                 | Description                              |
|---------------------------|------------------------------------------|
| Raspberry Pi 4 or 5       | 2GB RAM minimum (4GB+ preferred)         |
| SD Card or SSD            | 32GB+ (SSD/NVMe preferred for longevity) |
| Ethernet & Network Switch | For node-to-node communication           |
| Power Supply              | Official USB-C power supply per Pi       |
| Case & Cooling            | Optional but recommended                 |
{:class="table table-striped"}

---

> Avoid using Wi-Fi for clustering — use **wired Ethernet** for best stability and performance.

---

## Cluster Configurations

### Basic 2-node Dev Cluster

- 1x Pi as **master/control-plane**
- 1x Pi as **worker node**

Great for testing, prototyping, and local workloads.

### 3–4 Node Homelab Cluster

- 1x Pi as master
- 2–3x Pi as workers
- Optional external NFS or SSD storage

Perfect for real-world deployments, CI/CD testing, and microservices projects.

---

## Storage Options

| Option        | Notes |
|---------------|-------|
| MicroSD Card  | Cheap but wears out over time |
| USB SSD       | Fast, reliable, preferred for write-heavy workloads |
| NFS/SMB Share | For shared persistent volumes (optional but useful) |
{:class="table table-striped"}

---

> K3s includes **local-path-provisioner** for simple storage, but you can configure NFS or longhorn later for better performance.

---

## Networking Checklist

- All Pis must be on the **same subnet**
- Each Pi should have a **static IP** or DHCP reservation
- Use a basic unmanaged switch or your home router
- Optional: connect Pis via USB-C Ethernet adapter or mesh Hat

---

## Optional (But Helpful)

- External USB SSDs or NVMe adapters
- UPS (uninterruptible power supply)
- Pi racks or stackable cases with fans
- Label maker or naming stickers for nodes

---

## Example BOM (Bill of Materials)

| Item                    | Qty | Link Example |
|-------------------------|-----|---------------|
| Raspberry Pi 5 (4GB)    | 3   | [Pi Shop](https://www.raspberrypi.com) |
| 32GB+ microSD card      | 3   | SanDisk Ultra or Samsung Evo |
| Official power supplies | 3   | USB-C 5V/3A |
| Gigabit network switch  | 1   | TP-Link 5 or 8 port |
| Ethernet patch cables   | 3–5 | Any Cat5e+ |
{:class="table table-striped"}

---

## Next Step

Once you’ve gathered your hardware, the next step is to install the OS and prepare the nodes for Kubernetes.

Next up: [Installing the Operating System](03_os_installation)

---
