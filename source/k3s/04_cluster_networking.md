---
title: Cluster Networking
description: Set up reliable networking for your Raspberry Pi K3s cluster, including static IPs, hostnames, and local DNS options.
layout: lesson
type: page
cover: assets/k3s03.jpg
date_updated: 2025-05-24
---

![Cover]({{page.cover}}){:class="cover"}

---

Before installing K3s, it’s important to ensure that all nodes can **communicate consistently** over the network.

In this lesson, you’ll configure **static IP addresses**, **hostnames**, and optionally set up a **local DNS or hosts file** for easier management.

---

## 🌐 Why Networking Matters in Kubernetes

K3s (and Kubernetes) rely heavily on:

- IP-based communication between control-plane and worker nodes
- Internal container networking (via CNI plugins)
- External access for load balancing and services

Without stable networking, your cluster will be unreliable or fail to form properly.

---

## ✅ Step 1: Assign Static IPs

### 🛠 Option A: DHCP Reservation (Recommended)

- Login to your home router or DHCP server
- Reserve a static IP for each Pi based on its **MAC address**

| Hostname    | MAC Address        | Static IP        |
|-------------|--------------------|------------------|
| pi-master   | `b8:27:eb:xx:xx:xx`| `192.168.1.100`  |
| pi-node1    | `b8:27:eb:xx:xx:xx`| `192.168.1.101`  |
| pi-node2    | `b8:27:eb:xx:xx:xx`| `192.168.1.102`  |
{:class="table table-striped"}

---

> 🧠 This is the easiest and most router-friendly method.

---

### 🛠 Option B: Static IP on the Device

If your router doesn’t support reservations, configure static IPs directly:

Edit netplan config (Ubuntu):

```bash
sudo nano /etc/netplan/00-installer-config.yaml
````

Example:

```yaml
network:
  ethernets:
    eth0:
      dhcp4: no
      addresses: [192.168.1.100/24]
      gateway4: 192.168.1.1
      nameservers:
        addresses: [1.1.1.1, 8.8.8.8]
  version: 2
```

Apply changes:

```bash
sudo netplan apply
```

---

## 🧩 Step 2: Set Consistent Hostnames

Each Pi should have a unique, meaningful hostname:

```bash
sudo hostnamectl set-hostname pi-master
```

Also update `/etc/hosts`:

```bash
127.0.0.1   localhost
127.0.1.1   pi-master
```

Repeat on all nodes (e.g., `pi-node1`, `pi-node2`).

---

## 🧰 Step 3: Test Network Connectivity

On each Pi, ping the others:

```bash
ping pi-node1.local
ping 192.168.1.101
```

If `.local` doesn’t work, use IPs or set up a shared `/etc/hosts` file on all nodes:

```plaintext
192.168.1.100 pi-master
192.168.1.101 pi-node1
192.168.1.102 pi-node2
```

---

## 🔧 Optional: Set Up Local DNS with Pi-hole or dnsmasq

If you're managing a home lab or multiple clusters, consider setting up **Pi-hole** or **dnsmasq** to provide:

- DNS-based hostnames (e.g. `pi-master.localdomain`)
- Automatic IP resolution
- Blocklists and ad filtering (bonus!)

---

## 🧪 Test Summary

You’re ready to move forward when:

- ✅ All nodes have **static IPs**
- ✅ Each node has a unique **hostname**
- ✅ You can **SSH and ping** all nodes from each other

---

Next up: [Installing K3s](05_installing_k3s)

---
