---
layout: lesson
title: Adding Nodes
author: Kevin McAleer
type: page
cover: assets/k3s05.jpg
date: 2025-06-01
previous: 05_installing_k3s.html
next: 07_troubleshooting_install.html
description: Join additional Raspberry Pi nodes to your K3s cluster using the shared
  token and master node address.
percent: 30
duration: 2
date_updated: 2025-05-24
navigation:
- name: Running K3s on Raspberry Pi
- content:
  - section: Introduction
    content:
    - name: Introduction to K3s on Raspberry Pi
      link: 01_intro.html
  - section: Preparing Your Raspberry Pi Cluster
    content:
    - name: Hardware Requirements
      link: 02_hardware_requirements.html
    - name: Installing the Operating System
      link: 03_os_installation.html
    - name: Cluster Networking
      link: 04_cluster_networking.html
  - section: Installing K3s
    content:
    - name: Installing K3s
      link: 05_installing_k3s.html
    - name: Adding Nodes
      link: 06_adding_nodes.html
    - name: Troubleshooting Installation
      link: 07_troubleshooting_install.html
  - section: Using Your K3s Cluster
    content:
    - name: Kubectl Basics
      link: 08_kubectl_basics.html
    - name: Dashboard and Monitoring
      link: 09_dashboard_and_monitoring.html
    - name: Storage and Volumes
      link: 10_storage_and_volumes.html
    - name: Ingress and Services
      link: 11_ingress_and_services.html
  - section: Real-World Deployments
    content:
    - name: Deploying Apps
      link: 12_deploying_apps.html
    - name: Using Helm
      link: 13_using_helm.html
    - name: CI/CD on K3s
      link: 14_ci_cd_on_k3s.html
  - section: Advanced Topics
    content:
    - name: Advanced Networking & Custom CNI
      link: 15_custom_cni.html
    - name: Securing Your Cluster
      link: 16_securing_your_cluster.html
  - section: Final Project
    content:
    - name: "Final Project \u2013 Deploying a Multi-Service App"
      link: 17_final_project.html
  - section: Summary
    content:
    - name: Course Summary
      link: 18_summary.html
---


![Cover]({{page.cover}}){:class="cover"}

---

With your **K3s control plane (master)** running, it’s time to expand your cluster by **adding worker nodes**.

In this lesson, you’ll SSH into each worker Pi and join it to the K3s cluster using a one-liner join command.

---

## 🧠 How It Works

Worker nodes use:

- The **master node’s IP address**
- A shared **cluster token**
- The `k3s agent` service to register and run workloads

---

## 🔑 Step 1: Copy Your Cluster Token

On your **master node**, run:

```bash
sudo cat /var/lib/rancher/k3s/server/node-token
````

It will look something like:

```text
K108f02c5486e1bdf7a9a7f25a2e7aa3c2c74fa67cced8b31332f5c1b715c962c::server:5bc9c3f0d9249...
```

Copy this token to use on all worker nodes.

---

## 🔧 Step 2: Install K3s Agent on Each Worker Node

On each worker Pi:

1. SSH into the node:

   ```bash
   ssh pi@pi-node1.local
   ```

2. Run the installer with the following environment variables:

   ```bash
   curl -sfL https://get.k3s.io | K3S_URL=https://<MASTER-IP>:6443 K3S_TOKEN=<YOUR_TOKEN> sh -
   ```

   Replace `<MASTER-IP>` and `<YOUR_TOKEN>` with:

   - The IP address of your master node (e.g. `192.168.1.100`)
   - The token you copied earlier

> ✅ Example:
>
> ```bash
> curl -sfL https://get.k3s.io | K3S_URL=https://192.168.1.100:6443 K3S_TOKEN=K108f0... sh -
> ```

---

## 🧪 Step 3: Verify Node Joined the Cluster

Back on the master node:

```bash
kubectl get nodes
```

You should now see your worker(s) listed:

```plaintext
NAME        STATUS   ROLES                  AGE     VERSION
pi-master   Ready    control-plane,master   10m     v1.29.2+k3s1
pi-node1    Ready    <none>                 1m      v1.29.2+k3s1
pi-node2    Ready    <none>                 1m      v1.29.2+k3s1
```

---

## 🛠 Additional Notes

- You can repeat this process for as many worker nodes as you want.
- The install script automatically sets up and starts the `k3s-agent` systemd service.
- Worker nodes will begin running pods as scheduled by the control plane.

---

## 🔍 Troubleshooting

| Issue                     | Fix                                                              |
| ------------------------- | ---------------------------------------------------------------- |
| Node doesn't show up      | Check firewall/ports (`6443` must be open)                       |
| Wrong IP in `K3S_URL`     | Use master’s static IP, not hostname                             |
| Token rejected            | Double-check the token or regenerate with `--token` on reinstall |
| `k3s-agent` service fails | Check logs with `journalctl -u k3s-agent -e`                     |
{:class="table table-striped"}

---

🎉 Your K3s cluster is now multi-node and ready to deploy workloads!

Next up: [Troubleshooting Installation](07_troubleshooting_install)

---
