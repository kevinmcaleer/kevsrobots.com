---
layout: lesson
title: Troubleshooting Installation
author: Kevin McAleer
type: page
cover: assets/k3s06.jpg
date: 2025-06-01
previous: 06_adding_nodes.html
next: 08_kubectl_basics.html
description: Solve common problems when installing or joining K3s on Raspberry Pi,
  including networking, tokens, and service errors.
percent: 35
duration: 3
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

Even though K3s simplifies Kubernetes installation, things can still go wrong — especially with Raspberry Pis.

This lesson walks you through **common issues** and how to troubleshoot them effectively.

---

## 🧪 Problem: Node Doesn’t Join Cluster

### ❌ Symptom:

Worker node installs, but doesn't show up with `kubectl get nodes`.

### ✅ Solution:

- Ensure you're using the correct master IP:

  ```bash
  ping <MASTER-IP>
  ```

- Verify port 6443 is open and not firewalled.
- Check the K3s agent logs on the worker:

  ```bash
  journalctl -u k3s-agent -e
  ```

---

## 🔐 Problem: Token Rejected

### ❌ Symptom:

Worker node fails to join with error about invalid token.

### ✅ Solution:

- Re-check token syntax:

  ```bash
  sudo cat /var/lib/rancher/k3s/server/node-token
  ```

- Ensure no extra characters or missing colons.
- If needed, regenerate token by reinstalling K3s on master with:

  ```bash
  /usr/local/bin/k3s-uninstall.sh
  ```

---

## 🛑 Problem: k3s.service or k3s-agent.service Won’t Start

### ❌ Symptom:

K3s or agent fails to start on boot.

### ✅ Solution:

Check logs:

```bash
journalctl -u k3s -e
journalctl -u k3s-agent -e
```

Common fixes:

- Ensure `cgroup` support is enabled:
  Check `/boot/cmdline.txt` or `/boot/firmware/cmdline.txt` has:

  ```text
  cgroup_memory=1 cgroup_enable=memory cgroup_enable=cpuset
  ```

- Check for disk space:

  ```bash
  df -h
  ```

- Restart the service:

  ```bash
  sudo systemctl restart k3s
  ```

---

## 🌐 Problem: Hostname/IP Confusion

### ❌ Symptom:

Worker tries to join but fails due to DNS or unreachable host.

### ✅ Solution:

- Always use the master’s **IP address**, not its hostname (unless you have proper DNS).
- Confirm static IPs are in place.
- Optionally set `/etc/hosts` on each node.

---

## 🧹 Problem: You Want to Reinstall from Scratch

### ✅ Solution:

**On master:**

```bash
sudo /usr/local/bin/k3s-uninstall.sh
```

**On worker:**

```bash
sudo /usr/local/bin/k3s-agent-uninstall.sh
```

Then reinstall as per earlier lessons.

---

## ✅ General Troubleshooting Checklist

| Step                       | Command                         |
| -------------------------- | ------------------------------- |
| Check node status          | `kubectl get nodes`             |
| View K3s logs              | `journalctl -u k3s -e`          |
| View agent logs (worker)   | `journalctl -u k3s-agent -e`    |
| Check config               | `cat /etc/rancher/k3s/k3s.yaml` |
| Inspect cluster components | `kubectl get pods -A`           |
{:class="table table-striped"}

---

Once your cluster is running and stable, it’s time to start using it!

Next up: [Using Your K3s Cluster](08_kubectl_basics)

---
