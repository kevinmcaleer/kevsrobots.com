---
layout: lesson
title: Installing K3s
author: Kevin McAleer
type: page
cover: assets/k3s04.jpg
date: 2025-06-01
previous: 04_cluster_networking.html
next: 06_adding_nodes.html
description: Install K3s on your Raspberry Pi cluster, configure your master node,
  and prepare for adding workers.
percent: 25
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

In this lesson, you’ll install **K3s** on your Raspberry Pi master node, set up the cluster, and prepare for adding worker nodes.

K3s makes Kubernetes installation fast and easy — especially on resource-limited devices like Raspberry Pi.

---

## 🚀 Step 1: SSH into the Master Node

From your main computer, SSH into the Pi that will act as the **control plane (master)**:

```bash
ssh pi@pi-master.local
```

> Replace `pi` and `pi-master.local` with your username and hostname or IP if needed.

---

## 🧰 Step 2: Install K3s (Master Node)

Run this one-liner:

```bash
curl -sfL https://get.k3s.io | sh -
```

K3s will:

* Download and install Kubernetes
* Start systemd service: `k3s.service`
* Set up networking with Flannel
* Create a default service account and kubeconfig

---

## 📁 Step 3: Check Cluster Status

After installation:

```bash
sudo k3s kubectl get nodes
```

You should see your master node listed as `Ready`.

To make `kubectl` easier to use, copy the K3s kubeconfig to your user’s home directory:

```bash
mkdir -p ~/.kube
sudo cp /etc/rancher/k3s/k3s.yaml ~/.kube/config
sudo chown $(id -u):$(id -g) ~/.kube/config
```

Now you can run:

```bash
kubectl get nodes
```

---

## 🔑 Step 4: Get the Join Token

You’ll need a token to join worker nodes to the cluster:

```bash
sudo cat /var/lib/rancher/k3s/server/node-token
```

Copy the token — you’ll use it on the worker nodes in the next lesson.

---

## 🧪 Troubleshooting

| Problem                      | Solution                                    |
| ---------------------------- | ------------------------------------------- |
| `k3s.service` fails to start | Check logs: `journalctl -u k3s -e`          |
| `kubectl` not found          | Use `sudo k3s kubectl` or set up kubeconfig |
| Token file missing           | Confirm you're on the master node           |
| Network errors               | Check for cgroup settings and firewalls     |
{:class="table table-striped"}

---

## 📝 Notes

* K3s uses a built-in SQLite database by default (great for small clusters).
* For multi-master HA setups, use an external datastore (e.g. MySQL, Postgres, etcd).
* The default CNI is **Flannel**, which works well for most local setups.

---

🎉 Your Kubernetes control plane is now running!

Next up: [Adding Nodes](06_adding_nodes)

---
