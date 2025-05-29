---
layout: lesson
title: Using Helm
author: Kevin McAleer
type: page
cover: assets/k3s06.jpg
date: 2025-06-01
previous: 12_deploying_apps.html
next: 14_ci_cd_on_k3s.html
description: Learn how to use Helm to deploy and manage applications on your K3s cluster
  with reusable charts and versioned releases.
percent: 65
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

As your Kubernetes setup grows, writing and maintaining raw YAML files becomes harder to manage. That’s where **Helm** comes in.

Helm is a **package manager for Kubernetes** — like apt for Ubuntu or pip for Python. It allows you to deploy applications as **charts**, with versioning, values, and easy upgrades.

---

## 🎯 Why Use Helm?

| Feature       | Benefit                      |
|---------------|------------------------------|
| Charts        | Reusable templates for apps  |
| Versioning    | Rollbacks and upgrades       |
| `values.yaml` | Customization made easy      |
| Ecosystem     | Thousands of prebuilt charts |
{:class="table table-striped"}

---

## 🛠 Step 1: Install Helm

On your **development machine** or master node:

```bash
curl https://raw.githubusercontent.com/helm/helm/main/scripts/get-helm-3 | bash
````

Verify:

```bash
helm version
```

---

## 📦 Step 2: Add a Helm Repository

Helm charts are hosted in repositories like Docker images.

Add the Bitnami repo (for common apps):

```bash
helm repo add bitnami https://charts.bitnami.com/bitnami
helm repo update
```

---

## 🚀 Step 3: Install an App with Helm

Let’s install **WordPress** as an example:

```bash
helm install my-blog bitnami/wordpress
```

This will:

* Deploy WordPress + MariaDB
* Set up storage
* Create services

Check resources:

```bash
kubectl get all
```

---

## ⚙️ Step 4: Customize the Install

You can pass custom values inline:

```bash
helm install my-blog bitnami/wordpress \
  --set wordpressUsername=admin \
  --set wordpressPassword=secretpass
```

Or create a `values.yaml` file for reproducible installs.

---

## 🔄 Step 5: Upgrade and Rollback

Update your chart:

```bash
helm upgrade my-blog bitnami/wordpress --set wordpressBlogName="New Blog"
```

Roll back to the previous version:

```bash
helm rollback my-blog
```

List releases:

```bash
helm list
```

---

## 🧹 Uninstall a Release

```bash
helm uninstall my-blog
```

All resources created by the chart will be removed.

---

## 🧠 Common Use Cases for Helm

* Deploying **Dashboards**, **Monitoring** (e.g. Prometheus, Grafana)
* Managing **Databases**, **Message Queues**
* Packaging **your own apps** with custom charts
* Automating with **CI/CD pipelines**

---

## ✅ Summary

You now know how to:

* Install and configure Helm
* Use Helm charts to deploy apps
* Customize installs with `values.yaml`
* Upgrade, rollback, and uninstall releases

Helm makes managing Kubernetes apps much easier — especially for repeatable deployments and automation.

---

Next up: [CI/CD on K3s](14_ci_cd_on_k3s)

---
