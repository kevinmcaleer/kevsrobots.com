---
layout: lesson
title: Introduction to K3s on Raspberry Pi
author: Kevin McAleer
type: page
cover: assets/cover.jpg
date: 2025-06-01
next: 02_hardware_requirements.html
description: Learn what K3s is, why it's a great fit for the Raspberry Pi, and what
  this course will help you build.
percent: 5
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

Welcome to **Running K3s on Raspberry Pi**!

In this course, you’ll learn how to create a lightweight, fully functional **Kubernetes cluster** using **K3s** — all powered by **Raspberry Pi single-board computers**.

---

## 🚀 What Is K3s?

**K3s** is a lightweight Kubernetes distribution created by Rancher (now part of SUSE). It’s designed for:

- Edge computing
- IoT environments
- Low-resource devices (like Raspberry Pis)

K3s includes all the essential components of Kubernetes, while stripping away features not commonly used outside of large cloud clusters.

---

## 🍓 Why Use K3s on Raspberry Pi?

| Benefit                     | Why It Matters                        |
|-----------------------------|----------------------------------------|
| 🧠 Learn Kubernetes         | Build real clusters with real tools    |
| 💡 Low Cost                | Great for budget labs and education    |
| 🌍 Portable & Scalable     | Run anywhere: desk, shed, edge         |
| ⚡ Lightweight              | K3s runs well on 1GB–4GB RAM devices   |
| 🛠️ Real Workloads         | Deploy apps, databases, dashboards     |

---

## 🧰 What You’ll Learn

By the end of this course, you’ll be able to:

- Set up a Raspberry Pi cluster from scratch
- Install and configure K3s
- Add nodes and configure networking
- Deploy and manage real-world apps using Kubernetes
- Set up dashboards, storage, ingress, and CI/CD pipelines
- Secure your cluster for production use

---

## 📦 What You’ll Need

- 2–4 Raspberry Pi 4 or 5 boards (ideally with 2GB+ RAM)
- microSD cards (32GB+ recommended)
- Ethernet cables and switch/router
- Power supply for each Pi
- USB SSDs (optional, but recommended for reliability)
- A separate computer with `ssh` and `kubectl` installed
- Internet access for your cluster

---

## 📋 Course Format

Each lesson in this course builds on the previous one. Lessons include:

- Clear instructions and diagrams
- Code snippets and real terminal commands
- Troubleshooting tips
- Hands-on labs and a final deployment project

---

Let’s get started by assembling your Raspberry Pi cluster and preparing the OS!

Next up: [Hardware Requirements](02_hardware_requirements)

---
