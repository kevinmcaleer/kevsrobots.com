---
layout: course_overview
title: Running K3s on Raspberry Pi
author: Kevin McAleer
cover: /learn/k3s/assets/cover.jpg
date: 2025-06-01
description: Learn how to build your own lightweight Kubernetes cluster using K3s
  and Raspberry Pi. This hands-on course will guide you through setup, configuration,
  networking, persistent storage, and deploying real workloads, all on affordable
  single-board computers. Ideal for homelabbers, IoT developers, and anyone curious
  about Kubernetes at the edge.
level: intermediate
course_duration: 1h 40m
lesson_count: 18
first_lesson: 00_intro.html
quiz_total: 54
outcomes:
- Build a multi-node K3s cluster from Raspberry Pi boards, including the memory cgroup
  and fixed addresses
- Install a K3s server, join workers and troubleshoot a failed install from the service
  logs
- Use kubectl from your own computer to deploy, expose, scale and update apps
- Keep data with PersistentVolumeClaims and publish apps with Services and a Traefik
  Ingress
- Package an app with Helm and automate deployment with a runner or GitOps
- Protect the cluster with Secrets, RBAC, NetworkPolicy and TLS
prerequisites:
- Introduction to the Linux Command Line on Raspberry Pi OS
- Helpful but not required, the Docker course
you_need:
- 2 to 4 Raspberry Pi 4 or 5 boards (2GB of RAM or more, 4GB for the server)
- A microSD card or USB SSD per Pi, and the official power supplies
- Ethernet cables and a switch or router
- A computer with a terminal and ssh, to install kubectl and Helm on
navigation:
- name: Running K3s on Raspberry Pi
- content:
  - section: Introduction
    content:
    - name: Introduction to K3s on Raspberry Pi
      link: 00_intro.html
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
    - name: Advanced Networking and Custom CNI
      link: 15_custom_cni.html
    - name: Securing Your Cluster
      link: 16_securing_your_cluster.html
  - section: Final Project
    content:
    - name: Final Project - A Visit Counter That Survives
      link: 17_final_project.html
  - section: Summary
    content:
    - name: Course Summary
      link: 18_summary.html
---
