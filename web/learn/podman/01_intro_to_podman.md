---
layout: lesson
title: Introduction to Podman
author: Kevin McAleer
type: page
cover: assets/podman01.jpg
date: 2025-05-24
next: 01_video.html
description: Learn what Podman is, why it's a powerful alternative to Docker, and
  how it's changing the container landscape.
percent: 7
duration: 2
date_updated: 2025-05-24
navigation:
- name: From Docker to Podman
- content:
  - section: Getting Started
    content:
    - name: Introduction to Podman
      link: 01_intro_to_podman.html
    - name: 'Video: Podman vs Docker'
      link: 01_video.html
    - name: Docker vs Podman Architecture
      link: 02_docker_vs_podman_architecture.html
    - name: Installing Podman
      link: 03_installing_podman.html
  - section: Core Usage
    content:
    - name: CLI Compatibility and Basic Commands
      link: 04_cli_compatibility_and_commands.html
    - name: "Podman Features Docker Doesn\u2019t Have"
      link: 05_podman_unique_features.html
    - name: Managing Containers and Images
      link: 06_managing_containers_and_images.html
  - section: Advanced Concepts
    content:
    - name: Working with Pods
      link: 07_working_with_pods.html
    - name: Docker Compose vs Podman Compose
      link: 08_docker_compose_vs_podman_compose.html
    - name: Podman and Kubernetes
      link: 09_podman_and_kubernetes.html
  - section: Real-World Usage
    content:
    - name: Podman in Production
      link: 10_podman_in_production.html
    - name: Troubleshooting and Debugging
      link: 11_troubleshooting_and_debugging.html
  - section: Migration Lab
    content:
    - name: Migration Project
      link: 12_migration_project.html
  - section: Summary
    content:
    - name: Wrap-up and Future Outlook
      link: 13_wrap_up_and_future.html
---


![Cover]({{page.cover}}){:class="cover"}

---

If you've used Docker, you're already familiar with how containers can revolutionize software development. But Docker isn't the only option.

**Podman** offers a secure, daemonless, and rootless alternative — and it’s designed to be a drop-in replacement for Docker in most scenarios.

---

## 📦 What You’ll Learn

In this lesson, you’ll:

- Understand what Podman is and how it differs from Docker
- Learn the key benefits of Podman's design
- Discover how Podman fits into the OCI ecosystem
- See why developers and sysadmins are switching

---

## 🔍 Why Podman?

Podman is part of a modern container toolkit built to address many limitations of Docker, including:

- **Security:** Run containers without needing root privileges
- **Daemonless Design:** No central daemon means better reliability and auditability
- **System Integration:** Easily integrate containers with `systemd`
- **Docker Compatibility:** Use the same commands and scripts

---

## 🧠 A Bit of Background

Podman was created by Red Hat as a more secure and flexible alternative to Docker, and it works well with other tools like:

- **Buildah** (for image building)
- **Skopeo** (for image transfer)
- **CRI-O** (Kubernetes runtime)

Together, these form a robust, modular container toolchain.

---

## ✅ Prerequisites

Before you start, you should:

- Be familiar with Docker basics (e.g., `docker run`, `docker build`)
- Have a terminal environment ready (Linux, macOS, or WSL)

---

Next up: [Docker vs Podman Architecture](02_docker_vs_podman_architecture)

---
