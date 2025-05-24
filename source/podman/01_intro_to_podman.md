---
title: Introduction to Podman
description: Learn what Podman is, why it's a powerful alternative to Docker, and how it's changing the container landscape.
layout: lesson
type: page
cover: assets/podman01.jpg
date_updated: 2025-05-24
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
