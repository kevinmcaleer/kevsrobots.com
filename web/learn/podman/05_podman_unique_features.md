---
layout: lesson
title: "Podman Features Docker Doesn\u2019t Have"
author: Kevin McAleer
type: page
cover: assets/podman02.jpg
date: 2025-05-24
previous: 04_cli_compatibility_and_commands.html
next: 06_managing_containers_and_images.html
description: "Discover the powerful features that set Podman apart from Docker \u2014\
  \ including rootless containers, systemd integration, and pods."
percent: 42
duration: 3
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

While Podman offers strong Docker compatibility, it also introduces **powerful features** that go beyond Docker’s capabilities.

In this lesson, we’ll explore the unique capabilities of Podman that make it a favorite among developers, sysadmins, and DevOps teams.

---

## Rootless Containers

Podman was built from the ground up with **rootless** containers in mind:

- Run containers **as your own user**
- No `sudo`, no elevated permissions
- Improves security and auditability

> Containers cannot access system-wide files unless explicitly allowed — helping to prevent privilege escalation vulnerabilities.

---

## Daemonless Architecture

No long-running daemon process:

- Every container is launched and managed by your shell
- Easier to understand and debug
- No single point of failure

> Each Podman container is just a **regular Linux process**, making it more transparent than Docker’s centralized daemon model.

---

## Systemd Integration

You can generate native `systemd` unit files from containers or pods:

```bash
podman generate systemd --name mycontainer --files --restart-policy=always
````

Then:

```bash
systemctl --user enable container-mycontainer.service
systemctl --user start container-mycontainer.service
```

> This makes Podman perfect for running containers **as background services** that survive reboots — especially on servers or IoT devices.

---

## Pods: Kubernetes-Style Container Grouping

Podman supports **pods**, just like Kubernetes:

```bash
podman pod create --name webpod
podman run -dt --pod webpod nginx
podman run -dt --pod webpod redis
```

- Multiple containers share the same network namespace
- Ideal for colocating services (e.g., app + DB + reverse proxy)
- Mirrors the Kubernetes model — great for learning

---

## Modular Ecosystem

Podman is part of a container toolchain that includes:

- **Buildah** – image building (like `docker build`, but without a daemon)
- **Skopeo** – image copying and signing
- **CRI-O** – Kubernetes runtime alternative to Docker

These tools can be combined or used independently for **fine-grained control**.

---

## Enhanced Security with SELinux and AppArmor

Podman has **first-class support for SELinux and AppArmor**, allowing more robust security profiles:

- Containers are isolated using Linux security modules
- Great for use in hardened or multi-user environments

> Rootless + SELinux = secure containers with minimal privileges

---

## Summary of Unique Features

| Feature               |  Docker   |  Podman  |
|-----------------------|:----------|:---------|
| Rootless by default   |         |        |
| Daemonless            |         |        |
| Systemd integration   | Manual | Native |
| Kubernetes-style pods |         |        |
| SELinux/AppArmor      |  Limited  |  Full  |
| OCI toolchain support |         |        |
{:class="table table-striped"}

---

Next up: [Managing Containers and Images](06_managing_containers_and_images)

---
