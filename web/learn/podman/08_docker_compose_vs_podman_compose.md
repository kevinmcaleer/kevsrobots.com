---
layout: lesson
title: Docker Compose vs Podman Compose
author: Kevin McAleer
type: page
cover: assets/podman02.jpg
date: 2025-05-24
previous: 07_working_with_pods.html
next: 09_podman_and_kubernetes.html
description: Learn how to use `podman-compose` to run multi-container apps, and understand
  the differences and limitations compared to Docker Compose.
percent: 56
duration: 3
date_updated: 2025-05-24
navigation:
- name: From Docker to Podman
- content:
  - section: Getting Started
    content:
    - name: Introduction to Podman
      link: 01_intro_to_podman.html
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

If you've built multi-container applications with **Docker Compose**, you'll be pleased to know that **Podman offers a compatible tool**: `podman-compose`.

It parses `docker-compose.yml` files and launches containers using Podman — often with minimal changes.

---

## 📦 What is `podman-compose`?

`podman-compose` is a Python-based CLI tool that:

- Uses your existing `docker-compose.yml` files
- Creates a Podman **pod** and runs each service inside it
- Tries to maintain CLI and file compatibility with Docker Compose

> 🛠 It’s great for local development and small deployments — but not a 1:1 drop-in replacement in all cases.

---

## 🛠 Installing `podman-compose`

```bash
pip3 install podman-compose
````

Verify installation:

```bash
podman-compose version
```

---

## ▶️ Running a Compose File with Podman

Assume you have this `docker-compose.yml`:

```yaml
version: "3"
services:
  web:
    image: nginx
    ports:
      - "8080:80"
  redis:
    image: redis
```

Run it with:

```bash
podman-compose up
```

Stop it with:

```bash
podman-compose down
```

> ✅ By default, Podman Compose creates a pod named after your project and places all containers inside it.

---

## 🔍 Checking Pod Status

Check running pods and containers:

```bash
podman pod ps
podman ps
```

Inspect the pod:

```bash
podman pod inspect <pod-id>
```

---

## ⚠️ Limitations Compared to Docker Compose

| Feature                  | Docker Compose  | Podman Compose         |
| ------------------------ | --------------- | ---------------------- |
| Service restart policies | ✅ Supported     | ⚠️ Limited/partial     |
| Health checks            | ✅ Supported     | ❌ Not yet supported    |
| Compose v2 syntax        | ✅ Supported     | ⚠️ Partially supported |
| Networking               | Built-in bridge | Pod network/pod-based  |
| Volumes                  | Supported       | Supported              |
{:class="table table-striped"}

> 🧪 Podman Compose works best with **basic service definitions** and standard port/volume usage. Advanced features may require tweaking or systemd integration.

---

## 🧰 Alternative: Manual Pod + Systemd

For production use, many Podman users skip Compose altogether and:

- Use `podman generate systemd`
- Or define services as Kubernetes YAML
- Or use Podman’s native `pod` support for simple service grouping

---

## 🗂 Best Practices

- Use `podman-compose` for local testing and development
- Prefer systemd or `podman play kube` for production
- Keep Compose files simple and compatible with the Compose v2 spec

---

Next up: [Podman and Kubernetes](09_podman_and_kubernetes)

---
