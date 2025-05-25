---
layout: lesson
title: Managing Containers and Images
author: Kevin McAleer
type: page
cover: assets/podman03.jpg
date: 2025-05-24
previous: 05_podman_unique_features.html
next: 07_working_with_pods.html
description: "Learn how to manage containers, images, volumes, and logs using Podman\u2019\
  s CLI \u2014 including useful commands for day-to-day development and operations."
percent: 49
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

Now that you know what makes Podman powerful, let’s get hands-on with **managing containers and images**.

You’ll use the Podman CLI to list, inspect, stop, remove, and troubleshoot containers — just like you would with Docker, but daemonless and rootless.

---

## 📋 Listing and Inspecting Containers

### View running containers

```bash
podman ps
```

### View all containers (including stopped)

```bash
podman ps -a
```

### Inspect a specific container

```bash
podman inspect <container-id>
```

> 🧠 This returns detailed JSON output including network, storage, environment variables, and more.

---

## 🧼 Starting, Stopping, and Removing

### Start a container

```bash
podman start <container-id>
```

### Stop a container

```bash
podman stop <container-id>
```

### Remove a container:

```bash
podman rm <container-id>
```

> 🛑 Always stop a container before removing it, unless you use `-f` (force).

---

## 🖼 Managing Images

### Pull an image

```bash
podman pull ubuntu
```

### List local images

```bash
podman images
```

### Remove an image

```bash
podman rmi <image-id>
```

> 🔍 Need more control? Use `podman image inspect` or `podman image history` to see build layers and metadata.

---

## 📦 Volumes and Persistent Storage

### Create a named volume

```bash
podman volume create mydata
```

### Use a volume in a container

```bash
podman run -v mydata:/data alpine
```

### List volumes

```bash
podman volume ls
```

### Remove a volume

```bash
podman volume rm mydata
```

> 📁 Podman volumes live under `~/.local/share/containers/storage/volumes` (in rootless mode).

---

## 🪵 Logs and Output

### View container logs

```bash
podman logs <container-id>
```

### Follow logs in real time

```bash
podman logs -f <container-id>
```

> 📜 Podman log output goes to the standard user log files in rootless mode. Combine with `journalctl` for systemd-managed containers.

---

## 🧰 Useful Extras

### Copy files into a container

```bash
podman cp myfile.txt <container-id>:/root/
```

### Execute a command in a running container

```bash
podman exec -it <container-id> bash
```

### Clean up unused containers and images

```bash
podman container prune
podman image prune
```

---

Next up: [Working with Pods](07_working_with_pods)

---
