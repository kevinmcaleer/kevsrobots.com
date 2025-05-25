---
layout: lesson
title: CLI Compatibility and Basic Commands
author: Kevin McAleer
type: page
cover: assets/podman04.jpg
date: 2025-05-24
previous: 03_installing_podman.html
next: 05_podman_unique_features.html
description: Learn how to use Podman with Docker-compatible commands and explore its
  CLI for managing containers, images, and volumes.
percent: 35
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

One of Podman’s strongest features is its **Docker CLI compatibility**. In most cases, you can use the exact same commands — often by simply replacing the word `docker` with `podman`.

This makes switching from Docker to Podman easier than you might expect.

---

## 🧪 Try It: Podman vs Docker Commands

| Task                        | Docker Command                  | Podman Command                 |
|-----------------------------|----------------------------------|--------------------------------|
| Run a container             | `docker run -it alpine`         | `podman run -it alpine`        |
| List running containers     | `docker ps`                     | `podman ps`                    |
| List all containers         | `docker ps -a`                  | `podman ps -a`                 |
| Build an image              | `docker build -t myapp .`       | `podman build -t myapp .`      |
| View logs                   | `docker logs <id>`              | `podman logs <id>`             |
| Stop a container            | `docker stop <id>`              | `podman stop <id>`             |
| Remove a container          | `docker rm <id>`                | `podman rm <id>`               |
| Remove an image             | `docker rmi <image>`            | `podman rmi <image>`           |
{:class="table table-striped"}

---

> 🧠 **Tip:** You can even create an alias:
> ```bash
> alias docker=podman
> ```

---

## 🔍 What’s the Same?

- Most of the Docker CLI commands
- Image management using `pull`, `push`, `build`
- Container lifecycle management (`run`, `exec`, `rm`, `stop`, `logs`)
- Volume and network handling

---

## 🔄 What’s Different?

### 🔧 No Daemon

Podman commands run directly without a background service. This affects:

- How containers are started and monitored
- How logs and container state are handled

### 🛠 Rootless by Default

Most Podman installations use rootless mode:

- Your containers run as your user (no `sudo` needed)
- Files created by containers belong to your user

> 📁 Volumes and files created in containers are owned by the current user, not `root`, in rootless mode.

### 🧰 System Commands

Some Docker-style commands (e.g., `docker system prune`) may behave differently or be absent in Podman. Podman provides alternatives like:

```bash
podman image prune
podman container prune
````

---

## 🧪 Try It Yourself

Start a container and explore the CLI:

```bash
podman run -it --rm alpine
```

Inside the container:

```sh
apk add curl
```

Then exit and try listing and removing the container:

```bash
podman ps -a
```

---

## 🔐 Rootless vs Rootful Mode

You can choose to run Podman in root mode if needed:

```bash
sudo podman run ...
```

But in most cases, rootless mode is safer and preferred — especially for development.

---

Next up: [Podman Features Docker Doesn’t Have](05_podman_unique_features)

---
