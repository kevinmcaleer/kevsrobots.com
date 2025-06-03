---
layout: lesson
title: Migration Project
author: Kevin McAleer
type: page
cover: assets/podman03.jpg
date: 2025-05-24
previous: 11_troubleshooting_and_debugging.html
next: 13_wrap_up_and_future.html
description: "Apply everything you\u2019ve learned by migrating a Docker-based application\
  \ to Podman, including Compose conversion, systemd integration, and rootless configuration."
percent: 91
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

It’s time to put your knowledge into practice! In this hands-on project, you’ll migrate a simple Docker-based application to Podman, step by step.

You’ll see how to:

- Convert a Docker Compose setup to Podman
- Use `podman-compose` or pods directly
- Add systemd integration
- Run everything rootlessly

---

## 🎯 Project Goals

By the end of this project, you will have:

- Replaced `docker` commands with `podman`
- Converted a `docker-compose.yml` to `podman-compose`
- Created a rootless pod
- Generated a systemd service to manage your container

---

## 📁 Step 1: Clone the Sample App

```bash
git clone https://github.com/kevinmcaleer/docker-migration-demo.git
cd docker-migration-demo
````

This app contains:

* `docker-compose.yml`
* A simple web app (Node.js or Flask)
* A Redis backend

---

## 🔁 Step 2: Run with Podman Compose

Install `podman-compose` if you haven’t already:

```bash
pip3 install podman-compose
```

Then run:

```bash
podman-compose up
```

Inspect the generated pod:

```bash
podman pod ps
```

Check logs:

```bash
podman logs <container-id>
```

---

## 🛠 Step 3: Migrate to Native Podman Commands (Optional)

Create a pod manually:

```bash
podman pod create --name myapp -p 8080:80
```

Start your containers inside the pod:

```bash
podman run -dt --pod myapp --name redis redis
podman run -dt --pod myapp --name webapp myapp-image
```

---

## ⚙️ Step 4: Add Systemd Integration

Generate a systemd unit file:

```bash
podman generate systemd --name webapp --files --restart-policy=always
```

Move it into place:

```bash
mkdir -p ~/.config/systemd/user
mv container-webapp.service ~/.config/systemd/user/
systemctl --user daemon-reexec
systemctl --user enable --now container-webapp.service
```

Repeat for other services as needed.

---

## 🔐 Step 5: Verify Rootless Configuration

Ensure no root privileges are required:

```bash
whoami
podman ps
```

You should see your containers running under your user.

---

## ✅ Project Complete

You’ve now fully migrated a Docker-based app to Podman:

* ✅ Replaced Docker CLI
* ✅ Used `podman-compose` or pods
* ✅ Enabled rootless containerization
* ✅ Integrated with systemd for background service management

---

## 🚀 Challenge

Try deploying another project with:

* Kubernetes YAML (via `generate kube`)
* A more complex `docker-compose.yml`
* Additional features like volume mounts, secrets, or health checks

---

Next up: [Wrap-up and Future Outlook](13_wrap_up_and_future)

---
