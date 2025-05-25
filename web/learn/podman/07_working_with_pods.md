---
layout: lesson
title: Working with Pods
author: Kevin McAleer
type: page
cover: assets/podman01.jpg
date: 2025-05-24
previous: 06_managing_containers_and_images.html
next: 08_docker_compose_vs_podman_compose.html
description: "Learn how to use Podman\u2019s built-in support for pods to group and\
  \ manage multiple containers with shared networking \u2014 just like Kubernetes."
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

One of Podman’s standout features is its native support for **pods** — a concept borrowed directly from Kubernetes.

Pods allow you to group containers so they share the same **network namespace**, making it easy to colocate services like a web server and a database.

---

## 🧠 What Is a Pod?

A **pod** is a group of one or more containers that:

- Share the same IP address and port space
- Communicate over `localhost`
- Can be managed together

> 📦 This is useful when building applications with tightly coupled components that need to run together — and it mirrors how Kubernetes deploys workloads.

---

## ➕ Creating a Pod

Create a new pod:

```bash
podman pod create --name webstack
```

List all pods:

```bash
podman pod ps
```

Inspect a pod:

```bash
podman pod inspect webstack
```

---

## 🧱 Adding Containers to a Pod

Add a container to your pod:

```bash
podman run -dt --pod webstack nginx
```

Then add another:

```bash
podman run -dt --pod webstack redis
```

> ### Podman run flags
>
> The `podman run` command uses several flags:
> - `-d` runs the container in detached mode
> - `-t` allocates a pseudo-TTY
> - `--pod webstack` specifies the pod to join

---

Both containers now share:

- The same IP
- Same ports
- `localhost` connectivity

> 🔧 This is very similar to running containers in a `docker-compose` network, but native to the Podman CLI.

---

## 🔍 Viewing Pod Details

See what's inside the pod:

```bash
podman pod inspect webstack
```

List all containers in a pod:

```bash
podman ps --pod
```

Stop and remove the pod (and its containers):

```bash
podman pod stop webstack
podman pod rm webstack
```

---

## 🧪 Example: Web App + Database Pod

```bash
podman pod create --name blogpod -p 8080:80

podman run -dt --pod blogpod --name blogdb \
  -e POSTGRES_PASSWORD=secret postgres

podman run -dt --pod blogpod --name blogweb \
  -e DATABASE_URL=postgres://postgres:secret@localhost/postgres \
  myblog:latest
```

Your `blogweb` container can now talk to `blogdb` over `localhost:5432`.

---

## ⚠️ Notes and Limitations

- Containers in a pod share network, but **not filesystem** by default
- Each container still needs its own volume or mount
- Some tooling may not fully support pods (e.g., `podman-compose`)

---

## 🧩 Pods and Kubernetes

Podman’s pod feature is a stepping stone to full Kubernetes deployment. You can even export pods as Kubernetes YAML:

```bash
podman generate kube blogpod > blogpod.yaml
```

Then replay it later with:

```bash
podman play kube blogpod.yaml
```

> 🔄 This makes Podman a great tool for **local development** of Kubernetes apps.

---

Next up: [Docker Compose vs Podman Compose](08_docker_compose_vs_podman_compose)

---
