---
title: Working with Pods
description: Learn how to use Podman’s built-in support for pods to group and manage multiple containers with shared networking — just like Kubernetes.
layout: lesson
type: page
cover: assets/podman01.jpg
date_updated: 2025-05-24
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
