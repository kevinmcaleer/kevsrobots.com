---
title: Docker Compose vs Podman Compose
description: Learn how to use `podman-compose` to run multi-container apps, and understand the differences and limitations compared to Docker Compose.
layout: lesson
type: page
cover: assets/podman02.jpg
date_updated: 2025-05-24
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
