---
title: Managing Containers and Images
description: Learn how to manage containers, images, volumes, and logs using Podman’s CLI — including useful commands for day-to-day development and operations.
layout: lesson
type: page
cover: assets/podman03.jpg
date_updated: 2025-05-24
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
