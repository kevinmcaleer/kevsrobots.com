---
layout: lesson
title: Troubleshooting and Debugging
author: Kevin McAleer
type: page
cover: assets/podman02.jpg
date: 2025-05-24
previous: 10_podman_in_production.html
next: 12_migration_project.html
description: Learn how to troubleshoot and debug Podman containers using built-in
  tools like `inspect`, `logs`, `exec`, and systemd journal logs.
percent: 84
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

Even with a solid setup, container issues are bound to happen. In this lesson, you'll learn how to **debug Podman containers**, interpret logs, inspect configurations, and resolve common problems.

---

## 🛠 Inspecting Containers

Start with `podman inspect` to view everything about a container:

```bash
podman inspect <container-id>
````

This gives you:

* Environment variables
* Volume mounts
* Network settings
* Restart policies
* Container state and exit codes

> 📦 Use `jq` or `grep` to filter specific values:
>
> ```bash
> podman inspect <id> | jq '.[0].State.ExitCode'
> ```

---

## 📜 Viewing Logs

To see standard output and error logs:

```bash
podman logs <container-id>
```

To stream logs live:

```bash
podman logs -f <container-id>
```

If using systemd:

```bash
journalctl --user -u container-<name>
```

> 🧠 Podman containers integrate cleanly with the **Linux journal** when managed via systemd.

---

## 🧪 Executing Commands in a Running Container

Use `exec` to get a shell or run diagnostics inside a container:

```bash
podman exec -it <container-id> sh
```

Or if it has bash:

```bash
podman exec -it <container-id> bash
```

This is great for checking filesystem paths, environment variables, or running troubleshooting tools (e.g. `curl`, `ping`).

---

## ❌ Common Errors and Fixes

### "Image not found"

Check spelling and tags:

```bash
podman pull <correct-image-name>
```

### Port binding fails

Ensure the port is free or mapped correctly:

```bash
podman run -p 8080:80 ...
```

### Permission denied (rootless volume)

Make sure the host folder is writable by your user:

```bash
mkdir -p ~/mydata
chmod 755 ~/mydata
```

Mount it:

```bash
podman run -v ~/mydata:/data ...
```

### Container immediately exits

Check the container’s command:

```bash
podman inspect <id> | jq '.[0].Config.Cmd'
```

Or run with an interactive shell:

```bash
podman run -it <image> sh
```

---

## 🧹 Cleaning Up

Remove exited containers:

```bash
podman container prune
```

Remove dangling images:

```bash
podman image prune
```

Remove unused volumes:

```bash
podman volume prune
```

---

## 🧰 Useful Tools

| Tool/Command     | Purpose                          |
| ---------------- | -------------------------------- |
| `podman ps -a`   | Show all containers              |
| `podman stats`   | Real-time resource usage         |
| `podman diff`    | Show file changes in a container |
| `podman mount`   | Access container filesystem      |
| `podman history` | View image layer history         |
{:class="table table-striped"}

---

Next up: [Migration Project](12_migration_project)

---
