---
title: Troubleshooting and Debugging
description: Learn how to troubleshoot and debug Podman containers using built-in tools like `inspect`, `logs`, `exec`, and systemd journal logs.
layout: lesson
type: page
cover: assets/podman02.jpg
date_updated: 2025-05-24
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
