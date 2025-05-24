---
title: Docker vs Podman Architecture
description: Understand the key architectural differences between Docker and Podman, and why Podman's design offers improved security and flexibility.
layout: lesson
type: page
cover: assets/podman02.jpg
date_updated: 2025-05-24
---

![Cover]({{page.cover}}){:class="cover"}

---

To appreciate Podman’s advantages, it’s important to understand how it differs architecturally from Docker. The biggest shift? **No daemon.**

> What is a **Daemon?** A daemon is a background service that runs continuously, managing tasks and resources. In Docker, the `dockerd` daemon is responsible for running containers, while Podman operates without a central daemon.

---

Let’s explore the implications of this and what it means for how you work with containers.

---

## 🏗️ Docker’s Architecture

Docker uses a **client-server** model:

- The `docker` CLI talks to the `dockerd` daemon
- The daemon manages containers, images, volumes, and networks
- The daemon runs as root and controls everything on behalf of the user

### Pros:
- Centralized control
- Simplifies resource management

### Cons:
- Single point of failure
- Requires root privileges
- Difficult to audit individual actions

---

## 🔧 Podman’s Architecture

Podman is **daemonless** and **runs as the user**:

- The `podman` CLI executes commands directly
- Each container runs under its own system process
- No root access required by default (rootless)

{% include gallery.html images="/learn/podman/assets/comparison.png" titles="Docker vs Podman" descriptions="Podman has no daemon and can run rootless" noborder=true cols=2 %}

---

### Pros:

- No central daemon = fewer security concerns
- Runs as the current user = better access control
- Easy to integrate with `systemd` and user services
- Compatible with container standards (OCI)

---

## 🛡️ Security Implications

| Feature               | Docker                  | Podman                       |
|----------------------|:------------------------|:-----------------------------|
| Rootless Support      | ❌ Experimental/limited | ✅ Full support               |
| Daemon Required       | ✅ Yes                  | ❌ No                         |
| SELinux Integration   | 🟡 Limited              | ✅ Strong                     |
| Auditability          | ❌ Shared daemon logs   | ✅ Per-process/user logging   |
{:class="table table-striped"}

---

> 🔒 **Rootless containers** allow users to run containers without elevated privileges, greatly reducing attack surface.

---

## 🧠 Why It Matters

Podman’s architecture empowers users and sysadmins to:

- Run containers without special permissions
- Avoid the risks of a long-running root daemon
- Cleanly separate user workloads
- Better support containers as system services

---

## 🚧 When It Matters

- **On servers:** Fewer root processes = safer environment
- **In dev environments:** No need for `sudo`
- **With `systemd`:** Map containers directly to services
- **In shared machines:** Users manage their own containers securely

---

Next up: [Installing Podman](03_installing_podman)

---
