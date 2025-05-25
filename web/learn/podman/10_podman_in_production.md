---
layout: lesson
title: Podman in Production
author: Kevin McAleer
type: page
cover: assets/podman01.jpg
date: 2025-05-24
previous: 09_podman_and_kubernetes.html
next: 11_troubleshooting_and_debugging.html
description: Learn how to use Podman in production environments, including rootless
  services, systemd integration, monitoring, and best practices.
percent: 77
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

Podman isn’t just a developer tool — it’s also well-suited for **production environments**. Thanks to its **daemonless**, **rootless**, and **systemd-friendly** design, it integrates cleanly into traditional Linux server workflows.

Let’s explore how to run containers in production with confidence.

---

## 🛠 Rootless in Production

One of Podman’s key strengths is that it runs containers as the current user:

- No need to run `sudo` or expose root privileges
- Keeps services isolated to specific users
- Reduces the impact of potential security breaches

> ✅ Great for shared environments or untrusted code execution

---

## ⚙️ Systemd Integration

Use `podman generate systemd` to create service unit files that run containers like any other Linux service.

### Example

```bash
podman create --name myapp -p 8080:80 myimage
podman generate systemd --name myapp --files --restart-policy=always
````

Then move the generated unit file into your user systemd directory:

```bash
mkdir -p ~/.config/systemd/user
mv container-myapp.service ~/.config/systemd/user/
systemctl --user enable --now container-myapp.service
```

This ensures:

- Containers start on boot
- Restart policies are respected
- Logs are available via `journalctl`

---

## 📈 Monitoring and Logging

Since containers are just regular Linux processes, you can use standard tools:

- `top`, `htop`, `ps` to view running processes
- `journalctl --user -u container-myapp` for logs
- Podman’s built-in logging:

  ```bash
  podman logs myapp
  ```

> 📊 You can also integrate with monitoring tools like **Prometheus**, **Grafana**, and **Logrotate**.

---

## 🔒 Security Best Practices

| Practice                     | Benefit                                       |
| ---------------------------- | --------------------------------------------- |
| Use rootless containers      | Limit system-wide impact                      |
| Use volume mounts carefully  | Avoid leaking host filesystems                |
| Limit container capabilities | Drop unnecessary privileges with `--cap-drop` |
| Use SELinux or AppArmor      | Enforce policy-based isolation                |
{:class="table table-striped"}

> 🔐 Podman is designed with **security in mind**, so you can run containers with minimal trust assumptions.

---

## 🧪 CI/CD and Automation

Podman can be easily integrated into:

- **CI pipelines** (e.g., GitHub Actions, GitLab CI, Jenkins)
- **Ansible** and other automation tools
- **Bash scripts** or `Makefile`-based workflows

Rootless mode makes Podman easier to run in **restricted CI environments** with no elevated privileges.

---

## ☁️ Where People Use Podman

- **On-prem Linux servers** running systemd services
- **Cloud VMs** where root access is restricted
- **Edge devices** using containerized services (IoT)
- **Developer laptops** testing services before Kubernetes deployment

---

## 🧩 Production Considerations

- Use tagged image versions (avoid `:latest`)
- Set up log rotation and monitoring
- Regularly prune unused containers and volumes
- Consider system snapshots for rollback

---

Next up: [Troubleshooting and Debugging](11_troubleshooting_and_debugging)

---
