---
layout: lesson
title: Podman in Production
author: Kevin McAleer
type: page
cover: /learn/podman/assets/banners/10_podman_in_production.jpg
date: 2025-05-24
previous: 09_podman_and_kubernetes.html
next: 11_troubleshooting_and_debugging.html
description: Run the robot status app as a systemd service with a Quadlet file so
  it restarts on failure and starts at boot, and harden it with a few simple security
  options.
percent: 77
duration: 5
quiz_total: 47
date_updated: 2026-11-24
navigation:
- name: From Docker to Podman
- content:
  - section: Getting Started
    content:
    - name: Introduction to Podman
      link: 00_intro.html
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
    - name: Podman Features Docker Doesn't Have
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


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to write a Quadlet file that runs a container as a systemd service, make it start at boot, and read its logs.

A robot is not much use if its dashboard vanishes whenever the Pi reboots. Docker has a daemon that restarts containers. Podman has no daemon, so we use the one that already runs every Linux service: **systemd**.

## Quadlet: containers as systemd services

You write a short file describing the container, and a systemd helper called **Quadlet** turns it into a service. This replaces the older `podman generate systemd` command, which is now deprecated. Quadlet needs Podman **4.4 or newer**; check with `podman --version`.

Your user's Quadlet files live in `~/.config/containers/systemd/`.

```mermaid
flowchart LR
    A["robot-web.container<br/>(you write this)"] -- "systemctl --user daemon-reload" --> B["Quadlet generates<br/>robot-web.service"]
    B -- "systemd starts it" --> C["podman run ..."]
    C --> D["Container running<br/>restarts on failure"]
```

## Step 1: Write the file

Make sure you built the `robot-status` image as your user (lesson 6), then:

```bash
mkdir -p ~/.config/containers/systemd
nano ~/.config/containers/systemd/robot-web.container
```

```ini
[Unit]
Description=Robot status web app

[Container]
Image=localhost/robot-status:latest
ContainerName=robot-web
PublishPort=5000:5000
Environment=ROBOT_NAME=Sparky

[Service]
Restart=always

[Install]
WantedBy=default.target
```
{: .no-snakie }

The `[Container]` section holds the same things you gave `podman run`. The `[Service]` section is plain systemd and says to restart on failure. The `[Install]` section is what makes it start automatically, so there is no `systemctl enable` step for Quadlet services.

## Step 2: Start it

```bash
systemctl --user daemon-reload
systemctl --user start robot-web.service
systemctl --user status robot-web.service
```

The service is named after the file. Open `http://localhost:5000` to see it working.

## Step 3: Start at boot

User services normally start when you log in and stop when you log out. To keep them running on a Pi that nobody logs in to, enable **lingering**:

```bash
sudo loginctl enable-linger $USER
```

Reboot the Pi with `sudo reboot`, wait a minute, and visit the page again. It should be back with no help from you.

## Logs and monitoring

Containers run as normal processes, so the usual tools work:

```bash
journalctl --user -u robot-web.service -f
podman logs robot-web
podman stats --no-stream
```

If your Podman is older than 4.4, the old route still works. Create a container first (`podman create --name robot ...`), then:

```bash
podman generate systemd --name robot --files --restart-policy=always
mkdir -p ~/.config/systemd/user
mv container-robot.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now container-robot.service
```
{: .no-snakie }

## Harden it

Adding a few options costs nothing. In a Quadlet file they appear as extra lines under `[Container]`:

| Goal | Quadlet line | Same as `podman run` |
| --- | --- | --- |
| Drop all extra Linux privileges | `DropCapability=all` | `--cap-drop=all` |
| Stop gaining privileges | `NoNewPrivileges=true` | `--security-opt no-new-privileges` |
| Read-only filesystem | `ReadOnly=true` | `--read-only` |
{:class="table table-single"}

Try each one and check the app still runs. If something breaks, the logs will tell you what it needed. Also use **pinned image tags** such as `redis:7-alpine` rather than `latest`, so a surprise update cannot change your robot overnight, and keep passwords out of files by using `podman secret create` rather than writing them in plain text.

## Try it Yourself

1. Stop the service with `systemctl --user stop robot-web.service` and confirm the page goes away. Start it again
2. Kill the container's process with `podman kill robot-web` and watch systemd bring it back with `systemctl --user status robot-web.service`
3. Challenge: add `Environment=ROBOT_NAME=Twinkle` for a different name, run `daemon-reload` and `restart`, and confirm the page changed

## Common Issues

- **Problem**: `Unit robot-web.service not found`.
- **Solution**: Check the file is in `~/.config/containers/systemd/`, ends in `.container`, then run `systemctl --user daemon-reload`. Also check `podman --version` is 4.4 or newer.
- **Why**: Quadlet only generates services from files in the right folder, and only exists in newer Podman.

- **Problem**: The service starts when you log in but not after a reboot.
- **Solution**: Run `sudo loginctl enable-linger $USER`.
- **Why**: Without lingering, your user's services only run while you are logged in.

- **Problem**: The service fails and the logs say the image was not found.
- **Solution**: Run `podman images` as your own user, and rebuild with `podman build -t robot-status .` if it is missing.
- **Why**: The service runs as you, so it only sees your images, not root's.

> **Teachers**: Enabling lingering needs `sudo`, and it keeps a learner's services running after they log out. On shared Pis, agree who owns which port and stop services at the end of the session so the next group starts clean. Rebooting a shared Pi interrupts other users, so do the reboot test only on a Pi each learner owns.

## Check your understanding

{% quiz %}
? Where do rootless Quadlet files go?
- [ ] `/etc/podman/`
- [x] `~/.config/containers/systemd/`
- [ ] `~/.local/bin/`
> That folder is where the user's Quadlet generator looks.

? What makes a user service keep running after you log out?
- [ ] `systemctl --user disable`
- [x] `loginctl enable-linger`
- [ ] `podman machine start`
> Lingering lets your user's services run without a login session.

? What replaced the deprecated `podman generate systemd`?
- [ ] `podman compose`
- [x] Quadlet files
- [ ] `podman run --restart`
> Quadlet is the current way to run containers as services.

? What does `Restart=always` in the `[Service]` section do?
- [ ] Reboots the Pi
- [x] Tells systemd to restart the service whenever it stops
- [ ] Pulls a new image
> It is a normal systemd setting, applied to the container's service.
{% endquiz %}
