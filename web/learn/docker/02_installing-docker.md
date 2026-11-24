---
layout: lesson
title: Installing Docker
author: Kevin McAleer
type: page
cover: /learn/docker/assets/banners/02_installing-docker.jpg
date: 2024-01-21
previous: 01_what-is-docker.html
next: 03_basics.html
description: Install Docker Engine and the Compose plugin on Raspberry Pi OS (64-bit)
  or a Debian-based Linux PC, and run your first container.
percent: 27
duration: 5
quiz_total: 30
date_updated: 2026-11-24
navigation:
- name: Docker
- content:
  - section: Overview
    content:
    - name: Overview
      link: 00_intro.html
    - name: Introduction to Docker
      link: 01_what-is-docker.html
    - name: Installing Docker
      link: 02_installing-docker.html
  - section: Images and Containers
    content:
    - name: Docker Basics
      link: 03_basics.html
    - name: Building Your First Docker Image
      link: 04_building.html
    - name: Managing Docker Containers
      link: 05_managing.html
    - name: Docker Networking Basics
      link: 06_networking.html
    - name: Docker Volumes and Persistent Data
      link: 07_volumes.html
  - section: Docker Compose
    content:
    - name: Creating a Docker Compose File
      link: 08_docker-compose.html
    - name: 'Project: The Robot Status Stack'
      link: 09_dockerise.html
    - name: Conclusion and Next Steps
      link: 10_conclusion.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to install Docker Engine on a Raspberry Pi or Linux PC, run it without `sudo`, and prove it works with `hello-world`.

Everything in this lesson is typed in a terminal **on the Pi or Linux PC** that will run your containers.

## Before you start

Check that you are running a 64-bit system:

```bash
uname -m
```

On a Raspberry Pi 4 or 5 with the 64-bit Raspberry Pi OS you'll see `aarch64`. On a normal PC you'll see `x86_64`. Either is fine.

```mermaid
flowchart LR
    A["Add Docker's apt repository"] --> B["Install Engine + Compose plugin"]
    B --> C["Add yourself to the docker group"]
    C --> D["Log out and in"]
    D --> E["hello-world"]
```

## Option 1: Docker's apt repository (recommended)

This is the method Docker documents for Debian, which is what Raspberry Pi OS is built on. It means `sudo apt upgrade` keeps Docker up to date for you. Type these in order:

```bash
sudo apt update
sudo apt install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
```

Those lines fetch Docker's signing key, so your Pi can check that packages really come from Docker. Now add the repository itself:

```bash
sudo tee /etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/debian
Suites: $(. /etc/os-release && echo "$VERSION_CODENAME")
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF
```

The `$(...)` parts are filled in for you with your Debian release name (such as `bookworm`) and your CPU type. Finally install:

```bash
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

> If you are on Ubuntu or another distribution, the repository address and steps differ. Pick your system on [docs.docker.com/engine/install](https://docs.docker.com/engine/install/).

## Option 2: the convenience script

Docker also publishes a script that detects your system and does the above for you. It is quicker for a throwaway Pi, though Docker itself recommends the repository method for anything long-lived. It is good habit to read a script before running it as root:

```bash
curl -fsSL https://get.docker.com -o get-docker.sh
less get-docker.sh
sudo sh get-docker.sh
```

Press `q` to leave `less`. Use **either** option 1 or option 2, not both.

## Run Docker without sudo

By default only root can talk to Docker. Add your user to the `docker` group:

```bash
sudo usermod -aG docker $USER
```

Group changes only apply to new logins, so log out and back in (or reboot). Then check:

```bash
groups
```

You should see `docker` in the list.

## Test it

```bash
docker run hello-world
```

Docker downloads a tiny image and runs it. Look for "Hello from Docker!" in the output. Now check the Compose plugin, noting the **space** between `docker` and `compose`:

```bash
docker --version
docker compose version
```

Both should print a version number. If so, you're ready.

> Windows and macOS users can install Docker Desktop from [docs.docker.com/desktop](https://docs.docker.com/desktop/), which includes Compose. This course is written and checked for Linux and the Raspberry Pi, so the install steps above are the ones we have covered.

## Common Issues

- **Problem**: `permission denied while trying to connect to the Docker daemon socket`.
- **Solution**: Make sure you ran `sudo usermod -aG docker $USER`, then log out and back in. In a hurry, prefix the command with `sudo`.
- **Why**: Group membership is read at login, so the change does not reach your current terminal.

- **Problem**: `docker: command not found`.
- **Solution**: The install did not finish. Re-run the `apt install` line and read any error messages.
- **Why**: The `docker` program only exists once the `docker-ce-cli` package is installed.

- **Problem**: Old tutorials tell you to install Compose with pip, or to run `sudo chmod 666 /var/run/docker.sock`.
- **Solution**: Don't. Compose arrives with the `docker-compose-plugin` package, and the group step above is the safe way to get access.
- **Why**: The old pip package is the retired version 1 of Compose, and opening the socket to everyone lets any program on the machine control Docker.

> **Teachers**: Installing needs admin (`sudo`) rights, and members of the `docker` group effectively have root on the machine, so only add trusted learners. On a shared Pi, set it up once for everyone. Downloads come from `download.docker.com` and Docker Hub, which some school firewalls block, so test first. Installing takes roughly 5 to 10 minutes per Pi on a decent connection.

## Try it Yourself

1. Run `docker run hello-world` a second time. Is it faster? Why do you think that is?
2. Run `docker info` and find the line that tells you which version of Docker Engine you have.
3. Challenge: run `sudo systemctl status docker` and work out what "active (running)" is telling you. Press `q` to quit.

## Check your understanding

{% quiz %}
? Which command checks that the Compose plugin is installed?
- [ ] `compose --version`
- [x] `docker compose version`
- [ ] `pip show compose`
> Compose is now a Docker plugin, run as `docker compose` with a space. The old hyphenated form belongs to the retired version 1.

? Why do you log out and back in after `usermod -aG docker $USER`?
- [ ] To download the Docker images
- [x] Because group membership is only read at login
- [ ] To restart the Docker service
> Your current session still has your old groups. A new login picks up the new `docker` group.

? What does `docker run hello-world` prove?
- [x] Docker can pull an image from the internet and run a container
- [ ] Your robot is working
- [ ] Compose is installed
> It downloads the small `hello-world` image from Docker Hub and runs it, which exercises the whole chain.
{% endquiz %}
