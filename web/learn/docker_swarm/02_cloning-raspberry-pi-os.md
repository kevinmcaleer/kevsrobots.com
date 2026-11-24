---
layout: lesson
title: Cloning Raspberry Pi OS
author: Kevin McAleer
type: page
cover: /learn/docker_swarm/assets/banners/02_cloning-raspberry-pi-os.jpg
date: 2024-02-03
previous: 01_planning-the-cluster.html
next: 03_initializing-docker-swarm.html
description: 'Prepare several identical Raspberry Pi nodes: flash or clone the OS,
  give each a unique name and fixed address, and install Docker.'
percent: 18
duration: 6
quiz_total: 45
date_updated: 2026-11-24
navigation:
- name: Raspberry Pi 5 Cluster with Docker Swarm
- content:
  - section: Introduction
    content:
    - name: Docker Swarm on Raspberry Pi 5
      link: 00_intro.html
  - section: Building the Cluster
    content:
    - name: Planning the Cluster
      link: 01_planning-the-cluster.html
    - name: Cloning Raspberry Pi OS
      link: 02_cloning-raspberry-pi-os.html
    - name: Initializing Docker Swarm
      link: 03_initializing-docker-swarm.html
    - name: Adding Worker Nodes to the Swarm
      link: 04_adding-worker-nodes.html
    - name: Verifying Cluster Setup
      link: 05_verifying-cluster-setup.html
  - section: Deploying Applications
    content:
    - name: Docker Compose and Swarm Stacks
      link: 06_docker-compose-and-swarm-stacks.html
    - name: Writing a Docker Compose File
      link: 07_writing-a-docker-compose-file.html
    - name: Deploying a Stack
      link: 08_deploying-a-stack.html
    - name: Managing and Scaling Services
      link: 09_managing-and-scaling-services.html
    - name: Rebalancing Services in Docker Swarm
      link: 09_rebalancing.html
  - section: Monitoring and Maintenance
    content:
    - name: Monitoring Tools for Docker Swarm
      link: 10_monitoring-tools.html
    - name: Cluster Maintenance
      link: 11_cluster-maintenance.html
    - name: Backup and Recovery Strategies
      link: 12_backup-and-recovery.html
  - section: Conclusion
    content:
    - name: Project Ideas
      link: 13_project-ideas.html
    - name: Summary and Next Steps
      link: 14_summary-and-next-steps.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to prepare several identical Raspberry Pi nodes, each with its own hostname and fixed IP address, and with Docker installed and ready.

---

## The goal: identical, but not the same

Every node in a swarm should run the same OS and the same Docker version, so a container behaves the same wherever it lands. But each node must still be *unique*: its own hostname, its own IP address, and (once Docker is set up) its own swarm identity.

```mermaid
flowchart LR
    A["Flash 64-bit Pi OS"] --> B["Set hostname and SSH"]
    B --> C["Fixed IP address"]
    C --> D["Install Docker"]
    D --> E["Ready to join a swarm"]
```

There are two ways to get there. The first is the simplest and the one I recommend.

---

## Method 1: flash each card with Raspberry Pi Imager

Raspberry Pi Imager lets you pre-set a hostname, a username, SSH access and Wi-Fi *before* writing the card. That means each card comes out unique with no clean-up afterwards.

For each Pi:

1. Choose **Raspberry Pi OS (64-bit)** or the **Lite** version. Most images publish `arm64` builds for the Pi 5, and a 64-bit OS lets you run them.
2. Open the OS customisation settings and set the hostname (`pi-node1`, `pi-node2`, and so on), your username and password, and enable SSH.
3. Write the card, put it in the Pi and boot it.

That is all the "cloning" most clusters need, and it takes a couple of minutes per card.

---

## Method 2: clone a finished card

If you want to set a Pi up once (with all your tools installed) and copy it, you can image the card. The catch is that a copy contains everything, including things that must be unique.

On a Linux computer, with the source card in a reader (check the device name with `lsblk` first, because writing to the wrong device destroys data):

```bash
sudo dd if=/dev/sdX of=pi-node-base.img bs=4M status=progress
sudo dd if=pi-node-base.img of=/dev/sdY bs=4M status=progress conv=fsync
```

Replace `sdX` with the source card and `sdY` with the blank target. Raspberry Pi Imager does not copy one card to another, but it can write the saved `.img` file if you pick **Use custom**.

Clone **before** you run `docker swarm init` or `docker swarm join`. A swarm node stores its identity in `/var/lib/docker/swarm`, and two cards with the same identity will confuse the cluster. After cloning, on each new Pi you must:

- change the hostname (`sudo raspi-config`, then System Options, then Hostname) and reboot;
- regenerate SSH host keys if you copied them, so SSH does not warn about changed or duplicated keys.

If that sounds fiddly, it is. Method 1 avoids all of it.

---

## Give every node a fixed IP address

Swarm finds nodes by address, so the address must not change. The easiest fix is a **DHCP reservation** in your router: tell it to always give a particular IP to a particular Pi. Otherwise, set a static address on the Pi. Raspberry Pi OS Bookworm uses NetworkManager, so something like this works (find your connection name first):

```bash
nmcli connection show
sudo nmcli connection modify "Wired connection 1" \
  ipv4.method manual \
  ipv4.addresses 192.168.50.11/24 \
  ipv4.gateway 192.168.50.1 \
  ipv4.dns 192.168.50.1
sudo nmcli connection up "Wired connection 1"
```

Use your own network's addresses, and take care: a typo here can lock you out until you plug in a monitor.

---

## Install Docker on every node

Docker publishes an official convenience script. It is good practice to read a script before running it with `sudo`:

```bash
curl -fsSL https://get.docker.com -o get-docker.sh
less get-docker.sh
sudo sh get-docker.sh
```

Then let your user run Docker without `sudo`, and check it works:

```bash
sudo usermod -aG docker $USER
```

Log out and back in, then:

```bash
docker version
docker compose version
```

`docker compose` (two words) is the current Compose, installed as a Docker plugin. The older standalone `docker-compose` command is retired, so we will not use it. Docker Swarm itself needs no installation: it is already inside Docker Engine.

> **Teachers**: Flashing cards is the slow, queue-forming step in a class. Prepare the cards before the lesson, one per learner group, and label each Pi and card with its hostname. Shared microSD cards get mixed up easily, so a coloured sticker per node helps.

---

## Try it Yourself

1. Flash one card with Method 1 and confirm you can `ssh` to it by hostname, for example `ssh pi@pi-node1.local`.
2. Run `hostname -I` on each Pi and write the addresses in your cluster plan from lesson 1.
3. Challenge: run `docker run --rm hello-world` on every node and compare `docker version` output. Are all nodes on the same version?

---

## Common Issues

- **Problem**: `permission denied while trying to connect to the Docker daemon socket`.
- **Solution**: Run `sudo usermod -aG docker $USER`, then log out and back in.
- **Why**: Your session picked up your group membership when you logged in, so the change only applies to new sessions.

- **Problem**: Two Pis end up with the same hostname and one becomes unreachable by name.
- **Solution**: Give each a unique name, and reboot after changing it.
- **Why**: The `.local` name is announced over mDNS, and duplicates clash.

---

## Check your understanding

{% quiz %}
? Why should you clone a card before running `docker swarm init`?
- [ ] Docker will not run on a cloned card
- [x] A joined node stores its swarm identity on disk, and clones would share it
- [ ] The swarm only accepts brand new cards
> Swarm identity lives in `/var/lib/docker/swarm`. Cloning after joining copies that identity to every card.

? Which command do we use for Compose in this course?
- [ ] `docker-compose`
- [x] `docker compose`
- [ ] `compose up`
> The standalone `docker-compose` is retired. `docker compose` is the plugin that ships with current Docker.

? Why does each node need a fixed IP address?
- [x] Swarm finds nodes by address, so a changing address can drop a node
- [ ] Docker only works with static addresses
- [ ] Static addresses make the Pi faster
> Managers and workers talk to each other by address. A reservation or a static address keeps that stable.
{% endquiz %}
