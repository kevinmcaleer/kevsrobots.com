---
title: Docker Swarm on Raspberry Pi 5
description: >-
    Build a small Raspberry Pi 5 cluster, turn it into a Docker Swarm, and deploy, scale, update, monitor and back up real services across it.
layout: lesson
type: page
cover: /learn/docker_swarm/assets/banners/00_intro.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

{% include youtubeplayer.html id="tDENgLiJSh0" %}

By the end of this course you will be able to build a Raspberry Pi cluster, join it into a Docker Swarm, and keep real services running on it even when a node goes dark.

---

## Overview

One Raspberry Pi is a lovely little server. Four of them, working together, are a *cluster*: if one Pi loses power, the others carry on, and you can add more Pis when you need more room. **Docker Swarm** is the tool that makes a group of Pis behave like one big Docker machine, and it is built straight into Docker Engine, so there is nothing extra to install.

This course follows the way a small home cluster is built in practice. We plan it, prepare the Pis, form the swarm, deploy a stack, then learn to scale, update, rebalance, monitor, maintain and back it up. Everything works on 64-bit Raspberry Pi OS, and every image we use is published for `arm64`, the architecture of the Pi 5.

```mermaid
flowchart LR
    A["Plan and prepare the Pis"] --> B["Form the swarm"]
    B --> C["Deploy a stack"]
    C --> D["Scale and update"]
    D --> E["Monitor and maintain"]
    E --> F["Back up and recover"]
```

---

## Course Content

- Planning a cluster: roles, network and hardware
- Preparing identical Raspberry Pi OS nodes with Docker installed
- Creating a swarm and joining workers and managers
- Verifying the cluster with a real test service
- Docker Compose files versus Swarm stacks
- Writing and deploying a stack file with the `deploy:` settings
- Scaling, rolling updates, rollbacks and rebalancing
- Monitoring, routine maintenance, backup and recovery
- A capstone project: build it, then try to break it

---

## Key Results

After completing this course, you will:

- Explain what a manager, a worker, a service and a task are
- Initialise a swarm and join Raspberry Pi nodes to it
- Write a stack file and deploy it with `docker stack deploy`
- Scale a service, roll out an update and roll it back
- Drain a node, rebalance services and recover from a failed node
- Back up the swarm state and your data volumes

---

## What you'll need

- Two to four Raspberry Pi 5 boards (three is the sweet spot), each with a good power supply
- A microSD card per Pi, 32 GB or larger, or an NVMe or USB SSD if you have one
- An Ethernet switch and cables (Ethernet is much steadier than Wi-Fi for a cluster)
- A computer on the same network to SSH from, and the free Raspberry Pi Imager
- 64-bit Raspberry Pi OS (Lite is fine) on every Pi

No cluster yet? A single machine running Docker can still do the commands in lessons 3, 6, 7, 8, 9 and 10: `docker swarm init` works on one node. You just will not see work spread across several machines.

> **Teachers**: This course suits students aged 14 and over who are comfortable in a terminal. A cluster of three Pis per group is plenty. Each lesson takes 30 to 45 minutes, and the build lessons (2 to 5) take about an hour. With no hardware at all, learners can do the stack-file lessons on one laptop with Docker, or read the concepts and answer the quizzes. Check that your school network allows devices to talk to each other on the ports listed in lesson 1, and consider a small standalone switch so the cluster never touches the school network.

---

## Prerequisites

- The [Docker course](/learn/docker/) or equivalent: you should know images, containers and a basic Compose file
- Basic Linux command line skills, for example from [Introduction to the Linux Command Line on Raspberry Pi OS](/learn/linux_intro/)
- A rough idea of IP addresses and ports

---

## How the course works

Each lesson opens with a one-line objective, teaches one idea, then gives you things to try and a short quiz. Commands you type are in code blocks. Where a command must run on a particular machine we say so: **on the manager**, **on a worker**, or **on your computer**. Anything in angle brackets such as `<MANAGER_IP>` is a placeholder for you to replace.

Look out for **Common Issues** boxes, which list the problems people actually hit, and for **Teachers** notes where a classroom needs extra thought.

Swarm commands in this course were checked against a recent Docker Engine. Docker evolves, so if a flag has moved, `docker <command> --help` is always right.

Ready? Let's build a cluster.
