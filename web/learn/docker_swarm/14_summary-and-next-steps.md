---
layout: lesson
title: Summary and Next Steps
author: Kevin McAleer
type: page
cover: /learn/docker_swarm/assets/banners/14_summary-and-next-steps.jpg
date: 2024-02-03
previous: 13_project-ideas.html
description: Recap what you built with Docker Swarm on Raspberry Pi, see where Swarm
  fits against other tools, and find your next courses and trusted resources.
percent: 100
duration: 4
quiz_total: 45
date_updated: 2026-11-24
redirect_from: /learn/docker_swarm/14_further-resources.html
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

By the end of this lesson you will be able to summarise how a Docker Swarm cluster fits together, decide when Swarm is the right tool, and choose what to learn next.

---

## What you built

Congratulations, you went from a pile of Pis to a self-healing cluster. Here is the whole journey in one picture:

```mermaid
flowchart LR
    A["Plan: managers, workers, ports"] --> B["Prepare: OS, IPs, Docker"]
    B --> C["docker swarm init and join"]
    C --> D["Stack file: deploy key"]
    D --> E["Deploy, scale, update, roll back"]
    E --> F["Rebalance, monitor, maintain"]
    F --> G["Back up and recover"]
```

The ideas to keep:

- A **swarm** is a group of Docker Engines. **Managers** keep the state (using Raft, so use an odd number) and **workers** run tasks.
- You describe what you want in a **stack file**, and Swarm keeps reality matching it. That is the declarative idea.
- A **service** is the definition; **tasks** are the running containers. Swarm replaces failed tasks by itself.
- A stack ignores `build:` and `depends_on:`, so push images to a registry and make apps patient.
- Swarm does not rebalance for you. Use `docker service update --force` when a node returns.
- Named volumes live on one node. Plan for that, and back them up.
- Swarm has no backup command. Keep stack files in Git, archive your volumes and copy `/var/lib/docker/swarm` when needed.

---

## When is Swarm the right choice?

Swarm is small, simple and built into Docker, which makes it a good fit for a home or classroom Pi cluster. Kubernetes is far more widespread in industry and offers more features, at the cost of much more to learn and run. A lightweight Kubernetes called K3s is designed for exactly this Pi territory, and it is worth trying once you are comfortable here. For your own projects, the honest answer is: use whichever you will actually maintain.

---

## Where to go next

Courses on this site that pair well with this one:

- [Running K3s on Raspberry Pi](/learn/k3s/) to compare Swarm with a lightweight Kubernetes
- [Docker](/learn/docker/) if you want to revisit images, networking and volumes
- [From Docker to Podman](/learn/podman/) for a daemonless alternative to Docker
- [Introduction to the Linux Command Line on Raspberry Pi OS](/learn/linux_intro/) to get more comfortable on your nodes

---

## Trusted resources

- The official Docker documentation: [docs.docker.com](https://docs.docker.com/), especially [Swarm mode](https://docs.docker.com/engine/swarm/)
- [Play with Docker Classroom](https://training.play-with-docker.com/): hands-on labs in your browser, handy for practising Swarm without Pis
- [Portainer](https://www.portainer.io/): a friendly web UI for Swarm
- The book *Docker Deep Dive* by Nigel Poulton covers Swarm among other topics
- The [Docker forums](https://forums.docker.com/) for questions

---

## Try it Yourself

1. Without looking back, draw your cluster: nodes, roles, services and where the data lives. Compare it with lesson 1.
2. Write a one-page recovery plan for "my leader Pi has died": what you check, and in what order.
3. Challenge: pick one idea from the project list, and build it as a stack.

---

## Common Issues

- **Problem**: You are not sure if a problem is in your stack file or in the cluster.
- **Solution**: Run `docker stack config -c stack.yml` for the file, then `docker node ls` and `docker service ps --no-trunc <service>` for the cluster.
- **Why**: This splits the problem in two: is the file read correctly, and can the cluster run it?

- **Problem**: A command from an old tutorial does not exist.
- **Solution**: Run `docker <command> --help`, and check the official docs for your Docker version.
- **Why**: Docker changes over time, and older guides mention retired commands such as `docker-compose` or invented ones such as `docker swarm backup`.

---

## Check your understanding

{% quiz %}
? Which statement about managers is correct?
- [ ] Use an even number so they can vote evenly
- [x] Use an odd number, because Raft needs a majority
- [ ] Managers cannot run tasks
> An odd number of managers gives the best failure tolerance for the number of nodes.

? What should you do before deploying your own image to a swarm?
- [ ] Build it on the manager only
- [x] Push it to a registry every node can reach
- [ ] Nothing, Swarm copies it to each node
> Every node pulls its own copy, so a registry is needed.

? Why keep your stack files in Git?
- [x] You can redeploy the whole stack even if the swarm is lost
- [ ] Docker requires it
- [ ] It makes the Pis faster
> The stack file is the source of truth, so versioning it protects your setup.
{% endquiz %}
