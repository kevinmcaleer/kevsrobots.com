---
layout: lesson
title: Adding Worker Nodes to the Swarm
author: Kevin McAleer
type: page
cover: /learn/docker_swarm/assets/banners/04_adding-worker-nodes.jpg
date: 2024-02-03
previous: 03_initializing-docker-swarm.html
next: 05_verifying-cluster-setup.html
description: Join more Raspberry Pis to your swarm as workers and managers, promote
  and demote nodes, and remove nodes cleanly.
percent: 30
duration: 4
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

By the end of this lesson you will be able to join Raspberry Pis to a swarm, change a node's role between worker and manager, and remove a node safely.

---

## Joining a worker

We have one manager, `pi-node1`. Let's give it company. First, on the manager, print the worker join command:

```bash
docker swarm join-token worker
```

It prints a ready-made command. Copy it, SSH into your second Pi, and paste it:

```bash
docker swarm join --token SWMTKN-1-<token_string> 192.168.50.11:2377
```

You should see `This node joined a swarm as a worker.` Repeat on each Pi you want to add. The screenshot shows what the token command looks like on a real manager.

![Docker Swarm Join Token](/learn/docker_swarm/assets/join.png){:class="img-fluid w-100 shadow-lg rounded"}

---

## Checking who has joined

Worker nodes cannot run swarm commands, only managers can. Back on the manager:

```bash
docker node ls
```

Each node shows its hostname, STATUS (`Ready` is good, `Down` is not), AVAILABILITY (`Active`) and MANAGER STATUS. Workers have a blank manager status; the leader shows `Leader`.

---

## Making a three-manager cluster

Remember lesson 1: three managers survive one failure. To make a node a manager, you have two choices.

**Join as a manager from the start.** On a manager, get the manager token and join with that:

```bash
docker swarm join-token manager
```

**Or promote an existing worker.** On a manager:

```bash
docker node promote pi-node2
docker node promote pi-node3
```

Run `docker node ls` again. The promoted nodes now show `Reachable` under MANAGER STATUS, and `pi-node1` is still the `Leader`. If you later need to reduce the number of managers, `docker node demote pi-node3` turns a manager back into a worker.

```mermaid
flowchart LR
    W["Worker"] -- "docker node promote" --> M["Manager (reachable)"]
    M -- "docker node demote" --> W
```

---

## Leaving and removing nodes

To take a node out cleanly:

1. On the node itself, run `docker swarm leave`. Its status on the manager becomes `Down`.
2. On a manager, run `docker node rm pi-node4` to delete the entry.

A manager must be demoted before it can leave, or you must add `--force`, which risks losing quorum. Demote first.

Joining and leaving are cheap, so do not be afraid of experimenting. A node that left can always rejoin with the token.

---

## Try it Yourself

1. Join your second Pi as a worker, then run `docker node ls` and confirm it says `Ready` and `Active`.
2. Promote it to a manager, then demote it again. Watch the MANAGER STATUS column change.
3. Challenge: run `docker node ls` on a *worker*. What error do you get, and why?

---

## Common Issues

- **Problem**: The join command hangs or says `connection refused` or `timeout`.
- **Solution**: Check the manager's address, and that port 2377/tcp is reachable from the worker, for example with `nc -zv 192.168.50.11 2377`.
- **Why**: Joining needs a direct network path to the manager's management port.

- **Problem**: `Error response from daemon: This node is not a swarm manager`.
- **Solution**: Run swarm commands such as `docker node ls` on a manager, not a worker.
- **Why**: Only managers hold the cluster state and can change it.

- **Problem**: The node joins but shows as `Down` soon afterwards.
- **Solution**: Open ports 7946 (tcp and udp) and 4789/udp between nodes, and check the node's time with `timedatectl`.
- **Why**: Nodes need gossip and overlay traffic, and certificates are checked against the clock.

---

## Check your understanding

{% quiz %}
? Where do you run `docker node ls`?
- [ ] On any node in the swarm
- [x] On a manager node only
- [ ] On your own computer without Docker
> Worker nodes do not hold the cluster state, so swarm management commands only work on managers.

? How do you turn an existing worker into a manager?
- [x] `docker node promote <node>`, run on a manager
- [ ] Re-flash its card
- [ ] `docker swarm init` on the worker
> `docker node promote` changes a worker's role without moving anything.

? What should you do before a manager leaves the swarm?
- [ ] Nothing, it can just leave
- [x] Demote it to a worker first
- [ ] Delete `/var/lib/docker`
> Demoting first keeps the managers in agreement. Forcing a manager out risks losing quorum.
{% endquiz %}
