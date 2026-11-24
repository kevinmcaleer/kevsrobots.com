---
layout: lesson
title: Cluster Maintenance
author: Kevin McAleer
type: page
cover: /learn/docker_swarm/assets/banners/11_cluster-maintenance.jpg
date: 2024-02-03
previous: 10_monitoring-tools.html
next: 12_backup-and-recovery.html
description: Keep a Raspberry Pi swarm healthy with a safe rolling OS update routine,
  careful clean-up, log limits, security basics and a node-replacement drill.
percent: 78
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

By the end of this lesson you will be able to update the OS on each node without taking your services offline, clean up disk space safely, and replace a failed node.

---

## The golden rule: one node at a time

The whole point of a cluster is that it can lose a node and carry on. So update, reboot or repair **one node at a time**, and wait until the cluster is healthy before touching the next.

### A rolling OS update

For each node in turn, starting with workers:

```mermaid
flowchart LR
    A["Drain the node"] --> B["apt update and upgrade"]
    B --> C["Reboot"]
    C --> D["Wait: node Ready"]
    D --> E["Set node active"]
    E --> F["Check services 3/3"]
    F --> G["Next node"]
```

From a manager, drain the node:

```bash
docker node update --availability drain pi-node4
```

Then on that Pi itself:

```bash
sudo apt update
sudo apt full-upgrade -y
sudo reboot
```

Back on the manager, once `docker node ls` shows the node `Ready`:

```bash
docker node update --availability active pi-node4
docker service ls
```

Wait for every service to show its full replica count before doing the next node. Do managers last, and never reboot two managers at once in a three-manager swarm, or you lose quorum.

Docker Engine updates arrive through the same `apt` repository the install script set up, so a full upgrade keeps Docker current too. Try to keep all nodes on the same Docker version.

---

## Clean up, carefully

Docker slowly fills the SD card with old images and stopped containers. Look first, then clean:

```bash
docker system df
docker image prune
```

`docker image prune` removes dangling (untagged) images. `docker system prune` removes more: stopped containers, unused networks and dangling images, and adding `-a` also removes every image not used by a running container, which means the nodes must pull them again. These commands only act on the node where you run them, so run them on each Pi in turn. Never add `--volumes` unless you are sure: volumes hold your data.

---

## Stop the logs filling the disk

By default each container's log can grow without limit. Cap it on every node by creating `/etc/docker/daemon.json` (with `sudo`):

```json
{
  "log-driver": "json-file",
  "log-opts": {
    "max-size": "10m",
    "max-file": "3"
  }
}
```

Then run `sudo systemctl restart docker`. This keeps at most three 10 MB files per container. It applies to containers created afterwards, so redeploy your services. If you already have an `insecure-registries` entry in that file, merge the two rather than replacing the file.

---

## Security basics

- **Never expose the Docker socket over the network** (port 2375) without TLS. Anyone who can reach it controls the machine.
- **Limit who can reach the cluster ports** (2377, 7946, 4789) to your cluster's own network.
- **Rotate join tokens** with `docker swarm join-token --rotate worker` if one leaks.
- **Autolock the managers**, so the swarm's encryption keys are not readable from a stolen SD card:

```bash
docker swarm update --autolock=true
```

Docker prints an **unlock key**. Store it somewhere safe away from the Pis (a password manager). After a manager restarts it stays locked until you run `docker swarm unlock` and enter that key. Lose the key and that manager cannot rejoin.

> **Teachers**: Autolock is a good talking point but makes a classroom cluster fiddly, because every reboot needs a key. Use it as a demonstration on one group's cluster, not by default. Also agree rules for who may power off a shared Pi.

---

## Replacing a failed node

A node dies. Don't panic: its tasks have already moved. To replace it:

1. On a manager, `docker node ls` shows it `Down`.
2. Remove it: `docker node rm --force pi-node4` (a down node needs `--force`, or first `docker node demote` if it was a manager).
3. Flash a new card with the same hostname, install Docker, and join with the right token. This is why the preparation lesson matters.
4. Run `docker service update --force` on busy services to rebalance onto the new node.

---

## Try it Yourself

1. Write down your own maintenance routine as a checklist: what you check weekly, and what monthly.
2. Run `docker system df` on every node and find which has the most reclaimable space.
3. Add the log limits above to one node, restart Docker, redeploy a service on it and confirm with `docker inspect` on a container that the log options are set.
4. Challenge: do a full rolling update of your cluster, timing how long the services were below their replica count.

---

## Common Issues

- **Problem**: After a reboot, a manager shows `Unreachable`.
- **Solution**: Wait a minute for Docker to start. If autolock is on, run `docker swarm unlock` on that node.
- **Why**: A locked manager cannot read its keys until it is unlocked.

- **Problem**: The disk is full and containers stop starting.
- **Solution**: Run `docker system df`, then `docker image prune`, and cap logs as shown above.
- **Why**: Unbounded logs and old images are the usual culprits on small SD cards.

- **Problem**: The daemon will not start after editing `daemon.json`.
- **Solution**: Check the file is valid JSON (`python3 -m json.tool /etc/docker/daemon.json`) and read `journalctl -u docker`.
- **Why**: One misplaced comma stops the Docker daemon from reading its settings.

---

## Check your understanding

{% quiz %}
? Why drain a node before updating it?
- [ ] Draining speeds up `apt`
- [x] Its tasks move elsewhere first, so your services stay available
- [ ] A node cannot update while it is active
> Draining moves work off the node so a reboot does not interrupt your services.

? What does adding `--volumes` to a prune command risk?
- [ ] Nothing
- [x] Deleting unused volumes, which may hold your data
- [ ] Removing the swarm
> Volumes hold persistent data, so only delete those you are sure you don't need.

? What does autolock protect against?
- [x] Someone reading the swarm's secrets from a stolen manager's disk
- [ ] A worker failing
- [ ] Slow networks
> Autolock encrypts the keys at rest, and the unlock key is needed after a restart.
{% endquiz %}
