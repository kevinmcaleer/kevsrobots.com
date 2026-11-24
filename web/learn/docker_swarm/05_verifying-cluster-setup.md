---
layout: lesson
title: Verifying Cluster Setup
author: Kevin McAleer
type: page
cover: /learn/docker_swarm/assets/banners/05_verifying-cluster-setup.jpg
date: 2024-02-03
previous: 04_adding-worker-nodes.html
next: 06_docker-compose-and-swarm-stacks.html
description: Check that every node is healthy, then prove the cluster works by running
  a real test service and reaching it through any node.
percent: 36
duration: 5
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

By the end of this lesson you will be able to check a swarm's health with `docker node ls` and `docker node inspect`, and prove it works by running a test service and reaching it through any node.

---

## Step 1: are all the nodes there?

On a manager:

```bash
docker node ls
```

For a healthy three-node cluster, expect output along these lines:

```plaintext
ID                            HOSTNAME   STATUS   AVAILABILITY   MANAGER STATUS
x1a...  *                     pi-node1   Ready    Active         Leader
k9b...                        pi-node2   Ready    Active         Reachable
m3c...                        pi-node3   Ready    Active         Reachable
```

What to look for:

- **STATUS** should be `Ready`. `Down` means the manager cannot reach that node.
- **AVAILABILITY** should be `Active`. `Drain` or `Pause` means the node will not get new tasks (we meet drain in lesson 9).
- **MANAGER STATUS**: exactly one `Leader`, and the others `Reachable`. `Unreachable` means a manager has lost contact.

The `*` marks the node you are talking to.

---

## Step 2: look closer at a node

```bash
docker node inspect pi-node2 --pretty
```

This prints the node's hostname, role, availability, Docker Engine version, architecture (`aarch64` on a 64-bit Pi) and resources. It's the first place to look when one node behaves differently from the rest, for example a different Docker version.

---

## Step 3: run a real test service

A list of `Ready` nodes is encouraging, but let's prove the cluster can actually *do* something. Create a small nginx web service with four replicas:

```bash
docker service create --name smoke --replicas 4 --publish 8101:80 nginx:alpine
```

Watch where Swarm placed the containers (called *tasks*):

```bash
docker service ps smoke
```

You will see one row per task with the node it landed on. On a three-node cluster, the four tasks spread across the nodes. That spreading is Swarm's scheduler at work.

```mermaid
flowchart TB
    U["You: browser or curl"] -->|"port 8101"| N2["Any node (routing mesh)"]
    N2 --> T1["smoke.1 on pi-node1"]
    N2 --> T2["smoke.2 on pi-node2"]
    N2 --> T3["smoke.3 on pi-node3"]
    N2 --> T4["smoke.4 on pi-node1"]
```

### The routing mesh

Because you *published* port 8101, every node in the swarm now listens on it, even nodes not running a task. Swarm calls this the **routing mesh**. From your computer, try each Pi's address:

```bash
curl -I http://192.168.50.11:8101
curl -I http://192.168.50.12:8101
curl -I http://192.168.50.13:8101
```

Each should answer `HTTP/1.1 200 OK` from the nginx container. That is real proof: the network, the overlay and the scheduler all work.

---

## Step 4: read the logs

```bash
docker service logs smoke
```

Your `curl` requests appear here, from whichever node served them. This one command gathers logs from every node.

---

## Step 5: tidy up

```bash
docker service rm smoke
```

---

## Try it Yourself

1. Run the smoke test and note which node each of the four tasks landed on.
2. Scale it up with `docker service scale smoke=8` and check `docker service ps smoke` again.
3. Challenge: switch off one worker Pi and watch `docker service ps smoke`. Where do its tasks go?

---

## Common Issues

- **Problem**: `curl` works against one node's IP but not the others.
- **Solution**: Open 7946 (tcp and udp) and 4789/udp between the nodes.
- **Why**: The routing mesh relays traffic over the overlay network, which uses those ports.

- **Problem**: `docker service ps` shows tasks stuck at `Pending` or repeatedly `Rejected`.
- **Solution**: Run `docker service ps --no-trunc smoke` and read the error column. Typical causes are no image for the platform, or no node with enough memory.
- **Why**: Swarm only schedules a task where it can run. The error text says what blocked it.

- **Problem**: A node shows `Down` right after joining.
- **Solution**: Check the node's clock and network, and that its Docker Engine is running.
- **Why**: Nodes prove themselves with certificates that depend on the correct time.

> **Teachers**: Learners often skip this verification step and then misdiagnose later problems. Make it a checkpoint: each group shows you a three-node `docker node ls` and a `200 OK` from every node before moving on.

---

## Check your understanding

{% quiz %}
? What does an `Unreachable` manager status mean?
- [ ] The node is a worker
- [x] The other managers have lost contact with that manager
- [ ] The node is paused
> `Reachable` is normal for non-leader managers. `Unreachable` means the others cannot contact it.

? Why does `curl` to a node that is not running a task still work?
- [ ] Docker copies the container to that node
- [x] The routing mesh forwards published-port traffic to a node that has a task
- [ ] It does not, that is a mistake
> Publishing a port opens it on every node, and the routing mesh relays requests to a running task.

? Which command gathers logs from all tasks of a service?
- [x] `docker service logs <name>`
- [ ] `docker logs <name>` on one node
- [ ] `docker node logs`
> `docker service logs` collects the output of every task in the service, from every node.
{% endquiz %}
