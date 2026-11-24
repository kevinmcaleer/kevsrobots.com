---
title: Adding Worker Nodes to the Swarm
description: >-
    Join more Raspberry Pis to your swarm as workers and managers, promote and demote nodes, and remove nodes cleanly.
layout: lesson
type: page
cover: /learn/docker_swarm/assets/banners/04_adding-worker-nodes.jpg
date_updated: 2026-11-24
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
