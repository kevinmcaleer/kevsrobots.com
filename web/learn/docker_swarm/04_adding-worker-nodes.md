---
layout: lesson
title: Adding Worker Nodes to the Swarm
author: Kevin McAleer
type: page
cover: /learn/docker_swarm/assets/docker_swarm.jpg
date: 2024-02-03
previous: 03_initializing-docker-swarm.html
next: 05_verifying-cluster-setup.html
description: This lesson explains how to expand your Docker Swarm by adding more worker
  nodes to your Raspberry Pi cluster, increasing its capacity and fault tolerance.
percent: 30
duration: 3
navigation:
- name: Raspberry Pi 5 Cluster with Docker Swarm
- content:
  - section: Introduction
    content:
    - name: Introduction to Docker Swarm on Raspberry Pi 5
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
    - name: Further Resources
      link: 14_further-resources.html
---


## Expanding Your Docker Swarm Cluster

After initializing Docker Swarm on your Raspberry Pi cluster, adding worker nodes is the next step to increase its processing capacity and reliability. This lesson will guide you through the process of adding new Raspberry Pis as worker nodes to your existing Swarm.

---

### Preparing New Nodes

Before adding new Raspberry Pis to your Swarm, ensure they are:

- Properly set up with the Raspberry Pi OS and have Docker installed (refer to previous lessons for setup instructions).
- Connected to the same network as your Swarm and have internet access.
- Configured with static IP addresses (recommended) to ensure consistent communication within the Swarm.

---

### Using the Swarm Join Token

1. **Retrieve the Join Token**: If you haven't already noted the join token when you initialized the Swarm, you can retrieve it on the manager node by running:

   ```sh
   docker swarm join-token worker
   ```

   This command will output the complete command to join the Swarm as a worker, including the token.

1. **Join the Swarm**: On each new Raspberry Pi you wish to add as a worker, execute the join command provided by the manager node:

   ```sh
   docker swarm join --token SWMTKN-1-<token_string> <MANAGER_IP>:2377
   ```

   Replace `<token_string>` and `<MANAGER_IP>` with the actual token and IP address of your manager node.

---

### Verifying Worker Nodes

After adding the new worker nodes, you can verify they've successfully joined the Swarm:

- **List Nodes**: On the manager node, run:

  ```sh
  docker node ls
  ```

  This command lists all nodes in the Swarm, including the newly added workers, and shows their status, availability, and role.

---

### Managing Node Roles

Docker Swarm nodes can have either the `manager` or `worker` role:

- **Workers** are responsible for running containers and services but don't participate in Swarm management.
- **Managers** handle orchestration and cluster management tasks but can also run services.

You can dynamically change the role of a node:

- **Promote a Worker to Manager**:

  ```sh
  docker node promote <NODE_ID>
  ```

  This is useful for adding redundancy to your Swarm's management layer.

- **Demote a Manager to Worker**:

  ```sh
  docker node demote <NODE_ID>
  ```

  Use this if you need to reduce the number of manager nodes or to repurpose a node as a worker.

---

### Summary

Adding worker nodes to your Docker Swarm expands the cluster's capacity and fault tolerance, enabling you to run more or larger services. It's a straightforward process that involves using a join token and can be done as your cluster's needs grow. With your Swarm now expanded, you're ready to deploy applications across your Raspberry Pi cluster, leveraging the combined resources of your nodes.

---
