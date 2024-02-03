---
layout: lesson
title: Initializing Docker Swarm
author: Kevin McAleer
type: page
cover: /learn/docker_swarm/assets/docker_swarm.jpg
date: 2024-02-03
previous: 02_cloning-raspberry-pi-os.html
next: 04_adding-worker-nodes.html
description: This lesson guides you through the process of initializing Docker Swarm
  on your Raspberry Pi cluster, transforming individual Pis into a powerful, unified
  computing resource.
percent: 24
duration: 3
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
    - name: Further Resources
      link: 14_further-resources.html
---


## Setting Up Docker Swarm on Raspberry Pi

With your Raspberry Pis prepared and running the cloned OS, the next step is to initialize Docker Swarm. Docker Swarm turns a group of Docker engines into a single, virtual Docker engine, where you can deploy and manage containers across multiple Raspberry Pi nodes seamlessly.

---

### What is Docker Swarm?

Docker Swarm is a container orchestration tool, part of Docker, that allows you to manage multiple Docker hosts as a single entity. It's designed for ease of use and simplicity, making it an excellent choice for beginners and for use with Raspberry Pi clusters.

---

### Prerequisites

- All Raspberry Pis are set up with the cloned Raspberry Pi OS and are networked together.
- Docker is installed on all Raspberry Pis. Refer to the previous lesson if you haven't installed Docker yet.

---

### Step 1: Choose a Manager Node

- In a Docker Swarm, at least one node acts as the manager (or multiple for redundancy), which orchestrates the deployment and manages the cluster state.
- Decide which Raspberry Pi will be the manager node. Ideally, pick a Pi that you can easily access for administrative tasks.

---

### Step 2: Initializing the Swarm

1. **Open a Terminal** on your chosen manager Raspberry Pi.
1. **Run the Docker Swarm Init Command**: Execute the following command:

   ```sh
   docker swarm init --advertise-addr <MANAGER_IP>
   ```

   Replace `<MANAGER_IP>` with the IP address of your manager Raspberry Pi. This IP should be static or reserved to prevent issues.

1. **Note the Join Token**: Upon successful initialization, Docker will output a token that worker nodes will use to join the Swarm. It looks something like this:

   ```plaintext
   docker swarm join --token SWMTKN-1-<token_string> <MANAGER_IP>:2377
   ```

   Keep this token secure; you'll need it to add worker nodes to your Swarm.

### Step 3: Adding Worker Nodes

---

1. **Access Each Worker Node**: Log into each Raspberry Pi you wish to add as a worker node.
1. **Join the Swarm**: On each worker node, execute the join command you noted earlier. This will connect the node to the Swarm as a worker.

### Step 4: Verifying the Swarm

- **Check the Swarm's Status**: On the manager node, run:

  ```sh
  docker node ls
  ```

  This command lists all nodes in the Swarm, showing their roles (manager or worker) and statuses.

---

### Docker Swarm Management

- **Promoting Nodes**: You can promote worker nodes to manager status for redundancy using:

  ```sh
  docker node promote <NODE_NAME>
  ```

- **Demoting Nodes**: Similarly, managers can be demoted to workers with:

  ```sh
  docker node demote <NODE_NAME>
  ```

---

### Summary

Initializing Docker Swarm on your Raspberry Pi cluster is a pivotal step in creating a unified computing platform. With Swarm, you can deploy containers across multiple Pis, manage workload distribution, and ensure your applications are highly available. In the upcoming lessons, we'll explore how to add applications and services to your newly formed Docker Swarm cluster.

---
