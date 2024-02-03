---
layout: lesson
title: Docker Compose and Swarm Stacks
author: Kevin McAleer
type: page
cover: /learn/docker_swarm/assets/docker_swarm.jpg
date: 2024-02-03
previous: 05_verifying-cluster-setup.html
next: 07_writing-a-docker-compose-file.html
description: Learn the difference between Docker Compose and Swarm Stacks, and how
  to utilize them for deploying applications on your Docker Swarm cluster.
percent: 42
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


## Introduction to Deployment Tools

Deploying applications on a Docker Swarm cluster requires understanding the tools available for managing service configurations and orchestrations. This lesson introduces Docker Compose and Swarm Stacks, explaining their purposes, differences, and how to use them effectively in a Swarm environment.

---

### Understanding Docker Compose

Docker Compose is a tool for defining and running multi-container Docker applications. With Compose, you use a YAML file to configure your application's services, networks, and volumes. Then, with a single command, you create and start all the services defined in your configuration.

- **Primary Use**: Ideal for development, testing, and staging environments, as well as single-host production deployments.
- **Key Features**:
  - Simple, declarative YAML configuration for services.
  - Manages the entire application lifecycle with simple commands.
  - Supports variables and extension fields for flexible configurations.

---

### Introduction to Docker Swarm Stacks

Swarm Stacks are similar to Docker Compose but are specifically designed for managing applications across a Docker Swarm cluster. Stacks allow you to deploy, update, and scale services across multiple Docker hosts within the Swarm.

- **Primary Use**: Best suited for production environments and multi-host deployments in a Docker Swarm.
- **Key Features**:
  - Utilizes Docker Compose files (version 3 and above) for service definitions.
  - Seamlessly integrates with Docker Swarm for native clustering support.
  - Enables service replication across nodes, load balancing, and secure service-to-service communication.

---

### Differences Between Docker Compose and Swarm Stacks

| Feature | Docker Compose | Swarm Stacks |
|---------|----------------|--------------|
| Scope   | Single host    | Multi-host (Swarm) |
| Use Case| Development, testing, and small-scale production | Large-scale production deployments in Swarm |
| Command | `docker-compose up` | `docker stack deploy` |
| File Format | Docker Compose YAML (supports version 2 and 3) | Docker Compose version 3 YAML, with Swarm-specific options |
{:class="table table-striped"}

---

### Deploying a Stack to Docker Swarm

1. **Create a Docker Compose File**: Define your application stack in a `docker-compose.yml` file, using version 3 syntax to ensure compatibility with Docker Swarm.
1. **Deploy the Stack**:

   - Run the following command on a manager node to deploy the stack:

     ```sh
     docker stack deploy -c docker-compose.yml <STACK_NAME>
     ```

   - Replace `<STACK_NAME>` with a name for your stack.
1. **Verify Deployment**:
   - Check the status of your stack with:

     ```sh
     docker stack services <STACK_NAME>
     ```

   - This command lists the services in your stack, along with their replicas and status.

---

### Summary

Docker Compose and Swarm Stacks provide powerful and flexible tools for deploying and managing containerized applications. Understanding when and how to use each tool is crucial for effectively managing your Docker Swarm cluster. By leveraging Docker Stacks, you can take full advantage of Docker Swarm's capabilities for deploying scalable and resilient applications across multiple nodes in your cluster.

---
