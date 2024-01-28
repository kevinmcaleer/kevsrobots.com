---
layout: lesson
title: Docker Networking Basics
author: Kevin McAleer
type: page
cover: /learn/docker/assets/docker.jpg
date: 2024-01-21
previous: 05_managing.html
next: 07_volumes.html
description: Understanding the fundamentals of networking in Docker.
percent: 63
duration: 3
navigation:
- name: Docker
- content:
  - section: Overview
    content:
    - name: Overview
      link: 00_intro.html
    - name: Introduction to Docker
      link: 01_what-is-docker.html
    - name: Installing Docker
      link: 02_installing-docker.html
  - section: Images and Containers
    content:
    - name: Docker Basics
      link: 03_basics.html
    - name: Building Your First Docker Image
      link: 04_building.html
    - name: Managing Docker Containers
      link: 05_managing.html
    - name: Docker Networking Basics
      link: 06_networking.html
    - name: Docker Volumes and Persistent Data
      link: 07_volumes.html
  - section: Docker Compose
    content:
    - name: Creating a Docker-Compose File
      link: 08_docker-compose.html
    - name: 'Project: Containerize a Simple Application'
      link: 09_dockerise.html
    - name: Conclusion and Next Steps
      link: 10_conclusion.html
---


## Navigating Docker Networking

Docker networking is a key component in the deployment of applications that need to communicate within the Docker host or externally. This lesson introduces you to the fundamentals of Docker networking, helping you understand how to connect containers effectively.

---

### Docker Network Types

Docker supports several network types, each serving different use cases. Here's an overview:

1. **Bridge Network**:
   - The default network type when you run a container with `docker run`.
   - Creates a network stack on the host in the Docker bridge network.

2. **Host Network**:
   - Removes network isolation between the container and the Docker host, using the host's networking directly.
   - Useful when you don’t need network isolation between the container and the host.

3. **None Network**:
   - Disables all networking for the container.
   - Typically used for troubleshooting or when a container doesn't need a network.

4. **Overlay Network**:
   - Used in Docker Swarm for containers running on different Docker hosts to communicate.
   - Supports multi-host networking.

---

#### Use Cases for Each Network Type

- **Bridge**: Ideal for small-scale applications that need to communicate with each other.
- **Host**: Used when performance is critical, and the network isolation is not a concern.
- **None**: When you need a container to be completely network-isolated.
- **Overlay**: Best for larger, distributed applications across multiple hosts.

---

#### Default Networking Behavior

- By default, Docker uses the bridge network. Containers connected to the same bridge network can communicate, while being isolated from containers on other bridge networks.

---

### Connecting Containers

Containers can be networked together in various ways:

1. **Within the Same Network**:
   - Containers on the same network can communicate using their container names as DNS names.
   - Use `docker network create` to create a custom bridge network and connect containers to it.

2. **Between Different Networks**:
   - Containers on different networks can't communicate directly.
   - You need to use port mapping or link containers explicitly.

3. **Using Docker Network Commands**:
   - `docker network ls`: Lists all networks.
   - `docker network create [name]`: Creates a new network.
   - `docker network connect [network] [container]`: Connects a container to a network.

---

### Port Mapping

Port mapping is essential for making containers accessible to the outside world.

1. **Understanding Port Mapping**:
   - In Docker, port mapping binds a port on the container to a port on the Docker host.

2. **Exposing Container Ports**:
   - Use the `-p` flag in `docker run` (e.g., `docker run -p 80:80 [image-name]`) to map a host port to a container port.

3. **Configuring Port Mapping**:
   - The format for port mapping is `hostPort:containerPort`.
   - This enables external systems to access services on a container via the host's IP address and the mapped port.

---

Understanding Docker networking is crucial for deploying and managing applications effectively in Docker environments. By mastering different network types and learning how to connect and expose containers, you can ensure smooth communication for your Dockerized applications.

---
