---
layout: lesson
title: Writing a Docker Compose File
author: Kevin McAleer
type: page
cover: /learn/docker_swarm/assets/docker_swarm.jpg
date: 2024-02-03
previous: 06_docker-compose-and-swarm-stacks.html
next: 08_deploying-a-stack.html
description: This lesson guides you through the process of writing a Docker Compose
  file, enabling you to define and deploy multi-container applications with ease.
percent: 48
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


## Crafting Your Docker Compose Configuration

Docker Compose files are YAML files that describe a multi-container application. Writing an effective Docker Compose file is a foundational skill for deploying applications in both development environments and production Docker Swarms. This lesson will walk you through creating a basic Docker Compose file to define a simple web application stack.

---

### Understanding Docker Compose File Structure

A Docker Compose file is structured into several sections, including `services`, `networks`, and `volumes`, each defining different aspects of your application:

- **Services**: Define the containers you want to run, their Docker images, and configuration options like ports, environment variables, and dependencies.
- **Networks**: Specify the networks your containers should use to communicate with each other.
- **Volumes**: Define persistent storage for your containers, ensuring data is maintained across container restarts.

---

### Basic Docker Compose File Example

Here's a simple example of a Docker Compose file that defines a web application and a database service:

```yaml
version: '3.8'
services:
  web:
    image: nginx:latest
    ports:
      - "80:80"
    depends_on:
      - db
    networks:
      - webnet

  db:
    image: postgres:latest
    environment:
      POSTGRES_PASSWORD: example
    volumes:
      - db-data:/var/lib/postgresql/data
    networks:
      - webnet

networks:
  webnet:

volumes:
  db-data:
```

---

### Key Components Explained

- **version**: Specifies the Docker Compose file version. It's recommended to use `3.8` or above for Docker Swarm compatibility.
- **services**: This section defines two services, `web` (an Nginx web server) and `db` (a PostgreSQL database).
  - **image**: The Docker image to use for the container.
  - **ports**: Maps ports from the container to the host, formatted as `<host>:<container>`.
  - **depends_on**: Specifies that the `web` service depends on the `db` service, ensuring `db` starts before `web`.
  - **environment**: Sets environment variables in the container. Here, it's used to set the PostgreSQL password.
  - **volumes**: Maps persistent storage to the container. For `db`, it ensures data persists across container restarts.
- **networks**: Defines a network named `webnet` for inter-service communication.
- **volumes**: Declares a named volume `db-data` for the database storage, ensuring data persists.

---

### Tips for Writing Docker Compose Files

- **Use environment variables**: For sensitive information like passwords, consider using environment variables instead of hardcoding them in the file.
- **Leverage volumes for persistence**: Define volumes for data that should persist between container restarts or updates, such as database files.
- **Networks for communication**: Define networks to facilitate communication between services, especially when deploying to Docker Swarm.

---

### Summary

Writing a Docker Compose file is an essential step in deploying containerized applications. By understanding and utilizing the key components of a Docker Compose file, you can effectively define, deploy, and manage complex applications with multiple interdependent services. This foundational knowledge will be invaluable as you progress to deploying and scaling applications across a Docker Swarm cluster.

---
