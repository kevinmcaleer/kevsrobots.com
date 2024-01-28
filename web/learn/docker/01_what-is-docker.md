---
layout: lesson
title: Introduction to Docker
author: Kevin McAleer
type: page
cover: /learn/docker/assets/docker.jpg
date: 2024-01-21
previous: 00_intro.html
next: 02_installing-docker.html
description: Learn what Docker is and why it's revolutionizing software development.
percent: 18
duration: 1
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


## Understanding Docker

In this lesson, we introduce `Docker`, a platform that has changed the way software is developed and deployed.

### What is Docker?

Docker is a tool designed to make it easier to create, deploy, and run applications by using containers. Containers allow a developer to package up an application with all the parts it needs, such as libraries and other dependencies, and ship it all out as one package.

### Why Use Docker?

- **Consistency**: Docker ensures that your application works seamlessly in any environment.
- **Simplicity**: Simplifies configuration management.
- **Isolation**: Containers are isolated from each other and the host system.
- **Scalability**: Easily scale up or scale down applications.

### Docker vs. Virtual Machines

- Docker containers are more lightweight than virtual machines.
- Containers share the host system’s kernel, while VMs require a full-blown OS.
- Docker can start up containers quickly compared to VMs.

---
