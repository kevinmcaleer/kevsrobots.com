---
layout: lesson
title: Creating a Docker-Compose File
author: Kevin McAleer
type: page
cover: /learn/docker/assets/docker.jpg
date: 2024-01-21
previous: 07_volumes.html
next: 09_dockerise.html
description: Learn how to use Docker Compose to manage multi-container Docker applications.
percent: 81
duration: 2
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


## Introduction to Docker Compose

Docker Compose is an invaluable tool for developers working with applications that require running multiple Docker containers. It simplifies the process of defining and running multi-container Docker applications, using a single YAML file for configuration.

---

## Writing a Docker-Compose File

The heart of Docker Compose is the YAML file where you define your application's services, networks, and volumes.

---

### Understanding YAML Syntax

- YAML (YAML Ain't Markup Language) is a human-readable data serialization language.
- Docker Compose files typically start with the version definition, followed by the services section.
- Indentation is crucial in YAML and helps define the structure.

---

### Defining Services

- **Services**: Each service in the compose file represents a container.
- Define each service with the image to use, ports to expose, and other configurations.
- Example service definition:

  ```yaml
  version: '3.8'
  services:
    web:
      image: nginx:latest
      ports:
        - "80:80"
  ```

---

### Networks and Volumes

- **Networks**: Define how containers within your Compose file communicate with each other.
- **Volumes**: Specify persistent storage for your containers.

  ```yaml
  services:
    db:
      image: postgres:latest
      volumes:
        - db-data:/var/lib/postgresql/data
  volumes:
    db-data:
  ```

---

## Running Multi-container Applications

Docker Compose not only defines how your containers should run but also offers commands to manage them easily.

---

### Launching Services

- To start your application with all its services, use `docker-compose up`.
- This command reads the `docker-compose.yml` file in the current directory by default.

---

### Managing Services

- **Viewing the Status**: Use `docker-compose ps` to see the status of your application’s services.
- **Stopping Services**: Stop your application using `docker-compose down`. This stops and removes containers, networks, and volumes defined in the compose file.
- **Restarting Services**: To restart, simply run `docker-compose up` again.

---

Docker Compose streamlines the process of working with multi-container applications, making it easier to launch, manage, and scale services. Whether you’re developing locally or deploying to a server, Docker Compose provides a convenient way to manage complex application architectures.

---
