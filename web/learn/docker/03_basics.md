---
layout: lesson
title: Docker Basics
author: Kevin McAleer
type: page
cover: /learn/docker/assets/docker.jpg
date: 2024-01-21
previous: 02_installing-docker.html
next: 04_building.html
description: An introductory guide to the fundamental concepts and components of Docker.
percent: 36
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


## Understanding the Basics of Docker

Docker is a powerful tool that has revolutionized the way software is developed and deployed. This lesson introduces you to the fundamental concepts and components of Docker, ensuring you have a solid foundation for your Docker journey.

---

### What is Docker?

Docker is a platform for developing, shipping, and running applications. It enables you to separate your applications from your infrastructure so you can deliver software quickly. Docker provides the ability to package and run an application in a loosely isolated environment called a container.

---

### Key Concepts of Docker

1. **Containers**: Containers are lightweight, standalone, executable packages that include everything needed to run a piece of software, including the code, runtime, libraries, and system tools.

2. **Images**: A Docker image is a lightweight, standalone, executable package of software that includes everything needed to run an application: code, runtime, system tools, system libraries, and settings.

3. **Dockerfile**: A Dockerfile is a text document that contains all the commands a user could call on the command line to assemble an image.

4. **Docker Hub**: Docker Hub is a service provided by Docker for finding and sharing container images with your team.

---

### Installing Docker

The process of installing Docker varies based on your operating system. Check the previous lesson on "Installing Docker" for detailed instructions for Windows, MacOS, and Linux.

---

### Running Your First Container

1. **Pull an Image**:
   - Use `docker pull [image-name]` to download an image from Docker Hub. For example, `docker pull hello-world`.

2. **Run a Container**:
   - Use `docker run [image-name]` to run a container from the image. For instance, `docker run hello-world` will run the `hello-world` container.

3. **List Running Containers**:
   - Use `docker ps` to list all running containers.

---

### Basic Docker Commands

- `docker pull`: Downloads an image from Docker Hub.
- `docker run`: Runs a container from an image.
- `docker ps`: Lists running containers.
- `docker stop`: Stops a running container.
- `docker rm`: Removes a container.
- `docker rmi`: Removes an image.
- `docker build`: Builds an image from a Dockerfile.

---

### Lesson Summary

Docker is a powerful tool that simplifies the process of developing, deploying, and running applications. By using Docker, you can quickly deploy and scale applications into any environment and know your code will run.

---

This lesson provided a brief overview of Docker, its key concepts, and basic commands. Understanding these fundamentals is crucial for working efficiently with Docker and leveraging its full potential in software development and deployment.

---
