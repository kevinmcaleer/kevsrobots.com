---
layout: lesson
title: Building Your First Docker Image
author: Kevin McAleer
type: page
cover: /learn/docker/assets/docker.jpg
date: 2024-01-21
previous: 03_basics.html
next: 05_managing.html
description: Learn how to create a Docker image using a Dockerfile.
percent: 45
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


## Creating a Custom Docker Image

In this lesson, we delve into the process of creating your own Docker images using Dockerfiles. Mastering Dockerfiles is crucial for customizing your Docker environment and tailoring it to your specific needs.

---

### Introduction to Dockerfiles

A Dockerfile is a text document containing all the commands a user could call on the command line to assemble an image. It serves as a recipe for creating Docker images.

---

#### What is a Dockerfile?

- A Dockerfile automates the process of building Docker images.
- It contains a set of instructions and commands to build the image.

---

#### Basic Structure and Instructions of a Dockerfile

- **FROM**: Sets the base image for subsequent instructions.
- **RUN**: Runs a command in a new layer on top of the current image and commits the results.
- **COPY**: Copies new files or directories into the filesystem of the container.
- **CMD**: Provides defaults for executing a container.

---

#### Best Practices for Writing Dockerfiles

- Use official images as your base images.
- Keep your images as small as possible.
- Minimize the number of layers.
- Sort multi-line arguments.

---

### Writing Your First Dockerfile

Creating a Dockerfile involves defining the environment inside your container. Here's a step-by-step guide to writing a simple Dockerfile:

1. **Choosing a Base Image**:
   - Start with choosing an appropriate base image like `ubuntu`, `alpine`, or `node`.

2. **Defining Container Instructions**:
   - **COPY**: Copy your application's source code into the container.
   - **RUN**: Install any dependencies.
   - **CMD**: Specify the command to run your application.

---

#### Example Dockerfile

```Dockerfile
# Use an official Python runtime as a parent image
FROM python:3.7-slim

# Set the working directory in the container
WORKDIR /usr/src/app

# Copy the current directory contents into the container at /usr/src/app
COPY . .

# Install any needed packages specified in requirements.txt
RUN pip install --no-cache-dir -r requirements.txt

# Define environment variable
ENV NAME World

# Run app.py when the container launches
CMD ["python", "./app.py"]
```

---

### Building an Image from a Dockerfile

Once your Dockerfile is ready, you can build an image from it:

1. **Build the Docker Image**:
   - Use `docker build -t your-image-name .` to build your image.
   - The `-t` flag tags your image to make it easier to find later.

2. **Tagging Your Image**:
   - Tagging is done during the build process using the `-t` option.
   - Tags allow you to version and easily identify images.

3. **Running a Container from Your Custom Image**:
   - Use `docker run your-image-name` to run a container from your newly created image.

---

#### Example Command

```bash
docker build -t my-python-app .
docker run my-python-app
```

---

Through this lesson, you've learned how to create a Dockerfile, build a Docker image from it, and run a container using your custom image. This skill is fundamental in Docker, as it allows you to create tailored environments for your applications.

---
