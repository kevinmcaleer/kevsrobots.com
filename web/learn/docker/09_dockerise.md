---
layout: lesson
title: 'Project: Containerize a Simple Application'
author: Kevin McAleer
type: page
cover: /learn/docker/assets/docker.jpg
date: 2024-01-21
previous: 08_docker-compose.html
next: 10_conclusion.html
description: Practical project to containerize a simple web application using Docker.
percent: 90
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


## Project Overview

This practical project is designed to give you hands-on experience with Docker. You will containerize a simple web application, learning the entire process from creating a Dockerfile to running the application in a container. This exercise will solidify your understanding of Docker and its application in real-world scenarios.

---

## Step-by-Step Guide

### Application Overview

- **Web Application**: For this project, consider a basic web application, such as a simple Python Flask or Node.js app.
- The application should have a basic functionality, like displaying a static page or returning a response to a web request.

---

### Creating a Dockerfile

1. **Dockerfile Basics**:
   - Start with specifying the base image, like `python:3.8` or `node:14`.
   - Set a working directory in the container, e.g., `WORKDIR /app`.

2. **Adding Application Files**:
   - Use the `COPY` command to copy your application files into the container.

3. **Installing Dependencies**:
   - For Python, copy the `requirements.txt` file and run `pip install`.
   - For Node.js, copy the `package.json` file and run `npm install`.

4. **Setting the Run Command**:
   - Use the `CMD` command to set the command that runs your application, such as `python app.py` or `npm start`.

---

### Building the Image

- Run `docker build -t myapp:latest .` to build the Docker image from your Dockerfile.
- The `-t` flag tags your image, and `.` indicates the current directory as the build context.

---

### Running the Container

- Once the image is built, run it with `docker run -p 5000:5000 myapp:latest`.
- The `-p` flag maps a port on your host to a port in the container.

---

## Testing and Deployment

### Testing Your Container

- **Local Testing**: Access your application by navigating to `http://localhost:5000` in your web browser.
- Ensure all functionalities work as expected in the containerized environment.

---

### Tips for Deployment

- **Container Registry**: Consider pushing your image to a container registry like Docker Hub for easy deployment.
- **Environment Variables**: Use environment variables for configuration settings that change between environments.
- **Logging and Monitoring**: Implement logging and monitoring for your containerized application to ensure its health and performance in production.

---

By completing this project, you will have gained valuable experience in containerizing applications with Docker. This foundational skill is crucial for modern software development and deployment practices.

---
