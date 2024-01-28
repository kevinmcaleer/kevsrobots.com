---
title: "Project: Containerize a Simple Application"
description: Practical project to containerize a simple web application using Docker.
layout: lesson
type: page
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
