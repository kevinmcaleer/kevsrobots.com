---
layout: course_overview
title: Docker
author: Kevin McAleer
cover: /learn/docker/assets/docker.jpg
date: 2024-01-21
description: Learn how to use Docker on a Raspberry Pi or Linux PC. Build a robot
  status web app, package it in an image, then run it with Redis, a network and a
  volume using Docker Compose.
level: beginner
course_duration: 0h 54m
lesson_count: 11
first_lesson: 00_intro.html
quiz_total: 30
outcomes:
- Explain the difference between an image and a container
- Install Docker Engine and the Compose plugin on Raspberry Pi OS
- Write a Dockerfile and build your own image
- Debug a container using its logs and a shell
- Connect containers with ports and networks, and keep data in volumes
- Run a multi-container app with a Compose file
prerequisites:
- Introduction to the Linux Command Line on Raspberry Pi OS
you_need:
- A Raspberry Pi 4 or 5 running Raspberry Pi OS (64-bit), or a Linux PC
- An internet connection
- A web browser on any computer on the same network
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
    - name: Creating a Docker Compose File
      link: 08_docker-compose.html
    - name: 'Project: The Robot Status Stack'
      link: 09_dockerise.html
    - name: Conclusion and Next Steps
      link: 10_conclusion.html
---
