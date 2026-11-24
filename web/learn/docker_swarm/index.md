---
layout: course_overview
title: Raspberry Pi 5 Cluster with Docker Swarm
author: Kevin McAleer
cover: /learn/docker_swarm/assets/docker_swarm.jpg
date: 2024-02-03
description: Build a Raspberry Pi 5 cluster and turn it into a Docker Swarm. Deploy
  stacks, scale and update services, rebalance after a node failure, then monitor,
  maintain and back up your cluster.
level: intermediate
course_duration: 1h 23m
lesson_count: 16
first_lesson: 00_intro.html
quiz_total: 45
video: tDENgLiJSh0
outcomes:
- Plan a Raspberry Pi cluster with the right number of managers and workers
- Prepare identical nodes and create a Docker Swarm with docker swarm init and join
- Write a stack file with deploy settings and run it with docker stack deploy
- Scale a service, roll out an update and roll it back
- Drain, rebalance and replace nodes without taking your services offline
- Back up your volumes and swarm state, and recover from a lost manager
prerequisites:
- The Docker course, or equivalent knowledge of images, containers and Compose files
- Basic Linux command line skills
- A rough idea of IP addresses and ports
you_need:
- Two to four Raspberry Pi 5 boards with power supplies (three is ideal)
- A microSD card per Pi, 32 GB or larger, or an SSD
- An Ethernet switch and cables
- A computer on the same network with SSH and Raspberry Pi Imager
- 64-bit Raspberry Pi OS on every Pi
navigation:
- name: Raspberry Pi 5 Cluster with Docker Swarm
- content:
  - section: Introduction
    content:
    - name: Docker Swarm on Raspberry Pi 5
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
    - name: Summary and Next Steps
      link: 14_summary-and-next-steps.html
---
