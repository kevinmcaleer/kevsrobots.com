---
layout: course_overview
title: From Docker to Podman
author: Kevin McAleer
cover: /learn/podman/assets/cover.jpg
date: 2025-05-24
description: Learn how to transition from Docker to Podman and unlock better container
  workflows  with rootless and daemonless architecture. This hands-on course covers
  everything from  architectural differences and CLI compatibility to unique Podman
  features like systemd integration  and Kubernetes support. Perfect for developers
  and DevOps professionals looking to modernize  their container workflows.
level: intermediate
course_duration: 1h 9m
lesson_count: 14
first_lesson: 00_intro.html
quiz_total: 47
outcomes:
- Explain what daemonless and rootless mean and why they matter
- Install Podman on Raspberry Pi OS and run your first container without sudo
- Build, run, inspect and clean up images and containers with the Podman CLI
- Group containers into a pod and export it as Kubernetes YAML
- Run a container as a systemd service that restarts and starts at boot, using Quadlet
- Migrate a Docker robot status app to Podman
prerequisites:
- Docker
- Introduction to the Linux Command Line on Raspberry Pi OS
you_need:
- A Raspberry Pi 4 or 5 running Raspberry Pi OS (64-bit), or another Debian-based
  Linux PC
- An internet connection to pull images
- A terminal, on the Pi itself or over SSH
- A web browser on any computer on the same network
navigation:
- name: From Docker to Podman
- content:
  - section: Getting Started
    content:
    - name: Introduction to Podman
      link: 00_intro.html
    - name: 'Video: Podman vs Docker'
      link: 01_video.html
    - name: Docker vs Podman Architecture
      link: 02_docker_vs_podman_architecture.html
    - name: Installing Podman
      link: 03_installing_podman.html
  - section: Core Usage
    content:
    - name: CLI Compatibility and Basic Commands
      link: 04_cli_compatibility_and_commands.html
    - name: Podman Features Docker Doesn't Have
      link: 05_podman_unique_features.html
    - name: Managing Containers and Images
      link: 06_managing_containers_and_images.html
  - section: Advanced Concepts
    content:
    - name: Working with Pods
      link: 07_working_with_pods.html
    - name: Docker Compose vs Podman Compose
      link: 08_docker_compose_vs_podman_compose.html
    - name: Podman and Kubernetes
      link: 09_podman_and_kubernetes.html
  - section: Real-World Usage
    content:
    - name: Podman in Production
      link: 10_podman_in_production.html
    - name: Troubleshooting and Debugging
      link: 11_troubleshooting_and_debugging.html
  - section: Migration Lab
    content:
    - name: Migration Project
      link: 12_migration_project.html
  - section: Summary
    content:
    - name: Wrap-up and Future Outlook
      link: 13_wrap_up_and_future.html
---
