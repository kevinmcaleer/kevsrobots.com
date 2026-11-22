---
layout: course_overview
title: OpenClaw on Raspberry Pi
author: Kevin McAleer
cover: /learn/openclaw_raspberry_pi/assets/openclaw.jpg
date: 2026-01-31
description: Run OpenClaw, the open-source personal AI agent, on a Raspberry Pi in
  Docker. Build Pi Butler, an always-on assistant you talk to from your phone, with
  its own persona, a custom skill that reports how the Pi is feeling, a morning brief,
  backups, safe upgrades and a security check.
level: intermediate
course_duration: 2h 35m
lesson_count: 19
first_lesson: 00_intro.html
quiz_total: 54
video: 7JtSxb7wB8c
outcomes:
- Explain what the OpenClaw Gateway does, and what stays on the Pi versus what goes
  to the AI model provider
- Install OpenClaw on a Raspberry Pi with Docker and the official pre-built image
- Reach the Control UI safely through an SSH tunnel, and configure models and a persona
- Connect a Telegram bot and approve who may talk to your agent
- Write a custom skill and schedule an automation
- Monitor, troubleshoot, back up, upgrade and security-audit an always-on agent
prerequisites:
- Basic Linux command line, such as Introduction to the Linux Command Line on Raspberry
  Pi OS
- Helpful but not required, the Docker course
you_need:
- A Raspberry Pi 4 or Pi 5 with 2 GB of RAM or more (4 GB recommended)
- The official power supply, and a 16 GB or larger microSD card or a USB SSD
- A network connection, Ethernet preferred
- A computer to SSH from, and a free Telegram account on your phone
- An API key from an AI model provider such as Anthropic or OpenAI, which is billed
  per use
navigation:
- name: OpenClaw on Raspberry Pi
- content:
  - section: Getting Started
    content:
    - name: Course Overview
      link: 00_intro.html
    - name: What is OpenClaw?
      link: 01_what-is-openclaw.html
    - name: Hardware Requirements
      link: 02_hardware-requirements.html
    - name: Why Docker?
      link: 03_why-docker.html
  - section: Installation & Setup
    content:
    - name: Preparing Your Raspberry Pi
      link: 04_preparing-your-pi.html
    - name: Installing Docker
      link: 05_installing-docker.html
    - name: Docker Compose Setup
      link: 06_docker-compose-setup.html
    - name: First Access to the Control UI
      link: 07_first-access.html
  - section: Configuration & Usage
    content:
    - name: Configuring OpenClaw
      link: 08_configuring-openclaw.html
    - name: Pi Butler in Your Pocket
      link: 09_first-agent.html
    - name: Understanding Skills
      link: 10_understanding-skills.html
  - section: Management & Troubleshooting
    content:
    - name: Managing Containers
      link: 11_managing-containers.html
    - name: Monitoring Resources
      link: 12_monitoring-resources.html
    - name: Troubleshooting
      link: 13_troubleshooting.html
  - section: Advanced Topics
    content:
    - name: Is OpenClaw Safe on a Raspberry Pi?
      link: 14_security.html
    - name: Backup and Recovery
      link: 15_backup-and-recovery.html
    - name: Updates and Upgrades
      link: 16_updates-and-upgrades.html
    - name: Production Setup
      link: 17_production-setup.html
  - section: Summary
    content:
    - name: Next Steps
      link: 18_next-steps.html
---
