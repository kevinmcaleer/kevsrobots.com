---
layout: course_overview
title: Homelab Automation with n8n
author: Kevin McAleer
cover: /learn/n8n_homelab/assets/cover.jpg
date: 2026-11-24
description: Turn a Raspberry Pi into an automation hub with n8n and Docker. Build
  a homelab robot butler that checks your services on a schedule, listens for readings
  from a Pico W, tidies the data with a little JavaScript, and tells you when something
  is wrong.
level: beginner
course_duration: 0h 51m
lesson_count: 14
first_lesson: 00_intro.html
quiz_total: 9
video: Yds3_vQR5xM
outcomes:
- Run n8n on a Raspberry Pi with Docker Compose, with your data kept safe in a volume
- Build, test and publish a workflow with a trigger, nodes and branches
- Check a homelab service on a schedule and get a message when it fails
- Receive data from a Pico W with a Webhook node and protect it with a key
- Reshape data with the Code node, using JavaScript
- Keep secrets in credentials, debug past executions, and update and back up n8n
prerequisites:
- Docker
- Introduction to the Linux Command Line on Raspberry Pi OS
you_need:
- A Raspberry Pi 4 or newer (a Pi 5 or Pi 500 is more comfortable), running Raspberry
  Pi OS
- Docker and Docker Compose installed on the Pi
- A computer on the same network, with a web browser
- A Raspberry Pi Pico W, for lessons 6 and 7
- An email account you can send from (SMTP details), for lesson 5
navigation:
- name: Homelab Automation with n8n
- content:
  - section: Meet n8n
    content:
    - name: Introduction
      link: 00_intro.html
    - name: What Is n8n
      link: 01_what_is_n8n.html
    - name: Install n8n with Docker Compose
      link: 02_install_n8n_with_docker_compose.html
    - name: The Editor and Your First Workflow
      link: 03_the_editor_and_your_first_workflow.html
  - section: Watching your homelab
    content:
    - name: A Scheduled Health Check
      link: 04_a_scheduled_health_check.html
    - name: If the Service Is Down
      link: 05_if_the_service_is_down.html
  - section: Listening to your devices
    content:
    - name: A Webhook for the Pico
      link: 06_a_webhook_for_the_pico.html
    - name: A Pico W Sends a Reading
      link: 07_pico_w_sends_a_reading.html
    - name: Tidy Data with the Code Node
      link: 08_tidy_data_with_the_code_node.html
  - section: Running it for real
    content:
    - name: Credentials and Secrets
      link: 09_credentials_and_secrets.html
    - name: Executions and Debugging
      link: 10_executions_and_debugging.html
    - name: Updates and Backups
      link: 11_updates_and_backups.html
    - name: Troubleshooting
      link: 12_troubleshooting.html
    - name: Summary
      link: 13_summary.html
---
