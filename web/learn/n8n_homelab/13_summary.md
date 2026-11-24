---
layout: lesson
title: Summary
author: Kevin McAleer
type: page
cover: /learn/n8n_homelab/assets/banners/13_summary.jpg
date: 2026-11-24
previous: 12_troubleshooting.html
description: Look back at the six stages of the homelab robot butler, keep the rules
  that matter, and find where to go next
percent: 100
duration: 2
quiz_total: 9
date_updated: 2026-11-24
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


![Course Cover Image]({{page.cover}}){:class="cover"}

---

Your Raspberry Pi now has a butler. It checks your services, listens to a Pico W, tidies the data, tells you when something is wrong, and you know how to debug, update and back it up. That's a real automation platform, built from one container.

---

## What you built

| Stage | Lesson | What was added |
|---|---|---|
| 1 | 3 | A workflow you run on demand |
| 2 | 4 | A scheduled health check |
| 3 | 5 | An IF branch and an email alert |
| 4 | 6 and 7 | A webhook and a Pico W that posts readings |
| 5 | 8 | A Code node that tidies the data and flags a hot room |
| 6 | 9 to 11 | Credentials, debugging, updates and backups |
{:class="table table-single"}

```mermaid
flowchart LR
    Pico["Pico W"] -->|POST| Hook["Webhook"] --> Code["Code node"] --> If{"Too hot?"}
    Sched["Schedule"] --> Http["HTTP Request"] --> If2{"Status 200?"}
    If -->|yes| Mail["Send Email"]
    If2 -->|no| Mail
    Err["Error workflow"] --> Mail
```

## The ideas to keep

- **Trigger, nodes, items.** Everything is a trigger that starts a chain of nodes passing items.
- **Look at the output of every node.** It is the fastest way to learn and to debug.
- **Test URL while building, Production URL when published.**
- **A schedule or webhook only runs by itself once the workflow is published.**
- **Never expose n8n to the internet without authentication and HTTPS.**
- **Secrets belong in credentials.** Back up the encryption key with your data.
- **Update monthly, back up first.**

## What's next

You've got the platform. Now give it more to do.

- [Docker](/learn/docker/) goes deeper into containers, networks and volumes, which is what n8n runs on
- [Docker Swarm](/learn/docker_swarm/) spreads containers over several Pis
- [K3s](/learn/k3s/) runs a lightweight Kubernetes cluster on your Pis
- [MicroPython GPIO](/learn/micropython_gpio/) gives the Pico W more sensors to report

Kev's blog post on this topic also shows how to add a local AI model with Ollama and call it from an HTTP Request node, which makes a fine next step for the butler.

Happy automating!
