---
layout: lesson
title: The Editor and Your First Workflow
author: Kevin McAleer
type: page
cover: /learn/n8n_homelab/assets/banners/03_the_editor_and_your_first_workflow.jpg
date: 2026-11-24
previous: 02_install_n8n_with_docker_compose.html
next: 04_a_scheduled_health_check.html
description: Take a tour of the n8n editor and build, run and publish your first workflow
percent: 28
duration: 3
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

Time for stage 1: the butler runs a workflow when you ask. We'll build a tiny one and meet the editor on the way.

---

## A quick tour

Sign in and create a workflow from the **Overview** page. The big empty area is the **canvas**. The things you will use all the time:

- The **plus button** on the canvas, which opens the node panel
- Each node, which you double-click to open its settings, with **input** on the left and **output** on the right
- **Execute workflow**, which runs everything from the trigger
- The **Executions** tab, which lists past runs
- **Publish** in the canvas header, which makes the workflow run by itself

Changes save automatically within a few seconds, and they stay a draft until you publish.

## Build the first workflow

We'll ask n8n to fetch its own health page.

1. Add a **Manual Trigger** node, the "run it when I click" trigger.
2. Add an **HTTP Request** node after it.
3. Set **Method** to `GET` and **URL** to `http://localhost:5678/healthz`.
4. Click **Execute workflow**.

Open the HTTP Request node and look at its **Output**. You'll see the reply from n8n. Each node shows what went in and what came out, and this is the single best debugging habit you can build.

**Why `localhost` works here:** the request is made by n8n, from inside its container, to itself. Pointing at another service on the Pi is different, and lesson 4 explains why.

## Test runs and real runs

Clicking **Execute workflow** is a **manual execution**. It works on a draft and shows results on the canvas. A **production execution** happens when a published workflow is started by its trigger. A Manual Trigger only runs when you click, so there is nothing to publish here, but the schedule and webhook workflows ahead need **Publish** (or `Shift` + `p`) before they run on their own.

## Try it Yourself

1. Change the URL to `http://localhost:5678/not-a-real-page`. What does the node do?
2. Rename the workflow to `Butler - hello`. Click its name at the top.
3. Challenge: open the **Executions** tab. Do manual runs appear there? Compare with what the docs say about manual and production runs.

## Common Issues

**Problem**: The node fails with "connection refused" when I use a service on the Pi.
**Solution**: Use the Pi's LAN address, not `localhost`.
**Why**: `localhost` inside the n8n container is the container itself.

**Problem**: I clicked Execute workflow but nothing happened.
**Solution**: Check that the workflow begins with a trigger and that the nodes are joined by a line.
**Why**: n8n only runs nodes it can reach from the trigger.
