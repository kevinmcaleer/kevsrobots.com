---
layout: lesson
title: A Webhook for the Pico
author: Kevin McAleer
type: page
cover: /learn/n8n_homelab/assets/banners/06_a_webhook_for_the_pico.jpg
date: 2026-11-24
previous: 05_if_the_service_is_down.html
next: 07_pico_w_sends_a_reading.html
description: Add a Webhook node so devices on your network can send data to n8n, and
  protect it with a key
percent: 49
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

Stage 4, part one: the butler gets ears. A **webhook** is a web address that n8n listens on. When a device sends it an HTTP request, the workflow starts. It is the easiest way to get data from a Pico W, a script or another service into n8n.

---

## Add the Webhook node

Create `Butler - sensor inbox` and add a **Webhook** node as the trigger. The settings you need:

| Setting | What to choose |
|---|---|
| HTTP Method | `POST`, because the Pico is sending data |
| Path | A name you choose, such as `pico-reading`. Left alone, n8n generates a random one |
| Authentication | Header auth (we add it below) |
| Respond | Immediately |
{:class="table table-single"}

The node shows two addresses, selected with **Test URL** and **Production URL**.

## Test URL and Production URL

| | Test URL | Production URL |
|---|---|---|
| When it works | After you click **Listen for Test Event** or **Execute workflow**. Stays listening for 120 seconds | Once the workflow is published |
| Where the data shows | In the editor, on the node | Only in the **Executions** tab |
{:class="table table-single"}

Use the Test URL while you build, then switch the Pico to the Production URL. Mixing them up is the most common webhook problem, and lesson 12 lists it.

## Try it from the Pi

Click **Listen for Test Event**, then from another terminal send a request, pasting the Test URL from the node:

```bash
curl -X POST "PASTE_THE_TEST_URL_HERE" \
  -H "Content-Type: application/json" \
  -d '{"device": "pico-lounge", "temperature_c": 21.5}'
```

The node lights up with the data. An incoming request is an item with four parts: `body`, `headers`, `params` and `query`. Our JSON arrives inside `body`, so the temperature is at `body.temperature_c`. Remember that: lesson 8 reads from there.

## Protect it with a key

A webhook is a door into your butler. Anyone who can reach the address can knock. On a home network the risk is small, but build the habit now: never expose a webhook to the internet without authentication and HTTPS.

1. In the Webhook node, set **Authentication** to **Header auth**.
2. Create a new credential. It asks for a header **name** (for example `X-Pico-Key`) and a **value**, which is your shared secret. Generate a long random one, such as `openssl rand -hex 24`.
3. Save, then repeat the curl request with `-H "X-Pico-Key: YOUR_SHARED_SECRET"`. A request without the right header is refused.

Keep the secret out of screenshots and out of Git. The Pico will need a copy, and lesson 7 shows where it goes.

## Try it Yourself

1. Send the curl request without the header. What does curl print?
2. Add a second path, `pico-test`, on a duplicate Webhook node, and compare their Production URLs.
3. Challenge: look at the Webhook node's options. Which one would limit callers to the Pico's IP address?

## Common Issues

**Problem**: The Production URL does nothing.
**Solution**: Publish the workflow first.
**Why**: Production webhooks are only registered for published workflows.

**Problem**: The Test URL worked once and now fails.
**Solution**: Click **Listen for Test Event** again.
**Why**: The test listener only stays open for 120 seconds, for one request.
