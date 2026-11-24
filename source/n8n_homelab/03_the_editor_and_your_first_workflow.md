---
title: The Editor and Your First Workflow
description: Take a tour of the n8n editor and build, run and publish your first workflow
layout: lesson
type: page
cover: /learn/n8n_homelab/assets/banners/03_the_editor_and_your_first_workflow.jpg
date_updated: 2026-11-24
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
