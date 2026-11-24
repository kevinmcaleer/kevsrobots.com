---
layout: lesson
title: Executions and Debugging
author: Kevin McAleer
type: page
cover: /learn/n8n_homelab/assets/banners/10_executions_and_debugging.jpg
date: 2026-11-24
previous: 09_credentials_and_secrets.html
next: 11_updates_and_backups.html
description: Read past runs in the Executions tab, replay a failed run in the editor,
  and keep the execution history under control
percent: 77
duration: 4
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

Stage 6, part two: the butler keeps a diary. Every run of a workflow is an **execution**, and learning to read them turns "it's broken" into "this node got this item and failed".

---

## Where to look

Open a workflow and select its **Executions** tab to see every run of that workflow, each with a status and a time. There's also an all-executions list across every workflow. Select a run and the canvas shows what each node received and sent in that run.

Two kinds of run appear:

- **Manual**, from **Execute workflow** while you build
- **Production**, started by a published trigger such as your schedule or webhook

Production webhook data only shows here, never in the editor, so this tab is where you check on the Pico.

## Replay a failure in the editor

This is the debugging feature to remember. Break something on purpose: set the health check URL to a nonsense address and let it run, or send a bad request to the webhook. Then:

1. Open the **Executions** tab and select the failed run.
2. Choose **Debug in editor**. For a successful run, the button is **Copy to editor**.
3. n8n copies that run's data into your workflow and pins it to the first node.
4. Fix the node, then run again. It uses the pinned data, so you can test the fix against the original input without waiting for the Pico to send another reading.

Remember to unpin the data when you've finished, so the next test starts fresh. On self-hosted n8n, the docs say this needs a registered Community edition (or a paid one), so register your instance if the button is missing.

## Look at the right node

A good routine for any failure:

1. Which execution failed? Open it.
2. Which node is red? Open that one.
3. What was its **input**? Is it what you expected?
4. What does its error message say?

Nine times out of ten the input is the surprise, such as a field in `body` that you read from `json`.

## Keeping the history tidy

On a Pi, every stored execution takes space, and with a reading every minute, that adds up. n8n prunes old ones for you, and you can tune it with environment variables in your compose file:

| Variable | What it controls | Default |
|---|---|---|
| `EXECUTIONS_DATA_PRUNE` | Whether old executions are deleted | On |
| `EXECUTIONS_DATA_MAX_AGE` | Hours before a finished run can be deleted | `336` (14 days) |
| `EXECUTIONS_DATA_PRUNE_MAX_COUNT` | How many finished runs to keep | `10000` |
| `EXECUTIONS_DATA_SAVE_ON_SUCCESS` | Save successful runs, `all` or `none` | See the docs |
{:class="table table-single"}

For example, to keep a week of history:

```yaml
    environment:
      - EXECUTIONS_DATA_MAX_AGE=168
```

You can also choose what to save in each workflow's settings. n8n's database is SQLite by default, and freed space isn't returned to the disk automatically. The docs mention `DB_SQLITE_VACUUM_ON_STARTUP` for that.

## Try it Yourself

1. Find the oldest execution in your health check workflow. How many runs does it have per day?
2. Use **Debug in editor** on a failed run, fix the node, and run it again with the pinned data.
3. Challenge: for the sensor inbox, work out how many executions per day one Pico creates, and how long 10,000 would last.

## Check your understanding

{% quiz %}
? Where do you see data from a production webhook call?
- [ ] On the Webhook node in the editor
- [x] In the workflow's Executions tab
- [ ] In the Docker logs only
> The editor only shows data for test calls. Production runs are recorded as executions.

? What does Debug in editor do for a failed run?
- [ ] Deletes the execution
- [x] Copies its data into your workflow and pins it to the first node
- [ ] Restarts the container
> You can then fix the workflow and re-run it against the original input.

? Why prune old executions on a Pi?
- [ ] n8n refuses to run without pruning
- [x] Stored runs take disk space, and a busy webhook creates many
> Set a shorter max age if disk space is tight.
{% endquiz %}

## Common Issues

**Problem**: I can't find my Pico's readings.
**Solution**: Look in the Executions tab, not the editor, and check the workflow is published.
**Why**: Only test calls show data on the canvas.

**Problem**: The Executions tab is empty.
**Solution**: Check the workflow's settings for which executions are saved.
**Why**: You can choose to skip saving successful or manual runs.
