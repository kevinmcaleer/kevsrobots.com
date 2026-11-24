---
layout: lesson
title: Updates and Backups
author: Kevin McAleer
type: page
cover: /learn/n8n_homelab/assets/banners/11_updates_and_backups.jpg
date: 2026-11-24
previous: 10_executions_and_debugging.html
next: 12_troubleshooting.html
description: Keep n8n updated with Docker Compose and back up your data and encryption
  key
percent: 84
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

Stage 6, part three: the butler stays alive. A homelab tool you never update becomes a risk, and one you never back up becomes a loss waiting to happen. This lesson gives you routines for both.

---

## Before every update

n8n's own advice is to update regularly, ideally at least once a month, so you never jump many versions at once. Before you do:

1. Read the release notes for breaking changes
2. Take a backup (below), including the encryption key

## Update with Docker Compose

From the folder with your `compose.yml`:

```bash
cd ~/n8n
docker compose pull
docker compose down
docker compose up -d
```

`pull` fetches the newest image, `down` stops and removes the old container, and `up -d` starts a new one. Your workflows survive because they live in the volume, not in the container.

Check the result with `docker compose logs --tail 20 n8n` and open the editor.

**Pinning a version.** With `image: n8nio/n8n` you get the latest stable release on each pull. If you'd rather choose when to move, set a tag, such as `image: n8nio/n8n:YOUR_VERSION`. The docs show tags like a specific number or `next`, and they warn that `next` is unstable. Pin when you want to update on purpose.

**Rolling back.** Putting an older tag back doesn't undo database changes made by a newer version. This is why you back up first: if an update goes wrong, restore the backup and start the older image.

## What to back up

| What | Why |
|---|---|
| The `.n8n` folder (our `n8n_data` volume) | Holds the SQLite database, settings and the encryption key |
| Exported workflows and credentials | A readable copy that survives a lost volume |
| Your `compose.yml` and `.env` | Your configuration, including any custom encryption key |
{:class="table table-single"}

With SQLite, stop n8n before you copy the data so the database file isn't copied half-written.

## Back up the volume

This copies the volume into a compressed file, using a throwaway container. Change `n8n_n8n_data` if `docker volume ls` shows another name.

```bash
cd ~/n8n
mkdir -p ~/n8n-backups
docker compose down
docker run --rm \
  -v n8n_n8n_data:/data:ro \
  -v ~/n8n-backups:/backup \
  alpine tar czf /backup/n8n-data-$(date +%F).tar.gz -C /data .
docker compose up -d
```

Then copy `~/n8n-backups` off the Pi, to another computer or a NAS. A backup on the same SD card isn't much of a backup.

## Export workflows and credentials as files

n8n has a command line tool. Run it inside the container:

```bash
docker compose exec n8n n8n export:workflow --backup --output=/home/node/.n8n/exports/workflows/
docker compose exec n8n n8n export:credentials --backup --output=/home/node/.n8n/exports/credentials/
```

Use a different output folder for each command. These exports cover workflows and credentials only, not users, execution history or settings, so they don't replace the volume backup. Credentials stay encrypted, and the encryption key is needed to restore them.

## Restore

For the full instance: stop n8n, restore the volume contents and your configuration (including any custom encryption key), then start n8n. For files, the docs use:

```bash
n8n import:workflow --separate --input=backups/workflows/
n8n import:credentials --separate --input=backups/credentials/
```

Set up the owner account on a fresh instance first, and note that imported workflows are deactivated by default, so publish the ones you want running.

## Try it Yourself

1. Do a practice backup now, then list the file with `ls -lh ~/n8n-backups`.
2. Update n8n with the three commands, and confirm your workflows are still there.
3. Challenge: write a cron job on the Pi (see the [Linux course](/learn/linux_intro/)) that does the backup each week, and have a second n8n workflow remind you to copy it off the Pi.

## Common Issues

**Problem**: After an update, n8n won't start.
**Solution**: Read `docker compose logs n8n`, then roll back by restoring your backup and pinning the previous version.
**Why**: Newer versions may change the database, and an older image can't read it.

**Problem**: Restored credentials show errors.
**Solution**: Make sure the encryption key from the original install is in place.
**Why**: Credentials can only be decrypted with the key that encrypted them.
