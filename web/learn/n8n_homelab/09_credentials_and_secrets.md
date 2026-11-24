---
layout: lesson
title: Credentials and Secrets
author: Kevin McAleer
type: page
cover: /learn/n8n_homelab/assets/banners/09_credentials_and_secrets.jpg
date: 2026-11-24
previous: 08_tidy_data_with_the_code_node.html
next: 10_executions_and_debugging.html
description: Keep passwords and keys out of your workflows with n8n credentials, and
  look after the encryption key
percent: 70
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

Stage 6, part one: the butler learns to keep a secret. By now you hold an SMTP login and a webhook key. Where those live decides whether your homelab is safe.

---

## Use credentials, not pasted values

n8n stores logins as **credentials**. You can create one from the **Create** button in the side menu, or from inside any node's credential drop-down, as you did for email. A credential holds the secret once, and every node that needs it just points at it.

**Why this matters:**

- Secrets stay out of node fields, exported workflow files and screenshots
- Changing a password means editing one credential, not ten nodes
- n8n encrypts credentials before saving them to its database

n8n names a new credential after the node, like "SMTP account". Rename it to say what it is for, such as `SMTP - butler alerts`.

## Limit where a credential can go

On credentials used by the HTTP Request node, **Allowed HTTP Request Domains** lets you choose **All**, **Specific Domains** or **None**. Limiting a credential to the domain it belongs to stops a mistake from sending it somewhere else.

## The encryption key

n8n creates a random encryption key on first launch and saves it in its settings file, inside `/home/node/.n8n`, so it is already in your volume. You can set your own with the `N8N_ENCRYPTION_KEY` environment variable instead.

The important rule: **without that key, credentials can't be decrypted.** If you restore a backup of the database but lose the key, every saved login is useless. Lesson 11 backs it up.

To set your own, add a line to the `environment` list in `compose.yml`:

```yaml
    environment:
      - N8N_ENCRYPTION_KEY=${N8N_ENCRYPTION_KEY}
```

and put the value in a file named `.env` next to it:

```
N8N_ENCRYPTION_KEY=CHANGE_ME_TO_A_LONG_RANDOM_STRING
```

Generate the value with `openssl rand -hex 32`. Do this **before you save any credentials**, since changing the key later means n8n can no longer read credentials stored under the old one. If you already have credentials, leave the generated key alone.

## Keep secrets out of Git

If you put `compose.yml` in a Git repository, keep `.env` out of it:

```bash
echo ".env" >> .gitignore
chmod 600 .env
```

`chmod 600` means only your user can read the file. See the [Linux course on file permissions](/learn/linux_intro/05_file_permissions.html).

## Secrets on the Pico

The Pico's shared key is in plain text in `main.py`, which is fine for a home project on your own network, but you shouldn't post that file anywhere. Replace the key if you ever share the code, and use a different key per device so you can revoke one.

## Try it Yourself

1. Open the SMTP credential. Notice that n8n doesn't show the saved password again.
2. Rename your two credentials so you can tell them apart in six months.
3. Challenge: make a second Header auth credential for a second Pico, and add a second Webhook path that uses it.

## Common Issues

**Problem**: After a restore, every node says its credential can't be decrypted.
**Solution**: Restore the original encryption key, from the backed-up settings file or your `N8N_ENCRYPTION_KEY`.
**Why**: Credentials are encrypted with that key.

**Problem**: I changed `N8N_ENCRYPTION_KEY` and my credentials stopped working.
**Solution**: Put the old key back. To change it safely, follow the key-rotation page in the n8n docs.
**Why**: Stored credentials were encrypted with the old key.
