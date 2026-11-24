---
title: Install n8n with Docker Compose
description: Install n8n on a Raspberry Pi with a small Docker Compose file, a data volume and the right timezone
layout: lesson
type: page
cover: /learn/n8n_homelab/assets/banners/02_install_n8n_with_docker_compose.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Let's get the butler a home. We will run n8n in a container with Docker Compose, so it restarts after a reboot and keeps its data in a volume.

If Docker or Compose files are new to you, the [Docker](/learn/docker/) course covers them, including [volumes](/learn/docker/07_volumes.html) and [Docker Compose](/learn/docker/08_docker-compose.html).

---

## The compose file

The official image is `n8nio/n8n`. It listens on port `5678`, and keeps everything in `/home/node/.n8n`, which is where we mount a volume. Make a folder on the Pi and create the file:

```bash
mkdir -p ~/n8n && cd ~/n8n
nano compose.yml
```

```yaml
services:
  n8n:
    image: n8nio/n8n
    container_name: n8n
    restart: unless-stopped
    ports:
      - "5678:5678"
    environment:
      - GENERIC_TIMEZONE=Europe/London
      - TZ=Europe/London
      - N8N_ENFORCE_SETTINGS_FILE_PERMISSIONS=true
    volumes:
      - n8n_data:/home/node/.n8n

volumes:
  n8n_data:
```

**What each part does:**

| Part | What it does |
|---|---|
| `image: n8nio/n8n` | The official n8n image |
| `restart: unless-stopped` | Starts n8n again after a reboot or a crash |
| `5678:5678` | Makes n8n reachable on port 5678 of the Pi |
| `GENERIC_TIMEZONE` | The timezone used by schedule-based nodes |
| `TZ` | The container's own system timezone |
| `N8N_ENFORCE_SETTINGS_FILE_PERMISSIONS=true` | Keeps n8n's settings file locked down |
| `n8n_data:/home/node/.n8n` | A named volume holding your workflows, credentials and encryption key |
{:class="table table-single"}

Change both timezone values to yours, such as `America/New_York`. This matters more than it looks: the docs say the default is `America/New_York`, so a schedule you set for 07:00 would fire at the wrong time if you forget.

**A note on the official guide.** n8n's own Docker Compose guide now installs a larger stack with extra services. This file is a small, single-container setup built from the image, port, volume and variables in n8n's Docker page, and it is plenty for a homelab. For anything bigger, such as PostgreSQL, see the official guide.

## Start it

```bash
docker compose up -d
docker compose ps
docker compose logs -f n8n
```

Press `Ctrl+C` to leave the logs. Then, from any computer on your network, open:

```
http://YOUR_PI_IP:5678
```

Find your Pi's address with `hostname -I`. n8n shows a setup screen where you create the owner account. Pick a strong password: this account can reach everything the butler can.

## Keep it private

**Do not forward port 5678 on your router.** At this point n8n is served over plain HTTP, so anyone who can reach it can try to log in, and your password would travel unencrypted. Keep it on your home network. If you ever need outside access, put it behind a reverse proxy with HTTPS and a login, or use a VPN. Lesson 12 mentions one setting you will need when you do.

## Health check

n8n has a built-in health endpoint. From the Pi:

```bash
curl -sf http://localhost:5678/healthz && echo "n8n is up"
```

We will point the butler at this exact URL in lesson 4.

## Try it Yourself

1. Run `docker compose down`, then `docker compose up -d`. Is your owner account still there? Why?
2. Run `docker volume ls` and find the volume. It is named after your folder, such as `n8n_n8n_data`.
3. Challenge: reboot the Pi and check that n8n comes back by itself.

## Common Issues

**Problem**: The page won't load from my laptop.
**Solution**: Run `docker compose ps` to check the container is running, use the Pi's IP rather than `localhost`, and check that no firewall blocks port 5678.
**Why**: `localhost` on your laptop means your laptop, not the Pi.

**Problem**: n8n restarts in a loop with a permissions error, and I used a folder on the Pi instead of a named volume.
**Solution**: Give the folder to user 1000: `sudo chown -R 1000:1000 /home/YOUR_USER/n8n`.
**Why**: The container runs as the `node` user, which owns files as user 1000. A named volume avoids this.

## Check your understanding

{% quiz %}
? Which folder inside the container must be kept in a volume so your work survives an update?
- [ ] `/etc/n8n`
- [x] `/home/node/.n8n`
- [ ] `/var/www`
> That folder holds your database, your workflows, your credentials and the encryption key.

? Why set `GENERIC_TIMEZONE`?
- [ ] It makes n8n start faster
- [ ] It lets n8n talk to the internet
- [x] Schedule-based nodes use it, and the default is America/New_York
> A 07:00 schedule fires at 07:00 in that timezone, so set it to yours.

? Should you forward port 5678 from your router to the internet?
- [ ] Yes, so you can reach it from anywhere
- [x] No, not without HTTPS and a login in front of it
> n8n here speaks plain HTTP, and it can reach everything in your homelab.
{% endquiz %}
