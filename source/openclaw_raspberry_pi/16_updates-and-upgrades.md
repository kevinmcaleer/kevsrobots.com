---
title: Updates and Upgrades
description: Upgrade Pi Butler safely by swapping Docker images, pin a version, roll back when needed and keep Raspberry Pi OS up to date
layout: lesson
type: page
cover: /learn/openclaw_raspberry_pi/assets/banners/16_updates-and-upgrades.jpg
date_updated: 2026-11-22
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

OpenClaw moves quickly, and Raspberry Pi OS gets security fixes all the time. By the end of this lesson you will be able to upgrade Pi Butler in a few safe steps, pin it to a version you trust, roll back if an upgrade goes wrong, and keep the Pi's own OS patched.

---

## How upgrades work in Docker

This is where running OpenClaw in Docker pays off. Your state lives in folders on the Pi, mounted into the container. The program lives in the image. So an upgrade is just: pull a newer image, then replace the container. Your config, memory and chats stay exactly where they are.

The `openclaw update` command you may see in the docs is for native installs. In Docker we swap images instead.

You also don't need to run a separate repair step. Each time the container starts with a new image, its entrypoint runs `openclaw doctor --fix --non-interactive` before the Gateway starts, which handles config and database migrations.

```mermaid
flowchart LR
  A["Back up"] --> B["git pull"]
  B --> C["Pull new images"]
  C --> D["Stop the Gateway"]
  D --> E["Start the Gateway"]
  E --> F["Check with doctor and status"]
```

---

## Release channels and tags

The official images live on GitHub's container registry at `ghcr.io/openclaw/openclaw`, with a mirror on Docker Hub at `openclaw/openclaw`. Both publish arm64 builds for the Pi. The tag after the colon chooses which release you get.

| Tag | What it gives you |
| --- | --- |
| `latest` | The current stable release. This course uses it |
| `extended-stable` | A trailing release that moves about once a month, for a quieter life |
| A version, such as `2026.11.6` | Exactly that release, and it never changes |
| A beta, such as `2026.9.1-beta.1` | A prerelease for testing, not for Pi Butler |
| `slim` and `-browser` variants | A smaller image, or one with Chromium included |
{:class="table table-single"}

The moving tags such as `latest` are also rebuilt every week to pick up operating system security fixes inside the image. So pulling `latest` now and then is worthwhile even when there is no new OpenClaw version.

---

## The upgrade routine

Every `docker compose` command here is typed **on the Pi**, in `~/openclaw`. The `openclaw` commands use the alias from [lesson 7](/learn/openclaw_raspberry_pi/07_first-access.html).

### 1. Note the version and back up

Write down the version you are on now, so you know where to roll back to:

```bash
openclaw --version
```

Then take a verified backup, exactly as in [lesson 15](/learn/openclaw_raspberry_pi/15_backup-and-recovery.html):

```bash
~/openclaw-backup.sh
```

Don't skip this. Upgrades can migrate the databases, and a backup made with the old version is your only way back.

### 2. Update the checkout

```bash
cd ~/openclaw
git pull --ff-only
```

This keeps the compose file and scripts in step with the new image. `--ff-only` refuses to do anything clever if you have changed tracked files. Your `.env` and `docker-compose.override.yml` are ignored by git, so they are safe.

### 3. Pull the new images

```bash
docker compose pull openclaw-gateway openclaw-cli
```

### 4. Stop, then start

```bash
docker compose stop openclaw-gateway
docker compose up -d openclaw-gateway
```

The docs are clear about the order: stop the old container **before** starting the new one. The startup repair needs the state folder to itself, and only one Gateway should ever use a state folder at a time.

Watch it come up:

```bash
docker compose logs -f openclaw-gateway
```

Press Ctrl+C once it settles.

### 5. Check it

```bash
openclaw doctor --json
openclaw status
openclaw --version
```

`doctor --json` is a read-only report. Look for `"ok": true` and an empty or harmless list of findings. Then send Pi Butler a Telegram message to be sure.

---

## Pinning a version

`latest` moves whenever a release lands. If you would rather choose when Pi Butler changes, pin it to a version tag. Edit `~/openclaw/.env` **on the Pi**:

```bash
nano ~/openclaw/.env
```

Change the `OPENCLAW_IMAGE` line to a specific version:

```ini
OPENCLAW_IMAGE=ghcr.io/openclaw/openclaw:2026.11.6
```

Then apply it. Remember, a plain restart does not read `.env` changes:

```bash
cd ~/openclaw
docker compose up -d openclaw-gateway
```

To upgrade a pinned Pi Butler, change the version in `.env` and run the same routine. The `openclaw` alias reads the same `.env`, so the CLI always matches the Gateway.

One caution: if you ever run `./scripts/docker/setup.sh` again, it rewrites `.env` from your current shell. Export `OPENCLAW_IMAGE` first, or your pin is lost.

---

## Rolling back

If an upgrade misbehaves, rolling back takes **two** things: the old image tag, and the backup you made just before upgrading.

Why both? Upgrades can migrate the databases forward, and an older OpenClaw may not read them. Going back to the old image alone is not enough.

1. Stop the Gateway with `docker compose stop openclaw-gateway`.
2. Set `OPENCLAW_IMAGE` in `.env` to the version you wrote down in step 1.
3. Restore the pre-upgrade backup and activate it, following the restore steps in lesson 15.
4. Start with `docker compose up -d openclaw-gateway`, then check `openclaw status`.

If you used the cold copy from lesson 15 instead, unpack it in place of `~/.openclaw` while the Gateway is stopped. Then start the old version.

---

## Keeping the Pi's OS updated

OpenClaw is only half the story. The Pi's own operating system, Docker included, needs updates too. **On the Pi**:

```bash
sudo apt update && sudo apt full-upgrade
sudo reboot
```

`update` refreshes the list of packages, and `full-upgrade` installs the updates, including ones that add or remove packages. Rebooting loads any new kernel and firmware. Pi Butler comes back on its own after the reboot, because of the restart policy, which we test properly in [lesson 17](/learn/openclaw_raspberry_pi/17_production-setup.html).

---

## How often?

| Job | How often | Why |
| --- | --- | --- |
| `sudo apt update && sudo apt full-upgrade` | Weekly | Security fixes for the OS and Docker |
| Pull `latest` and recreate the Gateway | Every week or two | New releases and weekly image security rebuilds |
| Read the release notes before a big jump | Every upgrade | Spot changes that affect your setup |
| Check `openclaw doctor --json` | After every upgrade | Catch problems straight away |
| Test a rollback | Once, so you know it works | A plan you have never tried is a guess |
{:class="table table-single"}

If you pin a version, swap the second row for "when a release has something you want".

> **Teachers**: schedule upgrades between projects, not the night before a lesson. Pinning every classroom Pi to the same version tag means everyone sees the same menus and messages.

---

## Pi Butler progress

Pi Butler now **stays up to date**. You can upgrade it in five safe steps, pin it to a version, roll back with a matching backup, and keep its OS patched.

---

## Try it Yourself

1. **Find your version.** Run `openclaw --version` and `docker compose images`. Which tag and image are you running?
2. **Do a routine upgrade.** Work through all five steps, even if no new release is out. Pulling `latest` may still bring the weekly security rebuild.
3. **Challenge:** Pin Pi Butler to the version it is on now, apply it with `docker compose up -d openclaw-gateway`, and prove with `openclaw --version` and `docker compose images` that the pin took effect.

---

## Common Issues

**Problem**: I changed `OPENCLAW_IMAGE` in `.env` but the version did not change.
**Solution**: Run `docker compose up -d openclaw-gateway`, not `docker compose restart`.
**Why**: Restart reuses the existing container and its settings. `up -d` recreates it with the new `.env` values.

**Problem**: `git pull --ff-only` refuses to run.
**Solution**: Run `git status` to see which tracked file you changed, then put it back with `git checkout -- <file>`, keeping a copy if you need it.
**Why**: `--ff-only` will not merge your local edits. Put your own changes in `.env` or `docker-compose.override.yml`, which git ignores.

**Problem**: After rolling back to an older image, the Gateway refuses to start.
**Solution**: Restore the backup you made before the upgrade, with the Gateway stopped.
**Why**: The newer version migrated the databases, and the older version cannot read them.

---

## Check your understanding

{% quiz %}
? What is the right order for the last two upgrade steps?
- [ ] Start the new container, then stop the old one
- [x] `docker compose stop openclaw-gateway`, then `docker compose up -d openclaw-gateway`
- [ ] `docker compose restart openclaw-gateway`, which pulls the new image
> Only one Gateway should use a state folder at a time, so stop the old one first. Restart does not change the image.

? Why do you need a pre-upgrade backup to roll back?
- [ ] The old image deletes your config
- [x] Upgrades can migrate the databases, which an older version may not read
- [ ] Docker deletes the old image straight after pulling
> Rolling back needs the old image tag and the matching state from before the upgrade.

? Which command keeps Raspberry Pi OS itself up to date?
- [ ] `docker compose pull`
- [ ] `openclaw update`
- [x] `sudo apt update && sudo apt full-upgrade`
> Docker images update OpenClaw. The Pi's own OS and Docker engine are updated with apt.
{% endquiz %}
