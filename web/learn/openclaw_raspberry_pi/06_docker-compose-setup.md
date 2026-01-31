---
layout: lesson
title: Docker Compose Setup
author: Kevin McAleer
type: page
cover: /learn/openclaw_raspberry_pi/assets/openclaw.jpg
date: 2026-01-31
previous: 05_installing-docker.html
next: 07_first-access.html
description: Configure and start OpenClaw using Docker Compose.
percent: 35
duration: 4
navigation:
- name: OpenClaw on Raspberry Pi
- content:
  - section: Getting Started
    content:
    - name: Course Overview
      link: 00_intro.html
    - name: What is OpenClaw?
      link: 01_what-is-openclaw.html
    - name: Hardware Requirements
      link: 02_hardware-requirements.html
    - name: Why Docker?
      link: 03_why-docker.html
  - section: Installation & Setup
    content:
    - name: Preparing Your Raspberry Pi
      link: 04_preparing-your-pi.html
    - name: Installing Docker
      link: 05_installing-docker.html
    - name: Docker Compose Setup
      link: 06_docker-compose-setup.html
    - name: First Access & Configuration
      link: 07_first-access.html
  - section: Configuration & Usage
    content:
    - name: Configuring OpenClaw
      link: 08_configuring-openclaw.html
    - name: Your First Agent
      link: 09_first-agent.html
    - name: Understanding Skills
      link: 10_understanding-skills.html
  - section: Management & Troubleshooting
    content:
    - name: Managing Containers
      link: 11_managing-containers.html
    - name: Monitoring Resources
      link: 12_monitoring-resources.html
    - name: Troubleshooting
      link: 13_troubleshooting.html
  - section: Advanced Topics
    content:
    - name: Security Best Practices
      link: 14_security.html
    - name: Backup and Recovery
      link: 15_backup-and-recovery.html
    - name: Updates and Upgrades
      link: 16_updates-and-upgrades.html
    - name: Production Setup
      link: 17_production-setup.html
  - section: Summary
    content:
    - name: Next Steps
      link: 18_next-steps.html
---


## Docker Compose Setup

Now we'll set up OpenClaw using Docker Compose. This is where it all comes together.

### Step 1: Create the OpenClaw Directory

```bash
mkdir ~/openclaw
cd ~/openclaw
```

This is where your OpenClaw configuration and data will live.

### Step 2: Create the docker-compose.yml File

In your `~/openclaw` directory, create a file called `docker-compose.yml`:

```bash
nano docker-compose.yml
```

### Step 3: Paste This Configuration

Copy and paste this entire configuration:

```yaml
version: '3.8'

services:
  openclaw:
    image: openclaw/openclaw:latest
    container_name: openclaw
    ports:
      - "18789:18789"
    volumes:
      - ./workspace:/root/.openclaw/workspace
      - ./data:/data
    environment:
      - NODE_ENV=production
    restart: unless-stopped
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:18789/health"]
      interval: 30s
      timeout: 10s
      retries: 3
```

### Step 4: Save the File

- Press **Ctrl+X**
- Press **Y** (yes)
- Press **Enter**

### Understanding the Configuration

Let's break down what this does:

#### `version: '3.8'`
Docker Compose format version. This is the current standard.

#### `image: openclaw/openclaw:latest`
Use the latest OpenClaw image from Docker Hub.

#### `container_name: openclaw`
Name this container "openclaw" for easy reference.

#### `ports:`
```yaml
- "18789:18789"
```
Forward host port 18789 to container port 18789. This is where the web dashboard runs.

Format: `host-port:container-port`

#### `volumes:`
```yaml
- ./workspace:/root/.openclaw/workspace
- ./data:/data
```

These connect your Pi's folders to folders inside the container:
- `./workspace` on your Pi ↔ `/root/.openclaw/workspace` in container
- `./data` on your Pi ↔ `/data` in container

**Important:** This is how your data persists. Even if the container is deleted, these folders survive.

#### `environment:`
```yaml
- NODE_ENV=production
```
Run Node.js in production mode for better performance.

#### `restart: unless-stopped`
Docker automatically restarts the container if it crashes. Unless you explicitly stop it.

#### `healthcheck:`
```yaml
test: ["CMD", "curl", "-f", "http://localhost:18789/health"]
interval: 30s
timeout: 10s
retries: 3
```
Every 30 seconds, check if OpenClaw is healthy by hitting its health endpoint.

If unhealthy, restart after 3 failed checks.

### Step 5: Create the Workspace Directory

```bash
mkdir ~/openclaw/workspace
mkdir ~/openclaw/data
```

Docker will create these if they don't exist, but it's good practice to create them first.

### Step 6: Start OpenClaw

```bash
cd ~/openclaw
docker-compose up -d
```

**What `-d` means:** Detached mode (runs in background).

### Step 7: Watch It Start

First time takes longer (Docker downloads the image). Watch the progress:

```bash
docker-compose logs -f
```

You'll see output like:
```
openclaw | Starting OpenClaw gateway...
openclaw | Gateway listening on port 18789
openclaw | Dashboard ready at http://127.0.0.1:18789
```

Once you see "ready" messages, press **Ctrl+C** to stop following logs.

### Step 8: Verify It's Running

```bash
docker-compose ps
```

Should show:
```
NAME        COMMAND                  SERVICE    STATUS          PORTS
openclaw    "node index.js"           openclaw   Up 10 seconds   0.0.0.0:18789->18789/tcp
```

**Key fields:**
- **STATUS:** Should show "Up" (not "Exited")
- **PORTS:** Shows the port forwarding is working

### Step 9: Check the Logs

If status shows "Exited", something went wrong. Check logs:

```bash
docker-compose logs --tail 50
```

Look for error messages. Common issues:

**"port already in use":**
Something else is using port 18789. Change it in docker-compose.yml:
```yaml
ports:
  - "18790:18789"
```

Then `docker-compose restart`.

**"out of memory":**
Your Pi is running other things. Close them or increase available memory.

**"cannot connect":**
Wait another 10 seconds and retry. First startup takes time.

### Step 10: Access the Dashboard

From your computer, open a browser to:

**If on the same network:**
```
http://openclaw-pi.local:18789
```

Or:
```
http://192.168.1.100:18789
```

(Replace with your Pi's IP)

**If accessing from outside your network:**
Use an SSH tunnel:

```bash
ssh -N -L 18789:127.0.0.1:18789 kev@openclaw-pi.local
```

Then open:
```
http://localhost:18789
```

### Common Tasks

#### Stop OpenClaw

```bash
docker-compose down
```

This stops the container and removes it. Your data in `workspace/` survives.

#### Restart OpenClaw

```bash
docker-compose restart
```

Restart without stopping and removing.

#### View Live Logs

```bash
docker-compose logs -f
```

Press Ctrl+C to stop following.

#### Update to Latest Version

```bash
docker-compose pull
docker-compose up -d
```

### Troubleshooting

#### Container won't start

Check the logs:
```bash
docker-compose logs
```

Common causes:
- Port 18789 in use
- Permission issues on volumes
- Not enough disk space

#### Can't access dashboard

Check firewall:
```bash
sudo ufw status
```

Check port is listening:
```bash
lsof -i :18789
```

#### Want to start fresh?

```bash
docker-compose down -v
```

The `-v` flag removes volumes too (deletes your workspace).

**⚠️ Warning:** This deletes all your OpenClaw data!

### You're Ready!

OpenClaw is running in Docker on your Pi. Next, let's access it and do initial configuration.

---
