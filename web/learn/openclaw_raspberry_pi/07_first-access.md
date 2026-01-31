---
layout: lesson
title: First Access & Configuration
author: Kevin McAleer
type: page
cover: /learn/openclaw_raspberry_pi/assets/openclaw.jpg
date: 2026-01-31
previous: 06_docker-compose-setup.html
next: 08_configuring-openclaw.html
description: Access the OpenClaw dashboard and configure your setup.
percent: 40
duration: 2
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


## First Access & Configuration

Now that OpenClaw is running, let's access it and get it configured.

### Accessing the Dashboard

Open your browser and navigate to:

```
http://openclaw-pi.local:18789
```

Or your Pi's IP address:

```
http://192.168.1.100:18789
```

You should see the OpenClaw dashboard.

### Initial Configuration

#### Authentication

OpenClaw generates a security token on first run. You'll see something like:

```
Token: 8c4ccaa8e77b6bfdf25eeace15ac7da73498dc804a9589ec
```

Save this somewhere safe. You'll need it if you need to reconnect.

To set up a proper password:

1. Click **Settings** in the dashboard
2. Go to **Authentication**
3. Set a password you'll remember
4. Save

#### Add API Keys (Optional)

If you want to use Claude or other models:

1. Go to **Settings** → **API Keys**
2. Add your Anthropic API key (for Claude)
3. Add Brave Search API key (for web search)
4. Save

Without API keys, you're limited to local models only.

#### Select Skills

1. Go to **Skills** marketplace
2. Browse available skills
3. Install the ones you want

Start simple. Install:
- File system access
- Basic utilities
- One or two others

You can add more later.

### Your First Agent

Let's create your first agent:

1. Click **New Session**
2. Give it a name (e.g., "Test Agent")
3. Select a model
4. Click **Create**

### Test It

Send a simple message:

```
Say hello and tell me a little about yourself
```

The agent should respond. If it does, everything is working!

### Dashboard Overview

**Main areas:**

- **Sessions:** Your active agent conversations
- **Skills:** Available tools for agents
- **Settings:** Configuration and security
- **Workspace:** File management
- **Gateway Health:** System status

### Next Steps

Your OpenClaw is now running and configured. Next lesson covers managing your container and monitoring resources.

---
