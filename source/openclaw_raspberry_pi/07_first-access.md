---
title: First Access & Configuration
description: Access the OpenClaw dashboard and configure your setup.
layout: lesson
type: page
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
