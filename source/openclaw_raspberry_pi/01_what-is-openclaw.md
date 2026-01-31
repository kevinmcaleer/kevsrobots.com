---
title: What is OpenClaw?
description: Learn about OpenClaw, what it does, and why it's powerful.
layout: lesson
type: page
---

## What is OpenClaw?

OpenClaw is an open-source framework for building and running AI agents on your own machine. Unlike cloud-based AI services, OpenClaw is designed for local-first operation, giving you complete control over your data and privacy.

### Core Concepts

#### Agents

An agent is an AI system that can:

- **Understand** your requests and respond intelligently
- **Take actions** like reading files, running commands, or calling APIs
- **Learn and remember** previous interactions
- **Use tools** to extend its capabilities

#### Skills

Skills are like plugins. They extend what an agent can do. OpenClaw comes with core skills, and you can add more from the skill marketplace (ClawdHub).

Examples of skills:
- File system access
- Web search
- Smart home control
- Email management
- Calendar integration

#### The Gateway

The gateway is the central hub that:

- Manages your agents
- Handles authentication
- Provides the web dashboard
- Routes messages between you and agents

### Why OpenClaw?

#### Privacy

Your data stays on your machine. Nothing is sent to a cloud service. You own everything.

#### Control

You decide what tools your agent can access. You can sandbox it completely if you want.

#### Offline Operation

You don't need constant internet access. Run local models completely offline if you prefer.

#### Cost

No API calls, no per-message billing. Run as many agents as your hardware can handle.

#### Customization

It's open source. Modify it, extend it, make it yours.

### How It Works

```
┌─────────────┐
│   You       │
│  (via web)  │
└──────┬──────┘
       │
       ▼
┌─────────────────┐
│   Gateway       │
│ (orchestrator)  │
└────────┬────────┘
         │
    ┌────┴────┐
    ▼         ▼
┌──────┐  ┌──────┐
│Agent1│  │Agent2│
└──────┘  └──────┘
```

You interact with the gateway via the web dashboard. The gateway manages your agents. Each agent can use available skills to accomplish tasks.

### Local vs Cloud

| Aspect | OpenClaw (Local) | Cloud AI Services |
|--------|------------------|-------------------|
| **Privacy** | Your data stays local | Sent to cloud servers |
| **Cost** | One-time hardware | Per-message billing |
| **Latency** | Fast local operation | Network dependent |
| **Control** | Complete control | Limited customization |
| **Uptime** | Your responsibility | Provider's responsibility |

### When to Use OpenClaw

**Good for:**
- Personal AI assistants
- Automation and scripting
- Learning AI concepts
- Privacy-sensitive applications
- Running 24/7 without API costs

**Not ideal for:**
- Massive scale (thousands of users)
- Guaranteed 99.9% uptime SLAs
- Bleeding-edge models (you're limited to what runs on your hardware)

### Architecture Overview

OpenClaw runs on Node.js and includes:

- **Web UI** - Browser-based control panel
- **Gateway Service** - Core orchestration
- **Agent Engine** - Runs individual agents
- **Skill System** - Extensible tool framework
- **Workspace** - Your configuration and data

The beauty is: all of this runs on your Raspberry Pi. No cloud required.

### Next Steps

Now that you understand what OpenClaw is, let's look at the hardware requirements for running it on a Raspberry Pi.

---
