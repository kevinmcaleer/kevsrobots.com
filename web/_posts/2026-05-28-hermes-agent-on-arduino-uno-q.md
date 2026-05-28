---
title: "Why I'm Running Hermes Agent on an Arduino Uno Q"
description: >-
  I switched from OpenClaw to Hermes Agent — and I'm running it on an Arduino Uno Q instead of a Raspberry Pi. Here's why: cost, persistent learning, remote access via Tailscale, and an agent that gets smarter every session.
excerpt: >-
  One board. One agent. Always on. Here's how I moved from OpenClaw to Hermes Agent, why I picked the Arduino Uno Q to run it on, and what it actually costs compared to a VPS.
layout: showcase
mode: light
date: 2026-05-28
author: Kevin McAleer
difficulty: intermediate
cover: /assets/img/blog/hermes_uno_q/cover.jpg
hero: /assets/img/blog/hermes_uno_q/hero.png
tags:
  - hermes agent
  - arduino uno q
  - ai agent
  - tailscale
  - self-hosting
  - openclaw
  - raspberry pi
  - maker
  - agentic ai
groups:
  - arduino
  - ai
videos:
  - TBD
---

Ahoy there Makers,

I've been running an AI agent on a small board at my desk for a few weeks now. Not a Raspberry Pi — an Arduino Uno Q. Four gigabytes of RAM, a Qualcomm AI chip, costs about fifty-three euros. And the agent running on it remembers everything. It learns from every session. It builds its own skills as it goes.

If you've watched my [OpenClaw video](#), you know I've been deep into self-hosted AI agents. This is the next step. In this post I want to explain why I moved from OpenClaw to a tool called **Hermes Agent**, why the Arduino Uno Q is the right hardware for it, and walk through the full setup so you can do the same.

---

## What is Hermes Agent?

[Hermes Agent](https://github.com/NousResearch/hermes-agent) is an open-source AI agent framework from Nous Research. Think of it as being in the same category as OpenClaw or Claude Code — an autonomous agent that can run terminal commands, read and write files, browse the web, write code, and do all of that in a loop until the task is done.

But there are three things that make it genuinely different from other agents, and they're the things that actually matter in day-to-day use.

### 1. Skills

Hermes builds a library of reusable procedures as it works. When it solves a complex problem — setting up a server, debugging a tricky piece of code, figuring out a workflow — it can save that as a *skill document*. Next time it needs to do something similar, it loads that skill and doesn't start from scratch.

It also tags skills with their provenance. Skills it creates itself are marked as agent-created. Skills you write manually are separate. You always know what came from where, and you can review, edit, or pin any of them.

### 2. Persistent Memory

Hermes has persistent memory across sessions. It remembers who you are, your project structure, your preferences, your environment setup. It's not starting from zero every time you open a terminal. That context compounds — the more you use it, the more useful it becomes.

### 3. Provider Flexibility

Hermes works with basically any LLM provider — Anthropic, OpenAI, Google Gemini, GitHub Copilot, DeepSeek, and more. If you've already got a subscription to any of these, you can plug it straight in. No extra API cost, no new accounts.

---

## Why I Moved Away from OpenClaw

OpenClaw is genuinely impressive. The multi-agent architecture, the way it handles long-running tasks — it's well thought through. But after living with it for a while, two things started to bother me.

### Cost

OpenClaw has a large system prompt — we're talking ten thousand tokens or more just to start a session. Every heartbeat, every background check, every small task burns through tokens. The default setup, running all Opus models, comes out around **£430 a month**. Even with optimisation — routing heartbeats to cheaper models, being selective about what runs — you're still looking at a significant ongoing cost.

Hermes uses Anthropic prompt caching to reduce costs on multi-turn conversations. The system prompt is cached at the provider level, so you don't pay full input token rates on every turn. For a persistent agent running all day, this compounds meaningfully.

### The Security Model

OpenClaw has a concept called ClawHub — it's where skills and integrations come from. The issue is that you're pulling in skills from a shared registry, and if you're not careful, you're trusting third-party code to run with agent-level access on your machine. OpenClaw's own documentation flags this — running agents is risky, harden your setup.

With Hermes, the skills your agent builds come from your own sessions. Generated from what you've actually done, in your actual environment. There's no external registry pulling in code you didn't write. For a device sitting on my home network, that matters.

---

## Why the Arduino Uno Q?

The Arduino Uno Q 4GB version costs around **€53**. The 2GB version is €39. Compare that to a Raspberry Pi 5 with 4GB — you're looking at around £60–70 by the time you add a case, power supply, and SD card. The Uno Q comes with **32GB of eMMC storage built in**. No SD card needed. Faster and more reliable.

Under the hood it's got a Qualcomm Dragonwing QRB2210 processor — that's the MPU side, the Linux system. And it's got an STM32 microcontroller built in as well. Two computers on one board. The Linux side runs Python, runs Hermes, connects to the network. The microcontroller side handles sensors, motors, and real-time tasks. That split is useful if you're also building robots or automation projects.

In benchmarks, the Uno Q sits at around **40,000** sysbench single-thread score. A Raspberry Pi 4 is about 68,000. A Pi 500 is around 105,000. The Pi is faster in raw compute — but for an agent that's mostly waiting on API calls and doing file operations, raw CPU speed isn't the bottleneck.

The deeper reason I chose it: I run sessions on lots of different machines — my laptop, my desktop, Pis here and there. Every time I start a new session on a different machine, the learning from the previous one is gone. Having one Hermes instance that **lives permanently on a dedicated board** — always on, always accumulating context — is a fundamentally better design.

---

## SBC vs VPS — The Real Cost Comparison

The obvious alternative to a dedicated board is a VPS — rent a small cloud machine, install Hermes, done. For some people that makes sense. But let's look at what it actually costs.

A decent entry-level VPS — two cores, 2GB RAM — runs around **£5–8 a month** depending on the provider. Hetzner is cheaper, Digital Ocean a bit more. That's **£60–96 a year. Every year. Indefinitely.**

The Uno Q 4GB costs €53. One purchase. No subscription. At £5/month VPS, the Uno Q pays for itself in **under 11 months**. At £8/month, you're break-even in under 7. And unlike a VPS, if you ever stop using it as an agent host — it's still a capable SBC. It goes into your next project. A robot, a sensor node, a home automation controller. It's not sunk cost, it's hardware.

A Raspberry Pi 5 tells the same story. Around £60–70 all-in, break-even in 8–12 months versus a VPS. At the end of its life as an agent host, it's a Pi.

The VPS argument is: I'm paying for convenience and never have to think about hardware. That's fair. But if you're a maker — if you've got a Pi or an Arduino sitting in a drawer right now — the economics are pretty clear. Self-hosted wins on cost, and you get a physical asset.

---

## Remote Access with Tailscale

A dedicated always-on agent is only useful if you can actually reach it from wherever you are. That's where [Tailscale](https://tailscale.com) comes in.

Tailscale is a free VPN built on WireGuard. You install it on the Uno Q and on whatever device you want to connect from — your laptop, your phone, anything — and they become part of the same private network. No port forwarding. No firewall rules. No exposing anything to the public internet.

Your devices each get a stable private IP on the Tailscale network. You SSH into the Uno Q from anywhere in the world, and it's as if they're sitting next to each other on the same desk.

It's free for personal use, up to 100 devices. There's nothing to self-host — Tailscale handles the coordination server. Actual traffic goes peer-to-peer. For a home lab setup, it's one of the best tools out there.

---

## Setting It Up — The Full Walkthrough

Here's exactly what I did to get Hermes running on the Uno Q from a clean board.

### Step 1: Install Tailscale

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
```

That's it. It'll give you a URL to authenticate — open that on your phone or laptop, log in, and your Uno Q appears in your Tailscale network. From this point you can SSH into it from anywhere.

### Step 2: Install Hermes

```bash
curl -fsSL https://raw.githubusercontent.com/NousResearch/hermes-agent/main/scripts/install.sh | bash
```

Once that's done, run the setup wizard:

```bash
hermes setup
```

This walks you through picking your model provider. I'm using GitHub Copilot — I've already got a Pro Plus subscription, so there's no extra API cost. Hermes supports Anthropic, OpenAI, Google Gemini, GitHub Copilot, and more. If you've already got a subscription, you can use it straight away. No new accounts, no new billing.

Pick your provider, and you're done. Then just type `hermes` to start a session.

### Step 3: Check Resource Usage

There's a concern some people have — that running an AI agent on a small board is going to hammer it. Peg the CPU, eat all the RAM. Let's actually look at that.

Open a second terminal and run `htop`. You'll see CPU sitting around **2–5% during normal idle**. RAM usage is low. Hermes is a Python process, but it's not loading a model locally — it's making API calls over the network. The heavy lifting happens on the cloud provider's hardware, not here.

That's the key insight: **the Uno Q isn't running the AI — it's running the agent.** The agent is the orchestration layer. It reads files, runs commands, calls tools, manages skills. The actual inference is someone else's problem. Which means the hardware requirements are genuinely tiny.

### Step 4: Install tmux

```bash
sudo apt install tmux
tmux new -s hermes
```

tmux is a terminal multiplexer. It keeps sessions alive even if your SSH connection drops. Now Hermes runs inside that tmux session. Detach from it, close your SSH connection, come back later — it's still there. Reattach with:

```bash
tmux attach -t hermes
```

For a persistent agent that's supposed to be always-on, that's pretty much essential.

### Step 5 (Optional): Install Obsidian

I keep all my notes in Obsidian, so I installed it on the Uno Q too. The Uno Q is ARM64, and Obsidian distributes as an AppImage, so the process is slightly more involved:

```bash
sudo apt update
sudo apt-get install libfuse2
wget https://github.com/obsidianmd/obsidian-releases/releases/download/v1.12.7/Obsidian-1.12.7-arm64.AppImage
chmod +x Obsidian-1.12.7-arm64.AppImage
./Obsidian-1.12.7-arm64.AppImage &
```

The `libfuse2` package is the important one — AppImages need it on modern Ubuntu. Without it you'll get a confusing error about FUSE not being available. Install that first, and it runs fine.

---

## Memory and Skills in Action

After a few sessions, the learning starts to show. The memory system keeps a compact profile of who you are and what your environment looks like — injected into every session automatically. Hermes already knows your project structure, your preferences, how you like things done.

The skills directory is where the procedures live. When Hermes solves something non-trivial — a workflow it had to figure out, a tool quirk it discovered — it saves that as a skill. Next session, it loads that skill and picks up where it left off.

You can list them:

```bash
hermes skills list
```

Browse the built-in hub:

```bash
hermes skills browse
```

And pin the ones you want to keep so the automated curator doesn't archive them:

```bash
hermes curator pin <skill-name>
```

The curator is a background process that maintains the skills library. It tracks which skills are actually being used, marks idle ones as stale, and archives them if they stop being relevant. It never deletes — but it keeps the active library clean.

After a few weeks of use, Hermes has a personalised library of procedures that's specific to your setup, your projects, your way of working. That's not something you get from a fresh agent session on a different machine every time.

---

## Wrapping Up

Here's the short version of why I run Hermes on an Arduino Uno Q:

- **The hardware is cheap** — €53 for the 4GB model, no recurring cost, physical asset at the end of its life
- **It handles the workload easily** — 2–5% CPU at idle; the heavy work is in the cloud
- **Tailscale gives you secure remote access** from anywhere, for free
- **Hermes compounds knowledge** — every session makes it a little better at your specific setup
- **No external skill registries** — the skills your agent builds come from your own sessions

Compare that to OpenClaw — higher token costs, skills pulled from an external registry, no persistent learning across sessions — and for me the choice is clear. This is the setup I'm running for the foreseeable future.

In the next video I want to go deeper on the memory and skills system — how to tune it, how to write your own skills, and how to set up the gateway so Hermes can reach you on Telegram or Discord. Subscribe if that sounds interesting.

All the links — Hermes, Tailscale, the Uno Q — are in the description. And if you're already running something similar, drop it in the comments.

Ahoy! 🤖
