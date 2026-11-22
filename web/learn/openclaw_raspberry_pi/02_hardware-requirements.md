---
layout: lesson
title: Hardware Requirements
author: Kevin McAleer
type: page
cover: /learn/openclaw_raspberry_pi/assets/banners/02_hardware-requirements.jpg
date: 2026-01-31
previous: 01_what-is-openclaw.html
next: 03_why-docker.html
description: Choose the right Raspberry Pi, power supply, storage, network and cooling
  for an always-on OpenClaw Gateway, and understand why the Pi does not run the AI
  model
percent: 15
duration: 7
quiz_total: 54
date_updated: 2026-11-22
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
    - name: First Access to the Control UI
      link: 07_first-access.html
  - section: Configuration & Usage
    content:
    - name: Configuring OpenClaw
      link: 08_configuring-openclaw.html
    - name: Pi Butler in Your Pocket
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
    - name: Is OpenClaw Safe on a Raspberry Pi?
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


![Course Cover Image]({{page.cover}}){:class="cover"}

---

In this lesson we'll pick the hardware for Pi Butler. By the end you'll know which Raspberry Pi models work, which power supply and storage to use, and why a modest Pi is plenty for the job.

---

## Why the Pi doesn't need to be a monster

The Pi's job is to run the OpenClaw Gateway. The Gateway is a Node.js service that waits for messages, keeps track of sessions, runs the odd tool, and passes the hard thinking to a cloud AI model over the internet.

```mermaid
flowchart LR
    PI["Raspberry Pi: Gateway, config, history"] -->|"API call"| CLOUD["Cloud model: the heavy lifting"]
    CLOUD -->|"reply"| PI
```

Running a language model is a very different job. It needs lots of fast memory and, ideally, a GPU. The official OpenClaw Raspberry Pi guide puts it plainly: use cloud API models, and do not run local language models on a Pi, because even small models are too slow to be useful. So you don't need an AI accelerator board for this course.

So the Pi needs to be **reliable** more than it needs to be fast.

---

## The official compatibility table

This comes straight from the OpenClaw Raspberry Pi guide.

| Pi model | RAM | Works? | Notes |
| --- | --- | --- | --- |
| Pi 5 | 4 or 8 GB | Best | Fastest, recommended |
| Pi 4 | 4 GB | Good | Sweet spot for most users |
| Pi 4 | 2 GB | OK | Add swap |
| Pi 4 | 1 GB | Tight | Possible with swap, minimal config |
| Pi 3B+ | 1 GB | Slow | Works but sluggish |
| Pi Zero 2 W | 512 MB | No | Not recommended |
{:class="table table-single"}

The guide recommends **2 GB of RAM or more**, a **16 GB or larger** SD card or a USB SSD, and **Ethernet**. Whatever the model, you need the **64-bit** version of Raspberry Pi OS. The 32-bit version won't do.

We'll run OpenClaw in Docker, using the ready-made image that the OpenClaw project publishes. Building that image yourself needs 6 GB of RAM or more, which is why we never build it on the Pi. More on that in [lesson 3](/learn/openclaw_raspberry_pi/03_why-docker.html).

---

## Power supply

An always-on computer needs steady power. A weak or cheap supply causes random reboots and corrupted files, and those are miserable to track down.

| Pi model | Official power supply |
| --- | --- |
| Pi 4 | 5.1 V, 3 A, USB-C |
| Pi 5 | 27 W (5.1 V, 5 A), USB-C |
{:class="table table-single"}

Use the official supply for your model. On a Pi 5 this matters even more if you plan to boot from a USB SSD, because a lower-rated supply limits the power available to USB devices.

---

## Storage: SD card or USB SSD

Both work. The OpenClaw guide recommends an SSD, because SD cards are slow and wear out, while a USB SSD is much faster and survives far more writes. An agent that runs all day writes a steady trickle of logs, chat history and database updates, which is exactly what wears a card down.

| | microSD card | USB SSD |
| --- | --- | --- |
| Cost | Cheapest | A bit more |
| Speed | Slow | Much faster |
| Lifespan with constant writes | Wears out | Lasts much longer |
| Setup | Nothing extra | Boot the Pi from USB |
{:class="table table-single"}

If you use a card, pick a good-quality one of 16 GB or more, and keep backups (we cover those in [lesson 15](/learn/openclaw_raspberry_pi/15_backup-and-recovery.html)). Raspberry Pi has a guide to [booting from USB](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#usb-mass-storage-boot).

---

## Network: Ethernet if you can

Your agent is only useful if it's online. Ethernet is the recommended choice because it is steady and doesn't drop out. Wi-Fi works too, and we'll show you how to stop the Pi's Wi-Fi from dozing off in [lesson 4](/learn/openclaw_raspberry_pi/04_preparing-your-pi.html).

> **Teachers**: school and guest Wi-Fi often blocks devices from seeing each other, so `openclaw-pi.local` may not resolve and SSH may fail. A cheap switch and Ethernet cables, or a separate classroom router, saves a lot of time.

---

## Cooling

OpenClaw isn't a heavy load most of the time, but the Pi will be on all day, every day. Keep it cool and it stays fast and reliable.

- **Pi 5**: fit the official Active Cooler or a case with a fan.
- **Pi 4**: a case with a heatsink, or a small fan, is enough.

Later in the course we'll check whether the Pi has been throttling because of heat or power.

---

## Pick your setup

| Setup | Hardware | Good for |
| --- | --- | --- |
| **Starter** | Pi 4 2 GB, official PSU, 32 GB microSD, swap turned on | Learning the ropes. Expect it to feel a little slow |
| **Recommended** | Pi 4 4 GB or Pi 5 4 GB, official PSU, USB SSD or good microSD, Ethernet, heatsink or fan | Pi Butler running all day, every day |
| **Best** | Pi 5 8 GB, 27 W PSU, USB SSD, Ethernet, Active Cooler | Plenty of headroom for skills and browser tools |
{:class="table table-single"}

You'll also need a computer to SSH from, a phone with Telegram, and an API key from a model provider.

## Pi Butler progress

You've chosen Pi Butler's home. Next we'll decide how to run it: straight on the Pi, or in a container.

---

## Try it Yourself

1. **Check your Pi.** Find the model and RAM size of the Pi you plan to use, then find its row in the compatibility table.
2. **Check your power supply.** Read the label on your supply. Does it match the official rating for your model?
3. **Challenge:** Price up the "Recommended" setup with parts from your usual supplier, then add a rough monthly API budget. Which costs more over a year?

---

## Common Issues

**Problem**: The Pi reboots at random, or shows a lightning bolt icon on a screen.
**Solution**: Swap to the official power supply for your model.
**Why**: Under-voltage makes the Pi throttle or reset, and it can corrupt the SD card.

**Problem**: Everything feels sluggish on a Pi 4 with 2 GB.
**Solution**: Add swap (lesson 4 shows how) and stick to cloud API models.
**Why**: The official table rates 2 GB as OK only with swap.

**Problem**: You planned to run a local model on the Pi to avoid API costs.
**Solution**: Use a cloud API model with a spending limit instead.
**Why**: The official guide says even small local models are too slow on a Pi to be useful.

---

## Check your understanding

{% quiz %}
? Which Pi does the official OpenClaw guide call the sweet spot for most users?
- [ ] Pi Zero 2 W
- [x] Pi 4 with 4 GB
- [ ] Pi 3B+ with 1 GB
> The Pi 5 is best, but the Pi 4 with 4 GB is the sweet spot. The Pi Zero 2 W is not recommended.

? Why is a USB SSD better than an SD card for an always-on agent?
- [ ] It is the only storage that supports Docker
- [x] It is faster and survives many more writes
- [ ] OpenClaw refuses to run from an SD card
> SD cards work, but they are slow and wear out under the steady writes of an always-on service.

? Why don't we build the OpenClaw Docker image on the Pi?
- [x] Building it needs 6 GB of RAM or more, so we use the pre-built image
- [ ] There is no arm64 version of OpenClaw
- [ ] Docker can't build images on a Raspberry Pi
> A local source build needs at least 6 GB of RAM. The project publishes ready-made images, including arm64, so the Pi just downloads one.
{% endquiz %}
