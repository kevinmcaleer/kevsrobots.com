---
layout: lesson
title: Hardware Requirements
author: Kevin McAleer
type: page
cover: /learn/openclaw_raspberry_pi/assets/openclaw.jpg
date: 2026-01-31
previous: 01_what-is-openclaw.html
next: 03_why-docker.html
description: Choose the right Raspberry Pi for running OpenClaw.
percent: 15
duration: 5
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


## Hardware Requirements

Running OpenClaw on a Raspberry Pi requires careful consideration of your hardware. Let's look at what you need.

### Minimum Requirements

To run OpenClaw, you'll need:

- **Raspberry Pi 4** (2GB RAM minimum) or **Raspberry Pi 5**
- **Power Supply** suitable for your model
- **Networking** (Ethernet or Wi-Fi)
- **Storage** (microSD card)
- **Access** (SSH or direct connection)

### Recommended Specifications

For comfortable operation, we recommend:

| Component | Minimum | Recommended | Ideal |
|-----------|---------|-------------|-------|
| **Pi Model** | Pi 4 | Pi 4 8GB | Pi 5 8GB |
| **RAM** | 2GB | 4GB | 8GB |
| **CPU** | ARM Cortex-A72 | ARM Cortex-A72 | ARM Cortex-A76 |
| **Storage** | 16GB microSD | 32GB microSD | 64GB microSD |
| **Power** | 3A | 5A | 5A |

### Why More RAM Matters

OpenClaw runs several processes:

- **Node.js runtime** - ~100-150MB
- **Gateway service** - ~100-200MB
- **Agent session** - ~150-300MB
- **Buffer for system** - ~500MB

With 2GB, you're at capacity. With 4GB or 8GB, you have breathing room for:
- Multiple agents
- Local model serving
- System processes
- Spikes in usage

### Storage Considerations

Your microSD card needs space for:

- **Raspberry Pi OS** - ~3-4GB
- **Docker installation** - ~1-2GB
- **OpenClaw container** - ~500MB-2GB
- **Your workspace data** - 100MB-1GB+
- **System updates** - ~500MB
- **Free space buffer** - ~5GB minimum

**Total recommended: 32GB card minimum**

If you plan to:
- Run multiple models locally
- Store lots of data in your workspace
- Keep extensive backup copies

Consider a 64GB or 128GB card.

### Power Supply

This is critical and often overlooked.

| Model | Power Requirement | Recommended PSU |
|-------|------------------|-----------------|
| **Pi 3 Model B+** | 2.5A @ 5V | 2.5A minimum |
| **Pi 4 Model B** | 3A @ 5V | 3A+ recommended |
| **Pi 5** | 5A @ 5V | 5A required |

**Using an undersized power supply causes:**
- Mysterious crashes and reboots
- Corrupted filesystems
- Docker container crashes
- OpenClaw service failures

**Buy a quality power supply.** This is not the place to save money.

### Networking

**Ethernet recommended** for:
- Stability (no Wi-Fi interference)
- Latency (important for real-time operations)
- 24/7 uptime scenarios
- Higher bandwidth

**Wi-Fi acceptable if:**
- You're on 5GHz dual-band
- You're close to the router
- It's not mission-critical

### Optional Hardware

#### AI HAT+

The official Raspberry Pi AI HAT+ is useful if you want to:
- Run local language models faster
- Experiment with on-device inference
- Learn about hardware acceleration

Not required, but nice for projects.

#### Case and Cooling

For continuous operation, consider:
- A good case (better airflow)
- Passive or active cooling
- Heatsinks on the SoC

OpenClaw isn't computationally intensive, but the Pi might be doing other things. Cooling helps reliability.

#### UPS/Battery Backup

If you want 24/7 operation:
- A UPS (uninterruptible power supply) protects against power failures
- Prevents filesystem corruption
- Ensures graceful shutdowns

### Choosing Your Pi

#### Pi 4 2GB - Minimum (Not Recommended)

**Pros:**
- Cheapest option
- Works technically

**Cons:**
- Very tight on memory
- Swaps to disk frequently (slow)
- Limited headroom for experimentation

**Use case:** Learning only, expect slowdowns

#### Pi 4 4GB - Entry Level (Acceptable)

**Pros:**
- Good balance of cost and performance
- Comfortable for basic OpenClaw usage
- Room to experiment

**Cons:**
- Limited if running multiple agents
- No room for local models

**Use case:** Single agent, cloud models only

#### Pi 4 8GB - Recommended

**Pros:**
- Plenty of headroom
- Can run local models
- Multiple agents simultaneously
- Future-proof

**Cons:**
- Costs more than 4GB

**Use case:** Serious experimentation, production use

#### Pi 5 8GB - Best

**Pros:**
- Faster CPU (A76 cores)
- 25% faster performance
- Thermal design improved
- Future-proof

**Cons:**
- Most expensive
- Might be overkill for basic use

**Use case:** Heavy workloads, production, local model serving

### Sample Setups

#### Budget Setup ($50-70)
- Pi 4 4GB
- Basic power supply
- 32GB microSD
- Ethernet cable

✅ Good for learning
❌ Limited for production

#### Solid Setup ($100-130)
- Pi 4 8GB
- Quality 5A power supply
- 32GB microSD
- Case with cooling
- Ethernet

✅ Great for most use cases
✅ Room for growth

#### Premium Setup ($150-200)
- Pi 5 8GB
- Quality 5A power supply
- 64GB microSD
- Good case with active cooling
- Ethernet
- UPS battery backup

✅ Production-ready
✅ Runs anything OpenClaw can do

### Before You Buy

1. **Know what you're using it for** - Learning vs production changes your needs
2. **Don't cheap out on power supply** - This causes more problems than undersized RAM
3. **Get at least 4GB RAM** - 2GB is too tight
4. **Go 32GB+ storage** - Cheaper than debugging storage issues
5. **Consider cooling** - Continuous operation benefits from it

### Next Steps

Now that you know what hardware you need, let's understand why Docker is the best way to run OpenClaw on it.

---
