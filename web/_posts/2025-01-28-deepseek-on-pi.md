---
title: "Installing and Using DeepSeek-R1:1.5 on a Raspberry Pi with Docker"
description: "Learn how to install and use DeepSeek-R1:1.5 on your Raspberry Pi using Docker. "
excerpt: >-
    We explore its features, pros, cons, and why it's a disruptive innovation challenging ChatGPT and Gemini.
layout: blog
date: 2025-01-26
date_updated: 2025-01-26
author: Kevin McAleer
difficulty: beginner
cover: /assets/img/blog/deepseek/cover.png
# hero:  /assets/img/blog/deepseek/hero.png
# mode: light
tags:
 - Raspberry Pi
 - ai
 - MicroPython
groups:
 - robotics
 - ai
 - raspberrypi
---

## Introduction

DeepSeek-R1:1.5 is an advanced AI model designed to deliver cutting-edge natural language processing (NLP) capabilities on lightweight devices. With its compact design and efficient architecture, DeepSeek-R1:1.5 is a powerful alternative to mainstream models like ChatGPT and Gemini, proving that high-performance AI can thrive outside the realm of massive cloud infrastructure. This guide will walk you through installing and running DeepSeek-R1:1.5 on a Raspberry Pi using Docker.

---

## Why DeepSeek-R1:1.5?

DeepSeek-R1:1.5 is a groundbreaking AI model for the following reasons:

1. **Compact Efficiency**: Optimized for resource-constrained devices like Raspberry Pi, it delivers impressive NLP performance without heavy GPU or cloud reliance.
2. **Privacy-Centric Design**: By running locally, it ensures that sensitive data remains on the device, offering unparalleled user privacy.
3. **Customization**: Developers can fine-tune it to fit specific use cases, making it highly adaptable for personal and professional projects.
4. **Cost-Effective**: Unlike subscription-based models like ChatGPT or Gemini, DeepSeek is open-source and free to run locally.

While it may not yet surpass the conversational depth of larger models, its innovative approach pushes the boundaries of on-device AI.

---

## Installation Requirements

To run DeepSeek-R1:1.5 on your Raspberry Pi, you'll need:

1. A Raspberry Pi 5 or higher (8GB RAM recommended for optimal performance, 16Gb works nicely too!).
2. Docker and Docker Compose installed.
3. Internet connection for initial image download.
4. MicroSD card with at least 32GB of storage.

### Step 1: Install Docker and Docker Compose

Start by installing Docker and Docker Compose on your Raspberry Pi:

Follow the detailed instructions here: [Install Docker](/learn/docker/02_installing-docker.html)

Verify the installation:

```bash
docker --version
docker-compose --version
```

---

### Step 2: Download my ClusteredPi docker respository

Pull my ClusteredPi repository from GitHub (which contains the docker-compose file for Ollama & Open-WebUI):

```bash
git clone https://www.github.com/kevinmcaleer/ClusteredPi.git
```

---

### Step 3: Run the Docker Compose File

Navigate to the  `ClusteredPi/stacks/ollama` folder and bring up the docker containers:

```bash
cd ClusteredPi/stacks/ollama
docker-compose up -d
```

This will pull the Ollama and Open-WebUI images from Docker Hub and start the containers in the background.

---

### Step 4: Log into the Open-WebUI

Open a browser to `http://<your-pi-ip>:3000` and create an account.

Once you have created an account, you can log in and start using the Open-WebUI.

---

### Step 5: Grab the deepseek-r1:1.5b model

Once you are logged into Open-WebUI:

- Click on the user icon
- Click on Admin panel
- Click on Setting
- Click on Models
- Click into the `Pull a model from Ollama.com` textbox
- Type `deepseek-r1:1.5b` - for the smallest model
- Once downloaded click on `New Chat` and try it out for yourself!

---

## Pros and Cons of DeepSeek-R1:1.5

### Pros

- **Lightweight**: Optimized for edge devices, making it accessible to hobbyists and developers.
- **Privacy**: Data stays on the device, addressing privacy concerns.
- **Customizable**: Open-source and adaptable to various use cases.
- **Affordable**: No ongoing costs compared to cloud-based AI services.

---

### Cons

- **Performance Limitations**: Lacks the computational power of cloud-based models like ChatGPT and Gemini.
- **Initial Setup Complexity**: Requires technical knowledge to install and configure.
- **Limited Ecosystem**: Fewer integrations and community support compared to mainstream models.

---

## How DeepSeek-R1:1.5 Challenges ChatGPT and Gemini

DeepSeek-R1:1.5 is a disruptive innovation in the AI landscape for several reasons:

1. **On-Device AI**: By focusing on local deployment, it eliminates the dependency on powerful servers, making AI more accessible and environmentally friendly.
2. **Privacy by Design**: Its local-first approach caters to users wary of cloud data breaches or surveillance.
3. **Democratization of AI**: Open-source availability allows developers to harness its power without corporate barriers.
4. **Edge Computing Revolution**: Pioneers AI use cases in edge computing, opening doors for innovative IoT applications.

While ChatGPT and Gemini excel in sheer scale and capabilities, DeepSeek-R1:1.5 proves that smaller, specialized models can carve their niche in the rapidly evolving AI world.

---

## Final thoughts

DeepSeek-R1:1.5 is a remarkable step forward in making advanced AI accessible on low-power devices like the Raspberry Pi. Its privacy-first design, affordability, and flexibility make it a compelling alternative to big players like ChatGPT and Gemini. By following this guide, you can unleash the potential of DeepSeek-R1:1.5 and explore new possibilities in local AI deployment.

---
