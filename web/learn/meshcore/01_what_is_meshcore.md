---
layout: lesson
title: What is MeshCore?
author: Kevin McAleer
type: page
cover: /learn/meshcore/assets/cover.png
date: 2026-08-05
previous: 00_intro.html
next: 02_how_lora_works.html
description: Understand what MeshCore is, the four device roles, and how it differs
  from Meshtastic
percent: 12
duration: 6
navigation:
- name: Off-Grid Messaging with MeshCore
- content:
  - section: Overview
    content:
    - name: Introduction
      link: 00_intro.html
    - name: What is MeshCore?
      link: 01_what_is_meshcore.html
    - name: How LoRa Works
      link: 02_how_lora_works.html
  - section: Hardware
    content:
    - name: The Hardware
      link: 03_the_hardware.html
    - name: Antennas and the Law
      link: 04_antennas_and_the_law.html
  - section: Flashing the Firmware
    content:
    - name: Flashing the Firmware
      link: 05_flashing.html
    - name: Radio Settings for the UK
      link: 06_radio_settings.html
  - section: Build the Repeater
    content:
    - name: Setting Up the Repeater
      link: 07_repeater_setup.html
    - name: Where to Put the Repeater
      link: 08_repeater_placement.html
  - section: Build the Client
    content:
    - name: Setting Up the Client
      link: 09_client_setup.html
    - name: First Contact
      link: 10_first_contact.html
  - section: Using Your Mesh
    content:
    - name: Channels and Group Chat
      link: 11_channels.html
    - name: Paths and Routing
      link: 12_paths_and_routing.html
    - name: Remote Administration
      link: 13_remote_admin.html
  - section: Going Further
    content:
    - name: Troubleshooting
      link: 14_troubleshooting.html
    - name: Where to Go Next
      link: 15_next_steps.html
---


MeshCore is a lightweight, hybrid-routing mesh protocol for packet radios. In plain English: it is firmware that turns a cheap LoRa board into a node on a self-organising, encrypted text messaging network.

It is free, open source under the MIT licence, and it runs on hardware you can buy for the price of a takeaway.

---

## What problem does it solve?

Mobile phones are wonderful right up until the moment there is no signal. Hills, valleys, festivals, remote workshops, hiking trips, power cuts, and boats all have the same problem - the infrastructure is somewhere else.

A LoRa mesh moves the infrastructure to you. Each node can talk directly to any other node in radio range, and repeaters extend that range by forwarding packets onward. There is no central server, no SIM card, and no subscription.

![MeshCore roles](/learn/meshcore/assets/roles.svg){:class="img-fluid w-100"}

---

## The four roles

When you flash MeshCore you choose a **role** for the board. This is the single most important decision in the whole course, so let's take them one at a time.

### Companion (also called a client)

**What it does:** Connects to your phone, tablet or computer over Bluetooth LE or USB serial. This is the node you actually talk through.

**Why this matters:** A companion node does **not** repeat other people's traffic. It sends your messages and receives messages addressed to you or your channels, and that is it. If you buy two boards and flash both as companions, you have a network with no range extension at all.

### Repeater

**What it does:** Forwards packets on behalf of other nodes to extend the range of the network. It has no phone attached and normally sits somewhere high with a power supply.

**Why this matters:** This is the piece that turns two radios into a *network*. Critically, a MeshCore repeater does **not** blindly rebroadcast everything it hears - it makes a decision about whether a packet needs forwarding. That is what keeps a MeshCore mesh quiet and usable as it grows.

### Room Server

**What it does:** A bulletin-board style message store. It holds posts and hands them to users when they come back into range, a bit like a tiny email server.

**Why this matters:** In a mesh, if you are out of range when a message is sent, you miss it. A room server keeps a rolling buffer of recent posts - 32 by default - and tracks where each user got to, so you can catch up later.

Full build in [part two of this series](/learn/meshcore_rooms_sensors/).

### Sensor

**What it does:** Reports telemetry - temperature, humidity, battery voltage - into the mesh rather than carrying conversations.

**Why this matters:** It turns your mesh into a long-range, no-subscription IoT network. A sensor on a beehive at the bottom of a field is a very satisfying project.

Full build in [part two of this series](/learn/meshcore_rooms_sensors/).

**In this course** we will build one **Repeater** and one **Companion**. Those two roles cover 90% of real world use.

---

## How is this different from Meshtastic?

If you have been around LoRa before, you will have heard of Meshtastic. Both are excellent projects and they run on much of the same hardware, but they solve the problem differently.

### Routing

**Meshtastic** uses managed flooding - a message is rebroadcast by every node that hears it, up to a hop limit. Simple, robust, and it works brilliantly with a handful of nodes.

**MeshCore** uses hybrid routing. The very first message to a new contact floods to find them, the destination replies with the route it took, and every message after that follows that specific path. Only the repeaters named in the path bother to forward it.

**Why this matters:** Airtime is the scarce resource in a LoRa network. Flooding uses airtime proportional to the number of nodes; path routing uses airtime proportional to the number of hops. As a network grows past a couple of dozen nodes, that difference is the difference between a network that works and one that chokes on its own chatter.

### Clients repeat by default

In Meshtastic, an ordinary client node will typically also rebroadcast traffic. In MeshCore, a companion node **never** repeats. Repeating is a deliberate, separate role.

**Why this matters:** It means you cannot accidentally build a mesh out of pocket devices. You have to plan your infrastructure - which is more work up front, and far better behaved once it is running.

### Remote administration

MeshCore repeaters have a full command line, reachable over USB, over the web config tool, or **remotely over the mesh itself** from your phone. You can change the name, transmit power or advert interval of a repeater on a hilltop without climbing the hill.

---

## What MeshCore is not

- It is **not** LoRaWAN. LoRaWAN needs gateways and a network server, and is designed for sensors reporting to the cloud. MeshCore is peer to peer.
- It is **not** fast. You are sending short text messages at a few hundred bits per second. There is no voice, no images, no web browsing.
- It is **not** anonymous. Messages are end-to-end encrypted, but adverts announce your node's name and public key to anyone listening.

---

## Try it Yourself

1. Open [map.meshcore.io](https://map.meshcore.io) and find your nearest MeshCore repeater. How far away is it?
2. Look up your local MeshCore community - in the UK, [localmesh.co.uk](https://localmesh.co.uk) coordinates settings so everyone's nodes can hear each other.
3. Challenge: sketch your own street or village and mark the highest point you could reach with a mains socket. That is where your repeater wants to live.

---

## Common Issues

**Problem**: "I bought two boards and flashed both as companions - why does nothing get further away?"

**Solution**: Flash one as a repeater.

**Why**: Companion nodes do not forward traffic for others. Without a repeater your range is simply whatever the two radios can manage line of sight.

**Problem**: "My friend runs Meshtastic - can we talk to each other?"

**Solution**: No. You will both need to run the same firmware.

**Why**: The protocols, packet formats and encryption are completely different, even on identical hardware.

---
