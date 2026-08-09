---
layout: lesson
title: Channels and Group Chat
author: Kevin McAleer
type: page
cover: /learn/meshcore/assets/cover.png
date: 2026-08-05
previous: 10_first_contact.html
next: 12_paths_and_routing.html
description: How MeshCore channels work, the public channel key, and creating your
  own private group
percent: 72
duration: 4
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


Direct messages are one to one. Channels are how a group talks - a family, a hiking club, a village, or the whole regional network.

---

## What a channel actually is

A channel in MeshCore is nothing more than **a name and a shared secret key**. Anyone holding the same key can read and post to the channel. There is no server, no membership list, no admin.

**Why this matters:** Adding someone to a channel means giving them the key. Removing someone means creating a new channel with a new key and giving that to everyone else. It is simple, robust, and entirely in your hands.

---

## The public channel

MeshCore ships with a well-known public channel that every node can use out of the box. Its key is not a secret - it is published so that strangers can find each other:

```text
Hex:    8b3387e9c5cdea6ac9e5edbaa115cd72
Base64: izOH6cXN6mrJ5e26oRXNcg==
```

Note the third character of the Base64 string is a capital letter O, not a zero. That catches people out.

**Use it for:** saying hello to the local mesh, testing, emergencies, coordinating with strangers.

**Do not use it for:** anything private. Assume everyone within radio range can read it, because they can.

---

## Creating your own channel

In the MeshCore app, add a channel and either generate a new random key or type in a shared one.

To share it with your group, the easiest route is a **QR code** - the app will generate one for you, and the other person scans it. No typing, no transcription errors.

**A worked example - a family channel:**

1. On your phone, create a channel called `Family`
2. Let the app generate a random key
3. Show the QR code to each family member and have them scan it
4. Everyone with the key now sees every message posted to that channel

---

## Channels always flood

This is the important technical detail.

Direct messages learn a **path** and then follow it, which is efficient. Channel messages cannot do that - a channel has no single destination, so **every channel message floods across the mesh**.

**Why this matters:**

- Channel traffic is expensive. Ten people chatting on a channel across a regional mesh is a lot of airtime
- Repeater operators can and do limit how far floods travel, with `set flood.max <hops>`
- On a busy network, prefer direct messages for one to one conversation and keep channels for things the group genuinely needs to see

---

## Sensible channel etiquette

- **Keep messages short.** Every character costs airtime for everyone
- **Do not use channels for testing.** Test with direct messages to your own second node
- **Respect the local public channel.** It is often used for coordination and emergencies
- **Do not bridge a channel to the internet** without telling the people on it

---

## Limiting flood range on your repeater

From the repeater console:

```text
set flood.max 3
```

This caps how many hops a flooded packet will travel through your repeater. On a small local mesh, 3 is generous. On a dense regional network, operators sometimes go lower.

---

## Try it Yourself

1. Add the public channel to your client and post a short hello. Does anyone answer?
2. Create a private channel, generate a key, and export the QR code. Save it somewhere safe.
3. Challenge: with `log start` running on your repeater, post a channel message and then read the log. Can you see the flood?

---

## Common Issues

**Problem**: I typed the public channel key in and it does not work.

**Solution**: Check the third character of the Base64 key - it is a capital O, not a zero. Or use the hex version instead.

**Why**: The two characters are nearly identical in most fonts and it is the single most common transcription error.

**Problem**: My friend cannot see my channel messages.

**Solution**: Confirm you both have the exact same key, and that your radio settings match.

**Why**: A channel with a different key is a different channel, even if the *name* is identical. The name is only a label.

**Problem**: My channel messages take ages to arrive.

**Solution**: Nothing is wrong. Consider using a direct message instead.

**Why**: Channel messages flood, which means every repeater in range queues and retransmits them. On a busy mesh that queue can be long.

---
