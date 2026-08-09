---
layout: lesson
title: Where to Go Next
author: Kevin McAleer
type: page
cover: /learn/meshcore/assets/cover.png
date: 2026-08-05
previous: 14_troubleshooting.html
description: Room servers, sensors, solar repeaters, T-Decks and joining the wider
  MeshCore community
percent: 100
duration: 5
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


You have a working mesh. A repeater extending your range, a client in your pocket, and a network that owes nothing to anybody's infrastructure. Here is where to take it.

---

## What you built

Look back at what is now running:

- A **repeater** with a name, a location, a password, and a sensible advertising schedule
- A **client** paired to your phone, sending encrypted messages
- Matched radio settings that let you talk to every other UK MeshCore node
- The ability to administer the repeater over the air from anywhere in range

For about £30 of hardware, that is a genuinely capable off-grid communications system.

---

## Add a third node

The single best upgrade is a **third board flashed as a companion**. Suddenly you can send real messages between two humans, test paths properly, and see hop counts that are not zero.

Give it to a family member, a neighbour, or keep it in the car. Two clients and a repeater is where a mesh starts feeling real.

---

## Build a room server

A **room server** is a bulletin board that stores messages for people who are out of range, keeping a rolling buffer of recent posts. It is one flash away - same hardware, different firmware variant.

Useful for a club, a village, or a family that is scattered across a valley. Note the guidance from the MeshCore community: run a room server on its **own board** rather than enabling `repeat` on it, because doing so costs you the admin features.

The default guest password is `hello` - change it.

---

## Add sensors

The **sensor** role turns a board into a long-range telemetry node. Temperature in the greenhouse, a water level in a tank, battery voltage on a remote installation, door contacts on an outbuilding - all reporting back over kilometres with no WiFi and no subscription.

The relevant commands on the console are `sensor list`, `sensor get` and `sensor set`.

---

## Go solar

A solar repeater is the natural end point for this project - a node on a hill that never needs visiting.

**The honest advice:** the ESP32-S3 in the Heltec V3 is thirsty. For a serious solar build, use a **Heltec T114** instead - it is nRF52 based and uses a small fraction of the power. Pair it with a 5-10W panel, a proper charge controller and an 18650 pack, and it will run through a British summer comfortably.

Also worth setting on any battery node:

```text
powersaving on
```

---

## Get a T-Deck

The **Lilygo T-Deck** is a complete handheld MeshCore device with a QWERTY keyboard and a colour screen. No phone required, no Bluetooth pairing, nothing to go flat in your pocket. It is the closest thing MeshCore has to a proper radio you can just pick up and use.

---

## Join the community

A mesh with one owner is a hobby. A mesh with fifty owners is infrastructure.

- **[map.meshcore.io](https://map.meshcore.io)** - see who is on air near you
- **[localmesh.co.uk](https://localmesh.co.uk)** - the UK community, settings and coordination
- **[meshcore.io](https://meshcore.io)** and **[docs.meshcore.io](https://docs.meshcore.io)** - official documentation
- **[github.com/meshcore-dev/MeshCore](https://github.com/meshcore-dev/MeshCore)** - the firmware itself

Register your repeater. Tell people where it is and who owns it. Someone, one day, will be very glad it was there.

---

## Ideas to build

**Beginner**

- A second client for a family member, with a shared private channel
- A battery-powered portable repeater for camping trips

**Intermediate**

- A weatherproofed loft repeater with an external antenna on the chimney
- A room server for your local maker group
- A temperature sensor node at the bottom of the garden

**Advanced**

- A solar hilltop repeater on a T114
- A mesh-connected sensor network across a smallholding
- Bridging two mesh segments with `bridge.enabled` over a serial link

---

## Your capstone challenge

Here is a proper test of everything in this course:

> **Get a message from one end of your town to the other, with at least one hop, and record the path and signal quality at each step.**

That means: a well-placed repeater, matching settings, a client that can trace, and enough understanding of RSSI and SNR to know whether the link is healthy or lucky. If you can do that, you can build a mesh anywhere.

---

## Related courses

- [MeshCore Room Servers and Sensors](/learn/meshcore_rooms_sensors/) - part two, building the other two node types in full
- [Raspberry Pi Pico with MicroPython - GPIO Mastery](/learn/micropython_gpio/) - for the sensor projects
- [Mini-Rack 3D Design Tutorial](/learn/mini_rack/) - if your repeater ends up part of a home lab
- [Introduction to Linux](/learn/linux_intro/) - useful if you go down the bridging and gateway route

---

## Thank you

Thanks for working through this one. If you build something with MeshCore, I would love to see it - and if you spot something in this course that is out of date, tell me. Radio conventions move, and this stuff changes faster than most.

Now go and put something on a roof.

---
