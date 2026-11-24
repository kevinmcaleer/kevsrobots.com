---
layout: course_overview
title: Off-Grid Messaging with MeshCore
author: Kevin McAleer
cover: /learn/meshcore/assets/cover.png
date: 2026-08-05
description: Build your own off-grid LoRa mesh network with MeshCore and two Heltec
  V3 boards, one as a repeater and one as a client. No internet, no phone signal,
  no subscription.
level: beginner
course_duration: 1h 29m
lesson_count: 16
first_lesson: 00_intro.html
quiz_total: 45
outcomes:
- Explain what MeshCore is and what the repeater and companion roles do
- Describe how spreading factor, bandwidth and coding rate affect range and airtime
- Flash and configure a MeshCore repeater and a Bluetooth companion with matching
  radio settings
- Place and power a repeater sensibly, handling antennas and batteries safely
- Send messages, use channels, and read RSSI, SNR and paths to diagnose a weak link
- Administer a repeater over the air and troubleshoot a mesh that will not talk
prerequisites:
- No programming or soldering needed, and no earlier course required
- Helpful but not required, typing commands as in Introduction to Linux
you_need:
- Two Heltec WiFi LoRa 32 V3 boards in the frequency variant for your region
- Two antennas for that band, and a U.FL to SMA pigtail if needed
- A USB-C data cable and a computer with Chrome or Edge
- A phone with the MeshCore app, or a laptop to use the web client
- Optional, a LiPo battery and a weatherproof box for an outdoor repeater
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
    - name: Radio Settings for Your Region
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
