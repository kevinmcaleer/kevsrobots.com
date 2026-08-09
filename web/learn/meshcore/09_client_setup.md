---
layout: lesson
title: Setting Up the Client
author: Kevin McAleer
type: page
cover: /learn/meshcore/assets/cover.png
date: 2026-08-05
previous: 08_repeater_placement.html
next: 10_first_contact.html
description: Pair node 2 with the MeshCore phone app over Bluetooth and match its
  radio settings
percent: 60
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


Node 2 has companion firmware on it. It has no keyboard and no way to type a message - that is your phone's job. Let's pair them up.

---

## Getting the app

The MeshCore companion apps are built by Liam Cottle and are available for **iOS, Android, Windows and macOS**. Search your app store for **MeshCore**, or grab them from [files.liamcottle.net/MeshCore](https://files.liamcottle.net/MeshCore).

The app is free. There is an optional in-app purchase that removes a wait timer on remote server management, which becomes relevant in lesson 13 - you do not need it yet.

**No phone?** If you flashed **Companion (USB Serial)** instead of BLE, plug the board into a computer and open [app.meshcore.nz](https://app.meshcore.nz) in Chrome or Edge. It is free, open source, and does the same job over USB.

---

## Pairing over Bluetooth

1. Power up node 2 with its antenna fitted. The OLED should show a pairing screen or the node's name
2. Open the MeshCore app and choose to add or connect a device
3. Pick your node from the Bluetooth list - it will have a generic name like `MeshCore-a1b2` to begin with
4. If a pairing code is requested, the default is usually `123456`, and the OLED will show a code if one is generated
5. The app connects and shows you the node's status

**A note on Bluetooth:** BLE range is a few metres. The board needs to be near your phone - it is a radio *modem* for your phone, not something you leave in the loft. The long range part happens on the LoRa side.

---

## Naming your node

In the app's settings, set your node's name. This is what other people in the mesh will see.

Pick something you are happy being public. `Kev-Handheld` is good. Your full name and postcode is not.

---

## Matching the radio settings

This is the step that decides whether the whole project works.

1. Go into the app's **settings** for the connected node
2. Find the **frequency** or **radio** section
3. Look for a **preset** menu - usually a three-dot icon next to the frequency field
4. Choose **EU/UK (Narrow)**
5. Apply, and confirm

Verify the four numbers against what your repeater reported when you ran `get radio` in lesson 7:

| Setting | Expected |
|---|---|
| Frequency | 869.618 MHz |
| Bandwidth | 62.5 kHz |
| Spreading Factor | 8 |
| Coding Rate | 8 |
{:class="table table-single"}

**If these do not match your repeater exactly, nothing that follows will work.** It is worth double checking now rather than debugging later.

---

## Send an advert

With the settings applied, tell the mesh you exist. In the app there will be an option to **advertise** or **send advert** - often on the node's status screen or in a menu.

You have two choices:

- **Zero-hop advert** - heard by anything in direct radio range, including your repeater
- **Flood advert** - passed on across the whole mesh

Start with a zero-hop advert. If your repeater is on the desk next to you, that is all you need.

**Why this matters:** Unlike repeaters, client nodes do not advertise automatically on a timer. Nobody knows you exist until you tell them. If you have set everything up correctly and your contact list is empty, sending an advert is usually the missing step.

---

## Battery and power

The companion node is the one you carry, so power matters:

- USB-C from a power bank is the simplest option
- A LiPo on the SH1.25 connector makes it properly portable
- The OLED is a meaningful power draw - some builds let you dim or blank it

---

## Try it Yourself

1. Pair the app, rename the node, and confirm the new name appears on the OLED.
2. Send a zero-hop advert, then walk 20 metres away and send another. Does the repeater still hear you?
3. Challenge: unpair and re-pair the node. Getting comfortable with this saves panic later when the app forgets a device.

---

## Common Issues

**Problem**: The node does not appear in the Bluetooth list.

**Solution**: Confirm you flashed **Companion (BLE)** and not **Companion (USB Serial)** or **Repeater**.

**Why**: Only the BLE build advertises over Bluetooth. Repeater firmware has no BLE interface at all.

**Problem**: The app connects, then immediately disconnects.

**Solution**: Forget the device in your phone's Bluetooth settings, then pair again from inside the MeshCore app.

**Why**: Pairing at the OS level rather than in the app confuses the connection state on some phones.

**Problem**: Everything looks fine but my contact list is empty.

**Solution**: Send an advert, and get someone else to send one too.

**Why**: Contacts are learned from adverts. A silent mesh looks identical to a broken one.

---
