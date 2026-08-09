---
layout: lesson
title: Remote Administration
author: Kevin McAleer
type: page
cover: /learn/meshcore/assets/cover.png
date: 2026-08-05
previous: 12_paths_and_routing.html
next: 14_troubleshooting.html
description: Log into your repeater over the air from your phone, change settings
  and read its stats
percent: 84
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


Your repeater is in the loft, on a pole, or in a weatherproof box on a roof. Fetching a ladder and a laptop every time you want to change a setting would get old fast. It is a good job you do not have to.

---

## Three ways in

| Method | Where you need to be | Good for |
|---|---|---|
| **USB serial console** | Physically at the node | First setup, recovery, forgotten passwords |
| **[config.meshcore.io](https://config.meshcore.io)** | Plugged in via USB, in Chrome or Edge | A friendlier form-based UI over the same commands |
| **Over the mesh, from the app** | Anywhere in radio range | Day to day admin, checking stats |
{:class="table table-single"}

The third one is the interesting one, and it is what this lesson is about.

---

## Logging in over the air

1. In the MeshCore app, find your repeater in the contacts list
2. Choose the admin, manage or login option
3. Enter the admin password you set in lesson 7

You are now talking to the repeater's command line **over LoRa**, encrypted, from wherever you happen to be standing.

> ## About the wait timer
>
> The MeshCore mobile app imposes a short wait before remote server management becomes available. An optional in-app purchase removes it. The functionality is all there in the free version - you just wait a moment.

---

## What you can do remotely

Everything the serial console offers. In practice, the ones you will actually reach for:

**Health checks**

```text
stats-core
stats-radio
stats-packets
neighbors
```

**Tuning**

```text
set tx 20
set advert.interval 60
set flood.advert.interval 12
set flood.max 3
```

**Housekeeping**

```text
set name Kev-Loft-01
set owner.info Kev - kevsrobots.com
advert
reboot
```

**Why `reboot` matters:** being able to reboot a wedged node without a ladder is worth the whole exercise on its own.

---

## Security

Your admin password is the only thing standing between your repeater and anyone within radio range with a MeshCore client.

**Do:**

- Change it from the default `password` the moment you flash the node - which you did in lesson 7
- Use something you will actually remember, or write it down somewhere safe
- Set `owner.info` so people can contact you rather than reconfiguring you

**Consider:**

```text
set allow.read.only true
```

This lets people query your repeater's stats without being able to change anything - friendly and useful on a community node.

**For tighter control:**

```text
get acl
set acl <...>
setperm <public-key> <permission>
```

The access control list lets you grant specific node keys specific permissions, which is worth exploring if your repeater is shared infrastructure.

---

## Reading the numbers

`stats-core` gives you uptime, battery voltage and queue depth.

**Queue depth is the interesting one.** A consistently non-zero queue means the repeater is receiving faster than it can retransmit - the mesh around it is congested. That is the signal to reduce flood limits or advert intervals, or to talk to the local community about settings.

`stats-radio` gives you RSSI, SNR and the **noise floor**. Track the noise floor over time. A rising noise floor means new interference has appeared near your repeater, and it will quietly eat your range.

---

## Recovering a locked out node

If you forget the password there is always a way back, it just needs physical access:

1. Plug in over USB
2. `password <new-password>`

The serial console is always trusted - the password only ever gates access over the air. If that is not enough, `erase` wipes the filesystem and returns the node to defaults.

---

## Try it Yourself

1. Log into your repeater from the app and run `stats-core`. What is its uptime?
2. Change the repeater's name remotely, then send an advert and watch the new name appear in the app.
3. Challenge: set `allow.read.only true`, log out, and confirm you can still read stats but not change settings.

---

## Common Issues

**Problem**: Remote login times out.

**Solution**: Check you can message the repeater normally first, and that you are in range.

**Why**: Remote admin runs over the mesh. If messaging does not work, admin cannot either.

**Problem**: The app says the password is wrong but I am sure it is right.

**Solution**: Try the default `password`, in case a reflash reset it.

**Why**: Flashing new firmware can reset stored settings depending on the build and whether the filesystem was erased.

**Problem**: I changed a setting remotely and now the repeater is unreachable.

**Solution**: Fetch the ladder. Connect over USB and put it back.

**Why**: Changing frequency or radio parameters remotely instantly moves the node off the channel you were talking to it on. **Never change radio settings on a remote node unless you can physically reach it.**

---
