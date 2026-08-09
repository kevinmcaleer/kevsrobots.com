---
layout: lesson
title: Setting Up the Repeater
author: Kevin McAleer
type: page
cover: /learn/meshcore/assets/cover.png
date: 2026-08-05
previous: 06_radio_settings.html
next: 08_repeater_placement.html
description: Configure node 1 as a repeater over the serial console - name, location,
  password and adverts
percent: 48
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


Node 1 has repeater firmware on it. Now we give it an identity, a location, a password, and a sensible advertising schedule. This is all done through the command line.

---

## Getting a console

Plug node 1 in (antenna fitted!) and connect to it using any of these:

- The **console** on [flasher.meshcore.io](https://flasher.meshcore.io) - easiest, no install
- [config.meshcore.io](https://config.meshcore.io) - a form-based UI over the same commands
- A serial terminal at **115200 baud** - `screen /dev/cu.usbserial-0001 115200` on macOS, PuTTY on Windows

Test it:

```text
ver
```

You should get a firmware version back. If you get nothing, check the Common Issues at the bottom.

---

## Step 1 - give it a name

```text
set name Kev-Repeater-01
```

**Why this matters:** Your repeater's name is what everyone in radio range sees when it adverts. Make it useful. A good convention is `<callsign-or-nickname>-<location>-<number>`, for example `Kev-Liverpool-01`. Avoid spaces and keep it short - it goes out over the air on every advert.

---

## Step 2 - set the clock

```text
time <unix-epoch-seconds>
```

If you are using the flasher console or the config web UI, there is usually a button that syncs the clock from your computer for you. From a plain serial terminal you can also use:

```text
clock sync
```

**Why this matters:** MeshCore uses timestamps to detect and discard duplicate packets. A repeater with a wildly wrong clock will behave strangely in ways that are very hard to debug.

---

## Step 3 - set the location

```text
set lat 53.4084
set lon -2.9916
```

Use your own coordinates - right click on your location in any online map to copy them.

**Why this matters:** Coordinates let your repeater appear on [map.meshcore.io](https://map.meshcore.io) and let other users work out whether it is worth trying to route through you. If you do not want your exact address public, round to three decimal places (roughly 100 metres) or use a nearby landmark.

---

## Step 4 - change the admin password

**Do this. Do not skip it.**

```text
password <your-new-password>
```

**Why this matters:** Every MeshCore repeater ships with the admin password set to `password`. That is fine for the five minutes before you change it, and a genuinely bad idea afterwards - anyone within radio range can otherwise log in over the air and reconfigure your node.

Write it down somewhere. Recovering a repeater with a forgotten password on a rooftop means fetching a ladder and a laptop.

---

## Step 5 - set the advert intervals

An **advert** is how a node announces "I exist, here is my name and public key". There are two kinds:

- A **zero-hop advert** is heard only by nodes in direct radio range
- A **flood advert** is repeated across the whole mesh

```text
set advert.interval 60
set flood.advert.interval 12
```

`advert.interval` is in **minutes** and accepts 60 to 240. `flood.advert.interval` is in **hours** and accepts 3 to 168. The values above - a local advert hourly and a flood advert twice a day - are sensible defaults for a fixed repeater.

The units catch people out constantly. `set flood.advert.interval 12` means *twelve hours*, not twelve minutes.

**Why this matters:** Flood adverts are expensive. Every repeater in the region rebroadcasts them. Setting `flood.advert.interval` to something aggressive like 1 hour is one of the classic ways to annoy your local mesh community.

You can also cap how far floods travel:

```text
set flood.max 3
```

---

## Step 6 - confirm and advertise

Check your work:

```text
get name
get radio
get freq
get tx
get lat
get lon
stats-core
```

Then announce yourself:

```text
advert
```

Note that plain `advert` sends a **flood** advert, which travels across the whole mesh. To announce only to nodes in direct radio range, use:

```text
advert.zerohop
```

---

## The commands worth remembering

| Command | What it does |
|---|---|
| `ver` / `board` | Firmware version and board type |
| `stats-core` | Uptime, battery, queue depth |
| `stats-radio` | RSSI, SNR and noise floor |
| `stats-packets` | Sent and received packet counts |
| `neighbors` | Repeaters you can hear directly |
| `discover.neighbors` | Actively go looking for neighbours |
| `advert` | Send a flood advert right now |
| `advert.zerohop` | Send a zero-hop advert - direct range only |
| `log start` / `log stop` | Start and stop packet logging |
| `reboot` | Restart the node |
| `erase` | Wipe the filesystem back to defaults |
{:class="table table-single"}

---

## Try it Yourself

1. Run `stats-radio` and note the noise floor. Now switch on a microwave or a smart meter nearby and run it again.
2. Set a deliberately silly name, run `advert`, then set it back. Watch the client pick up the change in lesson 10.
3. Challenge: run `log start`, leave the repeater for an hour, then `log` to dump what it heard. Anything out there?

---

## Common Issues

**Problem**: The console shows garbage characters.

**Solution**: Set the baud rate to **115200**.

**Why**: Any other rate will decode the serial stream as noise.

**Problem**: `set lat` accepts my value but the node does not appear on the map.

**Solution**: Check `get lat` and `get lon` came back correctly, run `advert`, and be patient.

**Why**: The map is populated by adverts heard by internet-connected nodes. If nobody near you is bridging to the internet, you will not appear - which is not a fault.

**Problem**: I forgot the admin password.

**Solution**: Connect over USB and run `password <new-password>`, or `erase` and start again.

**Why**: The serial console is always trusted - the password only gates access over the air.

---
