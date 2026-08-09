---
layout: lesson
title: Troubleshooting
author: Kevin McAleer
type: page
cover: /learn/meshcore/assets/cover.png
date: 2026-08-05
previous: 13_remote_admin.html
next: 15_next_steps.html
description: A systematic checklist for a mesh that will not talk, from dead boards
  to silent networks
percent: 90
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


Something is not working. Do not start randomly changing settings - work down this list in order. Nine times out of ten it is one of the first four items.

---

## The five minute checklist

Before anything else, confirm all of these:

1. **Antennas fitted to both boards?**
2. **Both boards on the same frequency, bandwidth and spreading factor?** Run `get radio` on the repeater and read the app's settings screen on the client. Compare digit by digit
3. **Have you sent an advert?** Clients do not advertise automatically
4. **Are the boards at least a few metres apart?** Touching on a desk overloads the receiver
5. **Did you flash the right variants?** One repeater, one companion

---

## The board is not detected by the computer

**Symptoms:** No serial port appears; the flasher shows an empty device list.

**Fixes, in order:**

1. **Try a different USB-C cable.** Charge-only cables are the number one cause
2. Try a different USB port, ideally directly on the machine rather than through a hub
3. Check for the CP2102 device - `ls /dev/cu.*` on macOS, Device Manager on Windows, `dmesg | tail` on Linux
4. Hold the **BOOT** / **PRG** button while plugging in, to force the bootloader
5. Use Chrome or Edge - WebUSB does not exist in Firefox or Safari

---

## The OLED is blank

1. Is the board getting power at all? Look for the power LED
2. Reflash the firmware - a partial flash leaves a board that boots but does nothing
3. Some builds blank the screen after a timeout. Press the user button

---

## The two nodes cannot hear each other

This is the big one, and it is almost always settings.

**Check, in order:**

1. **Frequency.** `get freq` on the repeater. Is it `869.618`? Not `869.525`, not `868.0`
2. **Bandwidth and spreading factor.** `get radio`. These must match *exactly* - coding rate can differ, the other three cannot
3. **Reboot both** after any settings change
4. **Distance.** Move them 5-10 metres apart and try again
5. **Advert.** Press `advert` on the repeater console while watching the client's contact list
6. **Frequency variant.** Is one board a 433MHz unit? Check the sticker

---

## Messages send but never arrive

1. **Trace the contact** and look at the path and the SNR at each hop
2. **Reset the path** for that contact and let it rediscover
3. **Check the repeater is actually running** - `stats-core` for uptime
4. **Check queue depth** in `stats-core`. A large queue means the mesh is congested

---

## Range is much worse than expected

| Cause | Fix |
|---|---|
| Antenna lying horizontal | Stand it vertically at both ends |
| Wrong band antenna | Confirm it is an 868MHz antenna |
| Long thin coax | Move the board to the antenna, run USB instead |
| Foil-backed insulation | Get the antenna outside the roof space |
| Node too low | Height beats everything - get it higher |
| Damaged PA | Did you ever power up without an antenna? Compare against a known good board |
| Local interference | Check the noise floor with `stats-radio` |
{:class="table table-single"}

---

## The repeater keeps dropping off

1. **Power bank auto-shutdown.** An idle repeater draws too little current for many banks to stay awake. Use a mains charger
2. **Brownouts.** A weak USB supply causes reboots on transmit peaks - check uptime in `stats-core`
3. **Overheating.** A black box in direct sun gets remarkably hot. Shade or vent it
4. **Water ingress.** Check for condensation inside the enclosure

---

## I can see my nodes but nobody else's

1. **Ask your local community which preset they use.** Some regions still run legacy settings
2. **Check the map** at [map.meshcore.io](https://map.meshcore.io) - there may genuinely be nobody in range
3. **Get higher.** Community repeaters are often several kilometres away

---

## Signal quality reference

| RSSI | Meaning |
|---|---|
| -30 to -60 dBm | Very strong - possibly too close |
| -60 to -90 dBm | Comfortable |
| -90 to -110 dBm | Working, but marginal |
| Below -115 dBm | At the edge, expect losses |
{:class="table table-single"}

| SNR | Meaning |
|---|---|
| Above 0 dB | Signal is stronger than the noise - excellent |
| 0 to -7 dB | Normal for LoRa at range |
| -7 to -10 dB | Marginal at SF8 - the UK preset |
| Below -10 dB | Below the decoding threshold at SF8 (SF12 reaches about -20 dB) |
{:class="table table-single"}

---

## Starting over

Sometimes the fastest fix is a clean slate. On the repeater console:

```text
erase
reboot
```

Then reflash and reconfigure from lesson 5. It takes ten minutes and eliminates every accumulated mistake at once.

---

## Try it Yourself

1. Deliberately break your setup - change the client's spreading factor - then work down this checklist and find it.
2. Record your normal RSSI and SNR to the repeater somewhere. Having a baseline makes future problems obvious.
3. Challenge: write your own one-page troubleshooting card for your specific setup and tape it inside the repeater's enclosure lid.

---

## Common Issues

**Problem**: I have tried everything on this list and it still does not work.

**Solution**: Erase and reflash both boards, then configure them side by side on the same desk, 5 metres apart.

**Why**: Debugging two variables at once is much harder than starting from a known good state.

**Problem**: It works on the desk but not once the repeater is in place.

**Solution**: The problem is physical - height, obstruction, antenna orientation, or coax loss.

**Why**: If the settings were wrong it would not have worked on the desk either. Settings problems are all-or-nothing; physical problems are gradual.

---
