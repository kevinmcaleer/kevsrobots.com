---
layout: lesson
title: Antennas and the Law
author: Kevin McAleer
type: page
cover: /learn/meshcore/assets/cover.png
date: 2026-08-05
previous: 03_the_hardware.html
next: 05_flashing.html
description: Antenna basics, the golden rule of never transmitting without one, and
  what Ofcom allows on 868MHz
percent: 30
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


Your antenna matters more than your transmit power. A £5 antenna change can do more for your range than any setting in the firmware. This lesson covers what to buy, what not to do, and what the rules are in the UK.

---

## The golden rule

> **Never power on a LoRa board without an antenna connected.**

**Why this matters:** With no antenna, the RF energy the SX1262 generates has nowhere to go. It reflects straight back into the power amplifier. Do this repeatedly and you will damage the radio - permanently, and in a way that looks exactly like "my board mysteriously has terrible range now".

Fit the antenna *before* you plug in the USB cable. Every time. It is a two second habit that saves £15 boards.

---

## What makes a good 868MHz antenna

**Tuned for the band.** An antenna cut for 433MHz or 2.4GHz will radiate badly at 868MHz. Buy one explicitly sold as 868MHz or 863-870MHz.

**Vertical.** LoRa nodes almost universally use vertical polarisation. If your repeater's antenna is vertical and your client's is lying flat on a desk, you throw away up to 20dB. That is most of your range, gone, for free.

**Gain, honestly stated.** A cheap "stubby" is typically 1-2dBi. A half-wave whip is around 2-3dBi. A collinear on a mast might be 5-6dBi. Higher gain squashes the radiation pattern flatter - which is *ideal* for a rooftop repeater covering a town, and *bad* for a handheld you carry up a hill.

**Height beats everything.** Seriously. Getting an antenna 5 metres higher will usually beat any upgrade you can buy, because LoRa at these ranges is dominated by line of sight and Fresnel zone clearance.

---

## The upgrade order

If you have limited budget, spend it in this order:

1. **Get the repeater antenna higher** - free, or the cost of a bracket
2. **Mount it outside rather than inside** - a single brick wall can cost you 10dB
3. **Buy a proper half-wave 868MHz whip** instead of the stubby
4. **Only then** think about high gain collinears and low loss coax

---

## Coax matters at 868MHz

If you mount the antenna on a pole and run cable to the board, remember that thin coax is lossy. Cheap RG174 loses roughly 1dB per metre at 868MHz. Five metres of it throws away over half your transmit power *and* half your receive sensitivity.

**Better plan:** put the whole board in a small weatherproof box right at the antenna, and run the *USB power* up the pole instead. Power over a long cable is much more forgiving than RF over a long cable.

---

## The legal bit - UK and 868MHz

The 868MHz band is a licence-exempt Short Range Device band. You do not need an amateur radio licence, but you do have to stay inside the rules, which Ofcom publishes in **Interface Requirement IR 2030**.

The sub-band MeshCore uses in the UK and EU is **869.4 - 869.65MHz**, and in that slice you are allowed:

- Up to **500mW ERP** (Effective Radiated Power)
- A maximum **10% duty cycle** - no more than 6 minutes of transmitting in any hour

**How does that compare to your board?** The Heltec V3 puts out around **+21dBm** - Heltec quote 21 &plusmn;1dBm - which is roughly 125 to 160mW. With a typical 2-3dBi antenna your ERP lands comfortably under the 500mW ceiling, so a stock board on stock settings is fine.

**The duty cycle is the one to respect.** It is not usually a problem for text messaging, but it is exactly why the community keeps settings narrow and fast - and why a badly configured node that adverts every few minutes is genuinely antisocial.

---

## A note on other bands

You may see MeshCore nodes on 433MHz. In the UK, 433MHz is a licence-exempt SRD band too, but with a much lower power allowance (typically 10mW ERP) and it shares space with car key fobs and weather stations. Stick to 868MHz unless you know exactly why you are not.

> ## Not legal advice
>
> Rules change and this is a summary, not a substitute for the actual document. If you are building anything beyond a hobby node - especially anything on a mast or with an amplifier - read the current version of Ofcom IR 2030 yourself.

---

## Try it Yourself

1. Look at the antenna that came with your board. Is 868MHz printed on it anywhere?
2. Stand your client node's antenna vertically, walk to the edge of your range, then tip it horizontal. Watch the signal collapse.
3. Challenge: find the highest point in your house or garden that has power. Measure how much higher it is than your desk. That is your free range upgrade.

---

## Common Issues

**Problem**: My range is terrible even though both nodes are outside.

**Solution**: Check antenna polarisation and get at least one node higher.

**Why**: Mismatched polarisation and obstructed Fresnel zones cost far more than any firmware setting.

**Problem**: I powered the board up without an antenna once - is it dead?

**Solution**: Probably not, if it was only briefly and it had not transmitted. Test the range against a known good board.

**Why**: Damage is cumulative and comes from transmitting into an open circuit. Booting and immediately powering down is usually survivable - just do not make a habit of it.

---
