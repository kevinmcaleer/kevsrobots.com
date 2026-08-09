---
layout: lesson
title: Where to Put the Repeater
author: Kevin McAleer
type: page
cover: /learn/meshcore/assets/cover.png
date: 2026-08-05
previous: 07_repeater_setup.html
next: 09_client_setup.html
description: Height, power, weatherproofing and solar - turning a dev board into infrastructure
percent: 54
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


A repeater is only as good as its view of the world. This lesson is about the unglamorous physical stuff that decides whether your mesh covers a street or a county.

---

## Height is everything

At 868MHz you are working with something close to line of sight, softened a little by diffraction. The single biggest thing you control is how high the antenna is.

The rough rule of thumb for radio horizon:

> distance in km ≈ 4.12 × (√height_1 + √height_2), with heights in metres

| Repeater height | Client at 1.5m | Approx. radio horizon |
|---|---|---|
| 2m (desk) | 1.5m | ~11 km |
| 6m (roof gutter) | 1.5m | ~15 km |
| 15m (chimney or mast) | 1.5m | ~21 km |
| 100m (hilltop) | 1.5m | ~46 km |
{:class="table table-single"}

Real world results will be a lot less than this - buildings, trees and terrain all take their cut - but the *shape* of the table is right. Doubling your height buys you far more than doubling your transmit power ever will.

---

## Ranking the locations

**Best:** An external mast or pole above the roofline, antenna vertical, clear of the building.

**Very good:** A loft. Warm, dry, and you only lose a few dB through tiles - unless you have foil-backed insulation or a slate roof, which are brutal at 868MHz.

**Good:** An upstairs window, antenna against the glass. Cheap, easy, surprisingly effective in a direction.

**Poor:** A shelf downstairs, indoors, surrounded by other electronics.

**Worst:** Inside a metal enclosure, next to a router, behind a fridge. You have built a very expensive Faraday cage.

---

## Powering it

A repeater wants to run 24/7, and it is a low-power thing - a Heltec V3 idles at a few tens of milliamps and peaks around 120mA while transmitting.

**Mains USB** - a plain 5V phone charger is ideal. Cheapest and most reliable.

**USB power bank** - fine for a temporary or portable repeater, but be aware that many power banks shut down when the load drops too low, which is exactly what a mostly-idle repeater does. Look for one with a "low current" or "always on" mode.

**LiPo battery** - a 3.7V cell on the SH1.25 connector, charged over USB. A 2000mAh cell will typically run a Heltec V3 repeater for a day or two.

**Solar** - a 5-10W panel, a charge controller and a 18650 pack will keep a repeater alive indefinitely in a UK summer and will struggle in a UK December. If you are serious about solar, look at the **Heltec T114** instead - it is nRF52 based and uses a fraction of the power of the ESP32-S3.

---

## Weatherproofing

If it goes outside, it needs to survive outside.

- **A cheap IP65 junction box** from a DIY shop is perfect. Drill one hole for an SMA bulkhead connector and one for the cable gland
- **Put the board inside the box and the antenna outside**, connected through the bulkhead. Do not try to transmit from inside a sealed plastic box with the antenna inside it - it mostly works, and mostly is not good enough
- **Add a silica gel sachet.** Condensation is what actually kills outdoor electronics, not rain
- **Face the gland downwards** so water runs away from it, and put a drip loop in the cable
- **UV matters.** Cheap plastic boxes go brittle after a couple of summers. Black or grey UV-stable boxes last

---

## Settings for a fixed repeater

Once it is in place, a few settings are worth revisiting from the console:

```text
set tx 20
set advert.interval 60
set flood.advert.interval 12
set flood.max 3
```

Full transmit power makes sense for a mains powered fixed node. Modest advert intervals keep you a good neighbour.

If your repeater is battery powered, consider:

```text
powersaving on
set tx 17
```

---

## Being a good mesh citizen

- **Do not run a repeater on a moving vehicle.** Paths break constantly and force the network back to flooding
- **Do not set aggressive advert intervals.** Every flood advert is rebroadcast by every repeater in the region
- **Do register your node with your local community** so people know it exists and who owns it
- **Do set `owner.info`** so someone can find you if your node misbehaves:

```text
set owner.info Kev - kevsrobots.com
```

---

## Try it Yourself

1. Walk your client node away from the repeater until messages start failing. Note the distance. Now move the repeater one floor up and repeat.
2. Run `stats-radio` before and after moving the repeater. How much did the noise floor change?
3. Challenge: work out the radio horizon for the highest point you can reach, using the formula above.

---

## Common Issues

**Problem**: My repeater keeps dropping off overnight.

**Solution**: Suspect the power supply, particularly if it is a USB power bank.

**Why**: Many power banks auto-shutdown below a current threshold, and an idle repeater draws very little.

**Problem**: Range got *worse* after I put it in a box outside.

**Solution**: Get the antenna outside the box on a bulkhead connector, and check the coax is short.

**Why**: Enclosures, especially damp ones, detune and absorb. Thin coax at 868MHz is lossy - roughly 1dB per metre for RG174.

**Problem**: The loft install performs badly.

**Solution**: Check for foil-backed insulation.

**Why**: Foil-backed insulation board is effectively an RF mirror and will kill 868MHz almost completely.

---
