---
layout: lesson
title: Radio Settings for Your Region
author: Kevin McAleer
type: page
cover: /learn/meshcore/assets/banners/06_radio_settings.jpg
date: 2026-08-05
previous: 05_flashing.html
next: 07_repeater_setup.html
description: Pick the right regional preset, set it on the repeater, check transmit
  power and duty cycle, and see why both nodes must match
percent: 42
duration: 5
quiz_total: 45
date_updated: 2026-11-24
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


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to set and check the four radio settings on the repeater, and choose power and duty cycle values that suit your region.

Both boards are flashed. Now we make sure they can hear each other, and the other MeshCore nodes near you.

---

## The rule that catches everyone

**Frequency, bandwidth and spreading factor must match on every node that needs to talk.** There is no negotiation or fallback. If one is different, the nodes are silent strangers. That includes every other MeshCore node in your area, which is why communities agree on presets.

---

## Regional presets

MeshCore apps ship with named presets for each region. As I checked them while writing this lesson, the UK and EU preset is **EU/UK (Narrow)**:

| Setting | EU/UK (Narrow) |
|---|---|
| Frequency | 869.618 MHz |
| Bandwidth | 62.5 kHz |
| Spreading Factor | 8 |
| Coding Rate | 8 |
{:class="table table-single"}

For comparison, the USA preset I found was 910.525MHz, SF7, BW62.5, CR5. The MeshCore FAQ says many regions moved to narrow presets from October 2025, and an older EU/UK setting (869.525MHz, SF11, BW250) is now marked deprecated. **Presets change.** Before you commit, look at the preset list in the flasher or app, and ask your local community. Use the preset for *your* country, not the one in this lesson, and never transmit on a frequency your regulator does not allow.

```mermaid
flowchart TD
    S["Choose your country's preset"] --> R["Repeater: set radio"]
    S --> C["Client: pick same preset in the app"]
    R --> M{"Same freq, BW, SF?"}
    C --> M
    M -->|"Yes"| H["Nodes can hear each other"]
    M -->|"No"| X["Silent strangers"]
```

---

## Setting them on the repeater

Connect the console (lesson 5) and check what is there:

```text
get radio
```

To set them explicitly (UK values shown):

```text
set radio 869.618,62.5,8,8
```

The command takes `frequency,bandwidth,spreading factor,coding rate`. Frequency is in **MHz** and bandwidth in **kHz**. Then confirm, and restart so the settings are definitely in use:

```text
get radio
reboot
```

---

## Setting them on the client

The companion is configured from the app. After pairing in lesson 9, open the app's settings for the node and choose your regional preset from the radio settings. The exact menu names depend on the app version.

---

## Transmit power

```text
get tx
```

MeshCore accepts a power in dBm (the docs list 1 to 22). Your board has its own hardware limit, around 21dBm on the Heltec V3. To change it:

```text
set tx 20
```

Lower power uses less battery. Check your region's limit from lesson 4 before raising it.

---

## Duty cycle

Newer firmware (v1.15 and later, per the MeshCore docs) has a duty cycle limit:

```text
get dutycycle
set dutycycle 10
```

The docs say the default is 50%. In the UK and EU the sub-band we use is commonly limited to 10%, so `set dutycycle 10` is the setting to try there. Older firmware used `set af <value>` (airtime factor), which the docs now call deprecated. If `get dutycycle` is not recognised, your firmware is older: update it, or use `af`. Whichever you use, the responsibility for staying legal is yours.

---

## Try it Yourself

1. Run `get radio` on your repeater and write the four numbers down. You need them for the client.
2. Deliberately set your client to a different spreading factor and try to see the repeater. Confirm that nothing happens, then put it back.
3. Challenge: calculate the airtime of a 30 byte packet at SF8/BW62.5 and at SF7/BW62.5 with a LoRa calculator. How much airtime does the lower SF save?

---

## Common Issues

- **Problem**: `set freq 869618` does not work.
- **Solution**: Frequency is in MHz, so use `869.618`.
- **Why**: MeshCore expects megahertz with a decimal point.

- **Problem**: My nodes cannot hear each other even though I believe the settings match.
- **Solution**: Run `get radio` on the repeater, read the app's settings on the client, and reboot both.
- **Why**: Assuming is not checking; one digit different is enough to silence a node.

- **Problem**: I hear my own nodes but not the local community repeaters.
- **Solution**: Ask your local group which preset they use.
- **Why**: Regional presets have changed over time and some repeaters run older settings.

> **Teachers**: the preset must be legal in your own country, so decide it once and write it on the board, then have every pair copy it. A wrong frequency is the cause of nearly every "it does not work" moment, and a mistyped frequency is also the main way a learner might transmit outside the permitted band.

## Check your understanding

{% quiz %}
? Which command shows the repeater's current frequency, bandwidth, SF and CR?
- [x] `get radio`
- [ ] `get name`
- [ ] `advert`
> `get radio` prints all four values, and `set radio` takes them in the same order.

? In `set radio 869.618,62.5,8,8`, what is 62.5?
- [ ] Spreading factor
- [x] Bandwidth in kHz
- [ ] Transmit power
> The order is frequency, bandwidth, spreading factor, coding rate.

? Why should you check the preset list before copying this lesson's numbers?
- [ ] Presets never change
- [x] Regional presets change, and yours must be legal where you live
- [ ] The app picks a random one
> The lesson explains that presets have moved over time and rules differ by country.
{% endquiz %}

---
