---
layout: lesson
title: Introduction
author: Kevin McAleer
type: page
cover: /learn/meshcore_rooms_sensors/assets/banners/00_intro.jpg
date: 2026-08-05
next: 01_beyond_repeaters.html
description: Build the other two MeshCore node types - a room server and a sensor
  node
percent: 5
duration: 7
quiz_total: 53
date_updated: 2026-11-24
navigation:
- name: MeshCore Room Servers and Sensors
- content:
  - section: Overview
    content:
    - name: Introduction
      link: 00_intro.html
    - name: Beyond Repeaters and Clients
      link: 01_beyond_repeaters.html
    - name: The Hardware
      link: 02_hardware.html
  - section: The Room Server
    content:
    - name: What is a Room Server?
      link: 03_what_is_a_room_server.html
    - name: Flashing the Room Server
      link: 04_flashing_the_room_server.html
    - name: Configuring the Room Server
      link: 05_room_server_config.html
    - name: Joining a Room
      link: 06_joining_a_room.html
    - name: Permissions and Moderation
      link: 07_permissions.html
    - name: Running a Room Server
      link: 08_room_operations.html
  - section: The Sensor Node
    content:
    - name: What is a Sensor Node?
      link: 09_what_is_a_sensor_node.html
    - name: Flashing the Sensor Node
      link: 10_flashing_the_sensor.html
    - name: Building an Environment Sensor
      link: 11_environment_sensor.html
    - name: Battery and Solar Telemetry
      link: 12_battery_telemetry.html
    - name: Contact and Level Sensing
      link: 13_contact_and_level.html
    - name: Reading Telemetry
      link: 14_reading_telemetry.html
  - section: Going Further
    content:
    - name: Troubleshooting
      link: 15_troubleshooting.html
    - name: Where to Go Next
      link: 16_next_steps.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

Ahoy there makers! In [part one](/learn/meshcore/) we built a repeater and a client, and got a message across town with no phone signal and no subscription. That course covered conversation. This one covers the other two things a mesh can do: **remember**, and **measure**.

---

## Overview

MeshCore has four roles. You have already built two of them:

| Role | What it does | Covered in |
|---|---|---|
| Companion | Your phone-connected client | Part 1 |
| Repeater | Extends range by forwarding packets | Part 1 |
| **Room Server** | Stores messages for people who were out of range | **This course** |
| **Sensor** | Reports telemetry from somewhere you are not | **This course** |
{:class="table table-single"}

The problem with a plain mesh is that it has no memory. If you were out of range when a message was sent, you missed it - permanently. A **room server** fixes that: it holds recent posts and hands them over when you come back into range.

The other gap is that a mesh only carries what a human types. A **sensor node** fills that in: a board in a greenhouse, on a water tank, or at the bottom of a field, answering questions about the physical world across kilometres with no WiFi and no subscription.

---

## Course Content

- What room servers and sensor nodes add to a mesh, and when each is worth building
- The hardware for both, including which sensors MeshCore actually supports
- Flashing and configuring a room server
- Admin passwords, guest passwords and read-only access
- Joining a room from the app, posting, and catching up on what you missed
- Permission tiers, access control lists and moderating a shared room
- Powering a room server, and the one mistake that will lose all your posts
- How MeshCore's pull-based telemetry works, and why it is so power-efficient
- Flashing a sensor node and letting it auto-detect what you have wired up
- Building a BME280 environment sensor for a greenhouse or loft
- Reporting battery and solar voltage from a remote node
- Contact and level sensing - what works out of the box, and what needs custom firmware
- Requesting telemetry from your phone and reading what comes back
- Troubleshooting both node types
- Getting telemetry off the mesh and into your own tools, and what MeshCore's built-in bridge is really for

---

## Key Results

After completing this course, you will:

- Have a working room server your local group can post to and catch up on
- Understand MeshCore's three permission tiers and how to apply them
- Have a working sensor node reporting real environmental data over LoRa
- Be able to request and interpret telemetry from any node in your mesh
- Know how to power both node types for long unattended runs
- Know the options for getting mesh telemetry into your own dashboards

---

## What you'll need

**Prerequisite**

- [Off-Grid Messaging with MeshCore](/learn/meshcore/) - part one. This course assumes you have a working repeater and client, that your radio settings are already correct for your country, and that you know how to reach a serial console
- Helpful but optional: [I2C and SPI](/learn/i2c_spi/) for how the sensor bus works, and [Electronics Basics](/learn/electronics_basics/) if wiring is new to you

**Hardware**

- 1 or 2 more Heltec WiFi LoRa 32 V3 boards in the **868MHz** version (UK and EU; use the 915MHz version in the US) - one per role you want to build
- An antenna for each new board, matching its frequency. **Never power a LoRa board without its antenna fitted**
- A **BME280** breakout (the I2C version) for the environment build
- Optional: an **INA219** or **INA3221** breakout for proper battery and solar current monitoring
- Optional: a **VL53L0X** distance sensor for tank level
- Dupont jumper wires, four per sensor
- A USB-C **data** cable
- A single-cell LiPo battery with a protection circuit and a connector that matches your board, for the battery builds (check the polarity before plugging in, see lesson 8)

**Skills**

- Still no soldering strictly required, though a soldered header is far more reliable than a push-fit one
- Still no programming required, unless you take the custom firmware route in lesson 13

---

## How the course works

Same as part one. Commands typed into a node's console look like this:

```text
set guest.password hedgehog
```

Values you change for your own setup appear in angle brackets, like `set name <your-room-name>`.

Every lesson opens with an objective and ends with **Try it Yourself** exercises, **Common Issues** (problem, solution, why) and a short quiz. Where a lesson has a classroom catch, you will find a **Teachers** note.

MeshCore is moving quickly. Commands and settings in this course were checked against the official MeshCore documentation and firmware source at the time of writing, but menu names and defaults can change between firmware versions. If something differs on your board, trust what `ver` and the app show you.

One rule applies to every lesson: radio rules differ by country. The UK and EU use the 868MHz band with limits on transmit power and duty cycle, and the US uses 915MHz. Check the rules for where you live before you transmit.

> **Teachers**: This course suits secondary-school and club learners aged about 13 and up who have finished part one (or have an adult who has). Plan about 45 to 60 minutes per lesson; lessons 4, 10 and 11 are the hands-on ones. A group of three can share one room server board and one sensor board. With no hardware at all, learners can still do the reading, the diagrams, the quizzes and the planning exercises, and can send commands by following the examples. Things to settle before you start: who holds the admin passwords, whether pupils may transmit on school premises under your local radio rules, and where charged LiPo batteries are stored. Never power a radio without its antenna.

Let's get building.

---

## Check your understanding

{% quiz %}
? Why can a room server show you messages that were posted while you were out of range?
- [x] It stores recent posts and sends the ones you have not seen when you log in
- [ ] It re-broadcasts every message forever
- [ ] It forwards your phone's mobile data
> A room server keeps a buffer of recent posts and works out what each user has not seen when they log in.

? What does a MeshCore sensor node do when nobody asks it anything?
- [x] Stays quiet and measures locally
- [ ] Transmits every reading on a timer
- [ ] Acts as a repeater for the whole mesh
> Sensors are pull-based: they sample locally and only transmit when a client requests telemetry.

? Which of these must you do before powering any LoRa board?
- [x] Fit the antenna
- [ ] Flash the sensor firmware
- [ ] Set the clock
> Transmitting into an open antenna port can damage the radio, so the antenna goes on first.

{% endquiz %}
