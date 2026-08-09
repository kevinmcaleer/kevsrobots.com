---
layout: lesson
title: Beyond Repeaters and Clients
author: Kevin McAleer
type: page
cover: /learn/meshcore_rooms_sensors/assets/cover.png
date: 2026-08-05
previous: 00_intro.html
next: 02_hardware.html
description: What room servers and sensor nodes add to a mesh, and when each is worth
  building
percent: 10
duration: 6
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


A repeater and a client give you a working network. So why add anything else? Because a plain mesh has two blind spots, and each of these roles fixes one of them.

![The four roles working together](/learn/meshcore_rooms_sensors/assets/network_map.svg){:class="img-fluid w-100"}

---

## Blind spot one: the mesh has no memory

Here is the situation every mesh user runs into within a week.

You are out walking. Someone posts to the local channel that the footpath at the reservoir is flooded. You are three miles away, behind a hill, with no repeater between you. The message floods across the mesh, does not reach you, and is gone.

You come back into range an hour later. Nothing happens. There is no inbox, no queue, no retry. **The message did not fail to deliver - it simply never existed as far as your node is concerned.**

This is not a bug. A LoRa mesh is a broadcast medium: nodes hear what is in the air at the moment it is in the air. Nobody is holding anything for you.

**A room server holds it for you.** It is a small board that sits on the mesh, keeps recent posts in a buffer, and hands over what you missed the moment you log back in. It turns a live broadcast into something much closer to a message board.

---

## Blind spot two: the mesh only carries what a human types

Your mesh can tell you anything - as long as a person is standing there to type it.

That is a real limitation the moment you care about something unattended. Is the greenhouse below freezing? Is the water tank at the far end of the field getting low? Did the shed door get left open? Is the solar repeater on the hill actually charging, or has it been running down for a fortnight?

**A sensor node answers those questions.** It is a board with something measurable wired to it, sitting somewhere you are not, waiting to be asked.

---

## When is a room server worth it?

**Build one if:**

- You have a group of three or more people using the mesh regularly
- People move in and out of coverage - which is to say, people carry their nodes
- You want a persistent noticeboard rather than a live chat
- You are running a club, a village group, or a scattered family

**Do not bother if:**

- It is just you and one other person, both usually in range
- You want private one-to-one messaging - direct messages already do that better

**The honest test:** if your group regularly says "sorry, missed that, what did I miss?", build a room server.

---

## When is a sensor node worth it?

**Build one if:**

- You need a reading from somewhere with no WiFi and no mains
- The thing you are measuring changes slowly - hourly is fine
- You already have mesh coverage over the spot, or a repeater that reaches it

**Do not bother if:**

- The location has WiFi. An ESP32 posting to Home Assistant over WiFi is simpler, faster and gives you far more data
- You need readings every few seconds. LoRa airtime does not support that, and your neighbours will not thank you
- You need to *control* something remotely. MeshCore sensors report; they are not actuators

**The honest test:** if the spot has WiFi, use WiFi. LoRa is for the places WiFi cannot reach.

---

## How they differ from what you have built

| | Companion | Repeater | Room Server | Sensor |
|---|---|---|---|---|
| Attached to a phone | Yes | No | No | No |
| Forwards other traffic | No | Yes | Optional | Off by default |
| Stores messages | No | No | Yes | No |
| Answers telemetry requests | - | Yes | Yes | Yes |
| Typical power source | Battery / pocket | Mains, 24/7 | Mains, 24/7 | Battery or solar |
| Needs to be high up | Helps | Critical | Helps | Not really |
{:class="table table-single"}

Two things in that table are worth pausing on.

**A room server can also repeat**, using `set repeat on`. The MeshCore community broadly advises against it - you lose admin features and you tie two jobs to one board. Lesson 8 covers why, and what to do instead.

**A sensor is a leaf by default.** Sensor firmware ships with forwarding disabled, because sensors are usually the most power-constrained thing on the mesh and forwarding other people's traffic is the fastest way to flatten a battery. You *can* turn it on with `set repeat on` - you almost certainly should not.

---

## What we are building

Over the rest of this course:

**Lessons 3 to 8** build a room server - flashed, named, passworded, joined from a phone, with permissions set up so your group can post but strangers cannot reconfigure it.

**Lessons 9 to 14** build a sensor node - flashed, wired to a BME280, reporting temperature, humidity, pressure and battery voltage back to your phone on request.

You can do either half on its own. If you only have one spare board, read both and pick the one that solves a problem you actually have.

---

## Try it Yourself

1. Look at your own mesh use over the last week. How many times did someone miss a message? That number is your case for a room server.
2. Walk around your property and list every spot where you would like a reading but have no WiFi. That is your sensor shortlist.
3. Challenge: work out which single board would give you the most value - a second repeater, a room server, or a sensor. There is no universal right answer, and thinking it through is the useful part.

---

## Common Issues

**Problem**: I built a room server and nobody uses it.

**Solution**: Check that people actually know the guest password and how to join, and that the room appears in their contact list.

**Why**: A room server is invisible until someone logs in. It is not like a channel that just appears - joining is a deliberate step, covered in lesson 6.

**Problem**: My sensor never sends me anything.

**Solution**: That is normal. MeshCore telemetry is pull-based - you have to ask.

**Why**: The node reads its sensors locally on a timer, but it only transmits when a client requests telemetry. Silence is the correct, power-saving state. See lesson 9.

---
