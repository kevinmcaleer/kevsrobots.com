---
layout: lesson
title: Where to Go Next
author: Kevin McAleer
type: page
cover: /learn/meshcore_rooms_sensors/assets/banners/16_next_steps.jpg
date: 2026-08-05
previous: 15_troubleshooting.html
description: Bridging to MQTT, Home Assistant and Grafana, and a capstone that ties
  the whole network together
percent: 100
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

By the end of this lesson you will be able to describe the four MeshCore roles, plan a capstone that combines a repeater, room server and sensor, and find where to learn more.

You now have all four MeshCore roles under your belt. Here is where a network like this goes once the basics are working.

---

## What you have built

Across both courses:

- A **repeater** extending your range from a good high site
- A **companion client** in your pocket, messaging over it
- A **room server** holding recent posts for people who were out of range
- A **sensor node** reporting real measurements from somewhere you are not

That is a complete, self-contained communications and telemetry network with no internet and no subscription. Worth pausing on.

---

## Get the data off the mesh

The natural next step is joining your mesh to the systems you already run. Be careful here: the MeshCore firmware has a **bridge** feature, but it is not an MQTT client.

### What the built-in bridge is for

The CLI reference lists `get bridge.type`, `bridge.enabled`, `bridge.source`, `bridge.baud` and `bridge.secret`, and says they apply when bridge support is compiled in. The bridge transports are RS-232 (serial) and ESP-NOW, and the Heltec V3 build list in the repository includes repeater bridge variants for both. In other words, the bridge links two repeaters over a wired or ESP-NOW hop. It does not publish to an MQTT broker.

### Getting telemetry onto a computer

To get readings into Home Assistant, Grafana or a database, you need a computer connected to a companion node. Options include `meshcore-cli`, the official command-line tool, or community libraries and bridges that talk to a companion over USB. We have not checked those projects for this course, so look at their own documentation and check they match your firmware version before you rely on them.

**Because telemetry is CayenneLPP**, a lot of tools already understand the format, but the step from radio to broker is the part you will have to choose carefully.

**Keep it in proportion.** A node that shows readings in the app may be all you need. Only add a broker, dashboards and a computer that is always on if you will actually use them.

---

## More sensors

Now that one sensor works, the pattern repeats cheaply:

**Beginner**

- A second BME280 node comparing indoor and outdoor
- A battery-only node on your solar repeater, just to watch its health

**Intermediate**

- A VL53L0X on a water butt, reporting level through the summer
- An INA3221 on a solar installation, watching panel, battery and load together
- A soil moisture node on a RAK12035 in a polytunnel

**Advanced**

- Custom firmware adding a DS18B20 probe or a contact input
- A sensor network across a smallholding, all bridged to one dashboard
- Threshold alerts feeding a real notification pipeline

---

## More rooms

**A second room server** for a different purpose - one general, one for event coordination.

**A room server at a second site** so a group split across two valleys each have a local one.

**Battery-backed properly**, so a power cut stops being a data loss event.

---

## Grow the network

The most valuable upgrade is rarely another node of your own - it is another *person's* node.

- **[map.meshcore.io](https://map.meshcore.io)** - who is on air near you
- **[localmesh.co.uk](https://localmesh.co.uk)** - the UK community, settings and coordination
- **The `docs` folder of the repository below** - the official CLI reference and FAQ
- **[github.com/meshcore-dev/MeshCore](https://github.com/meshcore-dev/MeshCore)** - the firmware, and the place to start if you are building your own

Register your nodes. Tell people what they are and who owns them. A mesh with fifty owners is infrastructure; a mesh with one is a hobby.

---

## Your capstone challenge

Here is a test that exercises everything in both courses:

> **Set up a sensor at the far edge of your coverage, reachable only through your repeater. Post its reading to your room server. Then walk out of range, come back, and read the post you missed.**

That single exercise requires: a repeater placed well enough to reach the sensor, matching radio settings across four nodes, a working telemetry request, a room server holding the post, and a client that syncs on login. If you can do that, you understand this system.

Bonus: get the same reading onto a computer using `meshcore-cli` and note it down for a week.

---

> **Teachers**: The capstone needs a free afternoon and two or three working rooms-and-sensor groups. Check the legal transmit rules for your site before any outdoor range test, keep antennas fitted, and agree who carries which phone. A group can submit a short report with their radio settings, node names and readings as evidence.

---

## Try it Yourself

1. Write down the four MeshCore roles and, for each, one thing in your own life it would do.
2. Draw your own network map with every node, its role and its radio settings.
3. Challenge: carry out the capstone below and note what went wrong first.

---

## Common Issues

- **Problem**: The capstone does not work end to end.
- **Solution**: Test each link on its own: sensor to phone, phone to room, then the walk-away test.
- **Why**: Several things are chained together, and debugging one link at a time is far quicker than all at once.

---

## Related courses

- [Off-Grid Messaging with MeshCore](/learn/meshcore/) - part one, if you arrived here first
- [I2C and SPI](/learn/i2c_spi/) - for how the sensor bus really works
- [Electronics Basics](/learn/electronics_basics/) - for voltage, current and safe wiring
- [Raspberry Pi Pico with MicroPython - GPIO Mastery](/learn/micropython_gpio/) - to read the same sensors from a Pico
- [Introduction to Linux](/learn/linux_intro/) - useful for running any always-on computer in your setup
- [Mini-Rack 3D Design Tutorial](/learn/mini_rack/) - if your computer ends up in a home lab

---

## Thank you

Thanks for working through both courses. If you build a room server that a group actually uses, or get a sensor reading in from somewhere genuinely remote, I would love to see it.

And if something here goes out of date - which it will, because this ecosystem moves quickly - please tell me.

Now go and put something in a field.

---

## Check your understanding

{% quiz %}
? What is the MeshCore bridge feature actually for?
- [x] Linking nodes over RS-232 or ESP-NOW, not publishing to MQTT
- [ ] Publishing telemetry straight to MQTT
- [ ] Sending posts to social media
> The CLI reference lists serial and ESP-NOW bridge types, which link repeaters.

? Why does the lesson suggest logging telemetry at the receiving end for long-term history?
- [x] The node only keeps a short rolling buffer
- [ ] The radio cannot send numbers
- [ ] Phones cannot store text
> The sensor example keeps 24 hours of battery data.

? Which role is the one you check weekly for battery and uptime?
- [x] Every remote node, especially sensors
- [ ] Only the phone
- [ ] None, they are maintenance free
> Resets in uptime are an early warning.

{% endquiz %}
