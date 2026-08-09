---
title: Where to Go Next
description: Bridging to MQTT, Home Assistant and Grafana, and a capstone that ties the whole network together
type: page
layout: lesson
---

You now have all four MeshCore roles under your belt. Here is where a network like this goes once the basics are working.

---

## What you have built

Across both courses:

- A **repeater** extending your range from a good high site
- A **companion client** in your pocket, messaging over it
- A **room server** holding recent posts for people who were out of range
- A **sensor node** reporting real measurements from somewhere you are not

That is a complete, self-contained communications and telemetry network with no internet, no subscription, and about £60 of hardware. Worth pausing on.

---

## Get the data off the mesh

The natural next step is joining your mesh to the systems you already run.

### MQTT

A node bridged to MQTT publishes telemetry to a broker, and from there it goes anywhere. This is the standard route out of a mesh and into your own infrastructure.

MeshCore has bridge functionality built in:

```text
get bridge.type
set bridge.enabled on
set bridge.source logTx
set bridge.baud 115200
set bridge.secret <a-shared-secret>
```

Bridge options vary by transport - serial, RS-232 and ESPNow all appear depending on the build. Check `get bridge.type` on your hardware to see what you have.

### Home Assistant

With telemetry on MQTT, Home Assistant picks it up through MQTT discovery. From there your greenhouse temperature is a sensor entity like any other - dashboards, history graphs, automations, notifications.

**The satisfying version:** a Home Assistant automation that texts you when the greenhouse sensor drops below 2°C, using data that travelled two kilometres over LoRa to get there.

### Grafana and InfluxDB

For long-term history, push MQTT into InfluxDB and graph it in Grafana. This solves the rolling-buffer limitation properly: the node keeps a short window, and your database keeps forever.

**Because telemetry is CayenneLPP**, all of these tools already understand the format. You are not writing a parser.

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

**A second room server** for a different purpose - one general, one for event coordination. £15 and a socket.

**A room server at a second site** so a group split across two valleys each have a local one.

**Battery-backed properly**, so a power cut stops being a data loss event.

---

## Grow the network

The most valuable upgrade is rarely another node of your own - it is another *person's* node.

- **[map.meshcore.io](https://map.meshcore.io)** - who is on air near you
- **[localmesh.co.uk](https://localmesh.co.uk)** - the UK community, settings and coordination
- **[docs.meshcore.io](https://docs.meshcore.io)** - official documentation, including the full CLI reference
- **[github.com/meshcore-dev/MeshCore](https://github.com/meshcore-dev/MeshCore)** - the firmware, and the place to start if you are building your own

Register your nodes. Tell people what they are and who owns them. A mesh with fifty owners is infrastructure; a mesh with one is a hobby.

---

## Your capstone challenge

Here is a test that exercises everything in both courses:

> **Set up a sensor at the far edge of your coverage, reachable only through your repeater. Post its reading to your room server. Then walk out of range, come back, and read the post you missed.**

That single exercise requires: a repeater placed well enough to reach the sensor, matching radio settings across four nodes, a working telemetry request, a room server holding the post, and a client that syncs on login. If you can do that, you understand this system.

Bonus: bridge the same reading to Home Assistant and see it appear on a dashboard.

---

## Related courses

- [Off-Grid Messaging with MeshCore](/learn/meshcore/) - part one, if you arrived here first
- [Raspberry Pi Pico with MicroPython - GPIO Mastery](/learn/micropython_gpio/) - for the sensor and I2C fundamentals
- [Mini-Rack 3D Design Tutorial](/learn/mini_rack/) - if your bridge and broker end up in a home lab
- [Introduction to Linux](/learn/linux_intro/) - useful for running an MQTT broker

---

## Thank you

Thanks for working through both courses. If you build a room server that a group actually uses, or get a sensor reading in from somewhere genuinely remote, I would love to see it.

And if something here goes out of date - which it will, because this ecosystem moves quickly - please tell me.

Now go and put something in a field.

---
