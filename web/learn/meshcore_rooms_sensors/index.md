---
layout: course_overview
title: MeshCore Room Servers and Sensors
author: Kevin McAleer
cover: /learn/meshcore_rooms_sensors/assets/cover.png
date: 2026-08-05
description: Part 2 of the MeshCore series. Build the other two node types - a room
  server that stores messages for people who were out of range, and a sensor node
  that reports temperature, battery and tank levels across kilometres.
level: intermediate
course_duration: 2h 11m
lesson_count: 17
first_lesson: 00_intro.html
quiz_total: 53
outcomes:
- Flash, name, password and configure a MeshCore room server
- Join a room from the app and catch up on posts you missed
- Apply guest, read-only and admin permissions to a shared room
- Flash a sensor node and wire a BME280 to its sensor bus
- Read battery and environment telemetry from across the mesh
- Choose between supported I2C sensors and custom firmware for level and contact sensing
prerequisites:
- Off-Grid Messaging with MeshCore (part one)
you_need:
- One or two Heltec WiFi LoRa 32 V3 boards in the right frequency version for your
  country, with antennas
- A BME280 I2C breakout and four Dupont jumper wires
- A USB-C data cable and a computer with Chrome or Edge
- The MeshCore app on a phone, and a working repeater and companion from part one
- Optional - an INA219, a VL53L0X and a protected single-cell LiPo
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
