---
title: I2C Communication Protocol
description: >
    A versatile and widely used two-wire communication protocol for connecting low-speed devices in embedded systems.
layout: stem
date: 2024-04-30
author: Kevin McAleer
difficulty: beginner
cover: /assets/img/stem/i2c_cover.png
hero: /assets/img/stem/i2c_hero.png
mode: light
tags:
  - I2C
  - Communication Protocol
  - Microcontrollers
  - Embedded Systems
  - Electronics
  - STEM
category:
  - Communication Protocols
---

## What is I2C?

I2C, or Inter-Integrated Circuit, is a serial communication protocol that allows multiple slave devices to be controlled by a single or multiple master devices. It uses only two wires, making it ideal for connecting peripherals like sensors, displays, and other microcontrollers with minimal wiring.

---

## Description

I2C uses two lines: SDA (Serial Data Line) and SCL (Serial Clock Line), to communicate between devices. It supports multiple masters and slaves, addressing each device uniquely through 7-bit or 10-bit addressing.

---

### Key Features of I2C

- **Two-Wire Interface**: Only uses two wires, reducing complexity in system design.
- **Multi-Master Capability**: Allows multiple master devices to control the bus.
- **Clock Stretching**: Slaves can slow down communication by holding the clock line low.
- **Arbitration**: Prevents data collision by allowing only one master to control the bus at a time.
- **Addressing**: Supports up to 112 devices on the same bus with 7-bit addressing.

---

### Application Areas

**Consumer Electronics**: Used in smartphones, televisions, and other smart devices to control auxiliary components like sensors and touch screens.

**Automotive Systems**: Employed in vehicle internal networks to communicate with sensors and control systems.

**Industrial Automation**: Integral in managing communication between various sensors and actuaries in automated systems.

---

## Conventions

I2C is typically used in environments where communication simplicity and cost-effectiveness are prioritized over speed. It is suitable for applications where devices are relatively close to each other.

---

## Pros and Cons

**Pros**:
- Simple and efficient for small-scale communication.
- Reduces wiring complexity significantly.
- Scalable to a reasonable number of devices without additional hardware.

**Cons**:
- Limited by bus speed and length, which affects the number of devices and response times.
- Susceptible to interference in noisy environments.
- Requires careful handling of bus arbitration and clock synchronization.

---
