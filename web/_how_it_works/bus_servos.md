---
layout: how_it_works
title: Bus Servos
short_title: How it works - Bus Servos
description: Learn how serial bus servos like the Feetech STS3215 work, communicate and chain together
short_description: Learn how serial bus servos work
date: 2026-04-19
author: Kevin McAleer
excerpt: Bus servos like the Feetech STS3215 daisy chain on a single wire, report their own position, and make robotic arms a whole lot tidier.
cover: /assets/img/how_it_works/sts3215.png
tags:
 - Bus Servos
 - STS3215
 - Feetech
 - Serial
 - UART
 - How it works
---

A `Bus Servo` is a smart servo that talks to your microcontroller over a **serial bus** instead of a PWM pulse. You can daisy chain many of them on a single data wire, address each one by ID, and ask it to report back on its own position, load, voltage and temperature. The **Feetech STS3215** is one of the most popular examples — and a brilliant way to understand how the whole category works.

---

{:toc}
* toc

---

## Why Bus Servos Exist

If you've ever built a robot arm with classic PWM servos you'll know the pain:

* Every servo needs its **own signal wire** back to the microcontroller — six joints = six GPIO pins.
* The servo is a **one-way actuator**. You tell it where to go, but it can't tell you where it actually is.
* Any mechanical load on the output shaft can deflect it from the commanded angle with no way for you to detect it.

Bus servos were designed to fix all three of those problems in one go — and the trick they use is to put a tiny microcontroller inside the servo itself.

---

## Inside a Bus Servo

A bus servo contains everything a regular hobby servo has — DC motor, gearbox, potentiometer or magnetic encoder, control circuit — plus a small **onboard microcontroller** that handles serial communications and closes a position-control loop in firmware.

That extra intelligence is what makes all the new behaviours possible:

* It can **listen** to commands on the serial bus and only respond to its own ID.
* It can **read** its internal position sensor and stream that data back.
* It can **monitor** motor current, voltage and temperature.
* It can **enforce** its own position limits, store calibration data, and remember them across power cycles.

---

## Serial, Not PWM

Instead of a 50 Hz pulse-width signal, bus servos speak a **serial protocol** over UART (Universal Asynchronous Receiver/Transmitter).

Most hobby-grade bus servos use **half-duplex UART**: a single data wire carries traffic in both directions, taking turns. At any given moment, one end of the bus is transmitting and every other device is listening.

The Feetech STS3215 runs at **1 Mbaud by default** — roughly a thousand times faster than I²C's standard 100 kHz bus — so commands and replies are effectively instant from a robotics point of view.

---

## The Packet Format

Every command on the STS3215 bus is a small binary packet. It follows a structure you'll also recognise from Dynamixel servos and many other serial buses:

| Byte(s) | Field | Purpose |
|---|---|---|
| `0xFF 0xFF` | Header | Marks the start of a packet |
| `ID` | Servo ID | Which servo should respond (0–253, or 254 for broadcast) |
| `Length` | Packet length | Number of remaining bytes (params + instruction + checksum) |
| `Instruction` | Command | Ping, read, write, reg-write, action, reset, sync-write |
| `Params…` | Parameters | Register address, values, lengths |
| `Checksum` | Error check | `~(ID + Length + Instruction + Params) & 0xFF` |
{:class="table table-single table-narrow"}

So to ping servo ID 1, the master sends just six bytes:

```
0xFF 0xFF 0x01 0x02 0x01 0xFB
```

The servo replies with a similar packet containing an **error byte** (0x00 if no error) and a matching checksum, proving it heard the request and is alive.

---

## The Control Table

Inside every STS3215 is a little memory map called the **control table** — a set of numbered registers you can read and write across the bus. Some registers live in EEPROM (persistent across power cycles), others in RAM (reset on power-up).

A few of the most useful registers on the STS3215:

| Register | Address | Purpose |
|---|---|---|
| Model number | 3 | Identify the servo model |
| ID | 5 | Change the servo's bus address |
| Baud rate | 6 | Set the UART speed |
| Min / Max angle limits | 9–12 | Hardware-enforced position limits |
| Torque enable | 40 | 1 = powered, 0 = free to move by hand |
| Acceleration | 41 | Ramp-up rate |
| Goal position | 42 | Target position (0–4095) |
| Goal speed | 46 | Commanded speed |
| Present position | 56 | Measured position right now |
| Present speed | 58 | Measured speed |
| Present load | 60 | How hard the motor is working |
| Present voltage | 62 | Bus voltage |
| Present temperature | 63 | Motor temperature |
| Moving | 66 | 1 while in motion, 0 when stopped |
{:class="table table-single table-narrow"}

Control = **write to registers**. Feedback = **read from registers**. That's the entire mental model.

---

## Daisy Chaining

Each STS3215 has two identical ports — one **in**, one **out**. Both carry **power, ground, and the data line** on the same connector, which means the entire arm can be wired with a single continuous chain.

```
Driver board ──► Servo 1 ──► Servo 2 ──► Servo 3 ──► …
```

Because every servo is listening to the same bus, each one needs a **unique ID**. Out of the factory, every STS3215 ships with `ID = 1`. Before you daisy chain them together, connect them one at a time and write a new ID into register 5.

Send a packet with ID 254 and every servo on the bus responds — that's the **broadcast address**, useful for sync-writes that drive multiple joints in the same instant.

---

## Half-Duplex and Bus Contention

Because the bus is half-duplex, only one device can transmit at a time. The master microcontroller enforces this with a simple rule:

1. **Transmit** a command.
2. **Switch the line around** (either electronically via a direction pin, or by relying on an open-drain buffer).
3. **Listen** for the reply, timing out after a millisecond or two if nothing comes back.

On a driver board like the [Waveshare Bus Servo Adapter](https://www.waveshare.com/wiki/Bus_Servo_Adapter_(A)), this direction switching is handled in hardware — you just connect the microcontroller's TX and RX pins and the board takes care of the rest.

---

## Position Resolution

The STS3215 reports position as a **12-bit** value — that's **4096 steps** spanning the full 360° range of the output shaft. Each step therefore represents about **0.088°**, which is well beyond what a typical 3D-printed mechanism can hold anyway.

You can drive the servo **past** the normal 0–4095 range in continuous-rotation mode, making it behave like a geared DC motor with feedback — handy for wheels and turrets.

---

## Closed-Loop Feedback

This is the feature that genuinely changes what you can build. After commanding a position, you can immediately read `Present position`, `Present speed`, `Present load`, `Present voltage` and `Present temperature` back from the servo.

Some things this unlocks:

* **Motion sequencing** — wait until the servo has actually arrived before sending the next command.
* **Load detection** — notice when an arm has picked something up (or bumped into something).
* **Teach-by-demonstration** — disable torque, move the arm by hand, record the positions and play them back later.
* **Thermal protection** — back off before the servo overheats.

None of this is possible with a PWM servo, because a PWM servo has no way to talk back.

---

## Powering a Bus Servo

The STS3215 is rated for **6–8.4 V** (the "7.4 V class"), so the natural power source is a **2S LiPo** or a **7.4 V bench supply**. Higher voltages within the rated band generally give higher torque and speed.

A few important rules:

* **Never** power the servo from a microcontroller's 5 V or 3.3 V rail. Stall currents can run into several amps and will instantly destroy the regulator.
* **Share a common ground** between the microcontroller and the servo's power supply, otherwise the UART signals have no reference voltage.
* For multi-servo arms, size the supply for the **sum of stall currents**, not the idle draw — multiple servos stalling at once is a very real scenario during a collision.

---

## Compared to Other Servo Types

| Feature | Hobby PWM Servo | Serial Bus Servo (STS3215) | Smart Servo (Dynamixel) |
|---|---|---|---|
| Wiring | One signal wire each | Single daisy-chain bus | Single daisy-chain bus |
| Feedback | None | Position, speed, load, voltage, temperature | Position, speed, load, voltage, temperature |
| Resolution | Analogue pulse | 12-bit (0–4095) | 12–14-bit depending on model |
| Price (approx.) | £1–£5 | £10–£14 | £40–£200+ |
| Protocol | PWM | Half-duplex UART, 1 Mbaud | Half-duplex UART or RS-485 |
| Ecosystem | Massive | Growing | Well-established, industrial |
{:class="table table-single table-narrow"}

The STS3215 effectively sits in the sweet spot: **Dynamixel-grade features at hobby-grade prices**.

---

## Where Bus Servos Shine

* **Robot arms** — especially anything with more than three joints, where PWM wiring becomes a mess.
* **Hexapods and quadrupeds** — 12+ servos wired on a single data line rather than 12+ GPIOs.
* **Animatronics** — smooth coordinated motion with position feedback for safety.
* **Teaching rigs** — torque-off + read-position makes teach-by-demonstration almost free.

---

## Where a PWM Servo is Still the Right Choice

Bus servos aren't always the answer:

* **Single-servo projects** where the cost premium isn't justified.
* **Very small spaces** where a micro servo like the SG90 is the only thing that fits.
* **Ultra-fast RC applications** where hobby servos are optimised for minimum latency from dedicated receivers.

---

## Related Reading

* [Servos — How they work](servos.html)
* [I²C — another embedded bus protocol](i2c.html)
* [Shift Registers — multiplexing GPIO with serial data](shift-registers.html)

---
