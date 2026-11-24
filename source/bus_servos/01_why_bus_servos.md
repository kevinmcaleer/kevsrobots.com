---
title: Why bus servos
description: See what a bus servo is, how one wire carries commands to many servos, and why position feedback changes what a robot can do
layout: lesson
type: page
cover: /learn/bus_servos/assets/banners/01_why_bus_servos.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Picture a six-joint robot arm built from hobby servos. Six signal wires run back to six Pico pins, along with power for each one, all flexing every time the arm moves. And when you command a joint to 90 degrees, you just have to trust that it got there. Bus servos fix both problems.

---

## A quick recap of hobby servos

In [Motors and motor drivers](/learn/motors_and_drivers/) you drove SG90-style servos with a PWM pulse: the pulse width says where to go, the servo moves, and that is the whole conversation. Each servo needs its own signal pin, and the servo never reports back. Lessons 10 to 12 of that course cover the pulse, a servo class and calibration, so we won't repeat them here.

Two limits show up the moment you build an arm:

1. **Every servo needs its own signal wire** back to the microcontroller. Six servos means six pins.
2. **The servo can't tell you where it actually is.** You send a target and hope.

## What a bus servo does differently

The Feetech **STS3215** talks **serial** instead of PWM. Commands travel as small data packets over **half-duplex UART**, which means one wire carries traffic in both directions, taking turns.

Because the packets carry an address, many servos can share the same cable. Each servo has a unique **ID**, and a packet says "servo 3, go to this position". Every servo hears it, and only servo 3 acts.

```mermaid
flowchart LR
    P["Pico"] -->|"one data line"| D["Driver board"]
    D --> S1["Servo ID 1"]
    S1 --> S2["Servo ID 2"]
    S2 --> S3["Servo ID 3"]
```

The two-way conversation is the clever bit. You can ask a servo for its position, speed and temperature, and it answers. The servo becomes a **sensor as well as an actuator**.

| Feature | PWM servo | STS3215 bus servo |
| --- | --- | --- |
| Signal | Pulse width | Serial packets (half-duplex UART) |
| Wiring | One signal pin per servo | One bus, servos chained |
| Position feedback | None | Yes, 12-bit (0 to 4095) |
| Range | About 180 degrees | Full 360 degrees |
| Extra data | None | Speed, temperature and more |
| Typical cost | About 1 to 3 pounds | About 10 to 14 pounds |
{:class="table table-single"}

Yes, they cost around ten times as much as an SG90. Kev's verdict: you get what you pay for, and the feedback alone makes the difference.

## Peek inside a packet

Every command is a short run of bytes. The library builds them for you, but it helps to see one. Every packet starts with two `0xFF` bytes, then the servo ID, a length, an instruction and its parameters, and ends with a checksum. This program builds the packet that asks servo 1 "are you there?" and prints it. It uses only plain Python, so it runs on your Pico without any servo attached.

```python
# Build the "ping" packet for a servo, the same way the STS3215 library does
PING = 0x01

def build_packet(servo_id, instruction, params=None):
    if params is None:
        params = []
    length = len(params) + 2            # params + instruction + checksum
    packet = [0xFF, 0xFF, servo_id, length, instruction] + params
    checksum = (~sum(packet[2:])) & 0xFF   # invert the sum of ID, length, ...
    packet.append(checksum)
    return bytes(packet)

packet = build_packet(1, PING)
print("Ping servo 1:", " ".join("{:02X}".format(b) for b in packet))
print("Ping servo 5:", " ".join("{:02X}".format(b) for b in build_packet(5, PING)))
```

You should see `FF FF 01 02 01 FB` for servo 1. The checksum adds up everything after the two `FF` header bytes, flips every bit, and keeps the low byte. If a byte is corrupted on the wire, the checksum no longer matches and the servo ignores the packet.

**Why this matters:** when something goes wrong later, such as garbled data or a silent servo, you'll know there is a real protocol underneath, and a real reason it can fail.

**Common mistakes:**

- Thinking a bus servo is "just a digital servo". The ID and the two-way conversation are the whole point.
- Expecting to wire every servo to a different pin. They all share one bus.

## Try it Yourself

1. Change `build_packet(5, PING)` to servo 12. Which bytes change, and why does the checksum change with them?
2. Work out the checksum for servo 1 by hand: add the ID (1), the length (2) and the instruction (1), then flip the bits of the total.
3. Challenge: the library's `move()` docstring says servo IDs run from 0 to 253. How many different servos could that address on one bus?

## Common Issues

**Problem**: The program prints bytes but nothing moves.
**Solution**: Right, this program only builds a packet. It doesn't send one. We send packets from lesson 3.
**Why**: Building bytes and writing them to a UART are separate steps. Seeing them apart makes debugging easier.

---

## Check your understanding

{% quiz %}
? How does a bus servo know a command is meant for it?
- [ ] It listens on its own signal pin
- [x] Each packet carries a servo ID and only the matching servo acts
- [ ] It measures the pulse width
> All the servos share one data line. The ID in the packet decides who acts.

? What can you do with a bus servo that you can't with a basic PWM servo?
- [ ] Power it from a USB port
- [x] Ask it where it is and read the answer back
- [ ] Use it without a power supply
> The two-way conversation lets the servo report position, speed and temperature.
{% endquiz %}
