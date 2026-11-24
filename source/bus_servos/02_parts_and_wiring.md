---
title: Parts and wiring
description: Gather the parts, wire a Pico to the Waveshare bus servo driver board, set the jumpers and run a smoke test before you move anything
layout: lesson
type: page
cover: /learn/bus_servos/assets/banners/02_parts_and_wiring.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Most bus servo problems come down to wiring, so let's get this lesson right and the rest of the course gets easy. There are only a few connections, and one rule that matters more than the others: servos never get their power from the Pico.

---

## The parts

| Part | What it does | About |
| --- | --- | --- |
| Raspberry Pi Pico | The microcontroller. A Pico W later, for the web panel | 4 pounds |
| Waveshare bus servo driver board | Turns the Pico's UART into the single half-duplex servo line, and feeds power to the bus | 10 pounds |
| Feetech STS3215 servo (two or more) | The star of the show | 10 pounds each |
| 7.4V supply or 2S LiPo | Powers the servo bus | Varies |
| Jumper wires | Three wires: TX, RX and ground | A few pence |
{:class="table table-single"}

Kev bought his STS3215s on AliExpress for about 10 dollars each, which is a lot cheaper than most other places.

## Why the driver board?

The servo bus is **half-duplex**: one wire, both directions. A Pico UART has separate TX and RX pins, so something has to combine them. The Waveshare board does that, handles the voltage for the servo bus, and gives you two servo connectors. It makes life much easier.

## Wiring it up

Do this with everything unpowered.

1. Connect the driver board to your **7.4V supply**. The STS3215 runs at 7.4V, so a 2S LiPo or bench supply is ideal.
2. Wire the Pico to the board:

| Pico pin | Wire to | Why |
| --- | --- | --- |
| GP0 (UART0 TX) | Board RX | The Pico transmits, the board receives |
| GP1 (UART0 RX) | Board TX | The board transmits, the Pico receives |
| GND | Board GND | A shared ground, which is essential |
{:class="table table-single"}

3. Set both yellow jumpers on the board to **A**, labelled "UART to servo". This sends the Pico's UART to the servo bus. Position **B**, "USB to servo", connects the board's own USB-to-serial chip to the bus instead. That's handy if you ever want to drive the servos from a PC, but it's the wrong setting for a Pico.
4. Plug the first servo into either connector on the board.
5. Plug the Pico in over USB.

```mermaid
flowchart LR
    PSU["7.4V supply"] --> BD["Waveshare board"]
    PICO["Pico GP0 / GP1 / GND"] <--> BD
    BD --> SV["STS3215 servo"]
```

> **Never power servos from the Pico.** An STS3215 can pull a lot of current under load. Let the supply or battery do the heavy lifting, and the Pico stays on USB.

**Why the shared ground matters:** without a common ground the UART signals have no common reference, and you get garbled data or silence.

## The smoke test

Before any movement, check the conversation works. First get the library onto your Pico. Download the repo from [github.com/kevinmcaleer/sts3215](https://github.com/kevinmcaleer/sts3215), then copy the driver across:

```bash
mpremote cp sts3215.py :
```

In Thonny, open `sts3215.py` and choose File, Save as, then pick the Raspberry Pi Pico. Now run this program. A new servo is on ID 1.

```python
from sts3215 import STS3215   # copy sts3215.py from the repo onto the Pico first

SERVO_ID = 1

# UART0 on GP0 (TX) and GP1 (RX). 1 Mbaud is the STS3215 default.
servo = STS3215(uart_id=0, tx_pin=0, rx_pin=1, baudrate=1_000_000)

if servo.ping(SERVO_ID):
    print("Servo", SERVO_ID, "answered")
    print("Model number register:", servo.read_model(SERVO_ID))
else:
    print("No answer from servo", SERVO_ID)
    print("Check: power, TX and RX the right way round, shared GND, jumpers on A")
```

If it answers, you're done with wiring. If not, work through the checks it prints.

> **Heads up:** the STS3215 talks at **1,000,000 baud**, not the usual 9600 or 115200. The library sets that for you.

## Try it Yourself

1. Swap the TX and RX wires (power off first), run the program and see what it says. Then put them back.
2. Unplug the 7.4V supply and run it again. Does the Pico still answer?
3. Challenge: what is different about the failure in 1 and the failure in 2? Hint: neither is a Python error, just silence.

## Common Issues

**Problem**: `No answer from servo 1`.
**Solution**: Check that TX goes to RX and RX to TX, that both jumpers are on A, and that the 7.4V supply is on.
**Why**: A swapped pair, a wrong jumper, or no servo power all look identical from the Pico: no reply.

**Problem**: Garbled data or random disconnects.
**Solution**: Make sure the Pico GND and the board GND are connected.
**Why**: Without a shared ground the UART levels have no common reference.

**Problem**: The servo twitches or stalls later on.
**Solution**: Use a supply that can deliver enough current. A bench supply is much happier than a battery on its last legs.
**Why**: A moving servo draws far more current than an idle one.
