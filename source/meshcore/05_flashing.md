---
title: Flashing the Firmware
description: Use the MeshCore web flasher to load repeater firmware on one board and companion firmware on the other
type: page
layout: lesson
---

This is the moment the two identical boards become two different things. Node 1 gets **repeater** firmware; node 2 gets **companion** firmware. Everything else in the course follows from this choice.

---

## Before you start

- Fit the antennas to **both** boards (see the last lesson - this is not optional)
- Use **Google Chrome** or **Microsoft Edge**. The web flasher uses WebUSB, which Firefox and Safari do not support
- Have your **USB-C data cable** ready
- Label the boards. A strip of masking tape marked "REPEATER" and "CLIENT" will save you a lot of confusion in twenty minutes' time

---

## The firmware variants

At [flasher.meshcore.io](https://flasher.meshcore.io) you will be offered several builds for the Heltec V3. The ones that matter to us:

| Variant | What it is for |
|---|---|
| **Repeater** | Node 1. Forwards packets, no phone attached, full CLI |
| **Companion (BLE)** | Node 2. Pairs to a phone over Bluetooth |
| **Companion (USB Serial)** | Node 2 alternative. Talks to a computer over USB instead |
| **Room Server** | A message store - not used in this course |
| **Sensor** | Telemetry node - not used in this course |
{:class="table table-single"}

**Which companion build?** Choose **Companion (BLE)** if you want to use your phone, which is what most people want. Choose **Companion (USB Serial)** if you plan to drive it from a laptop with the web client at [app.meshcore.nz](https://app.meshcore.nz). You can always reflash later.

---

## Flashing node 1 - the repeater

1. Fit the antenna, then plug the board into your computer with the USB-C data cable
2. Open [flasher.meshcore.io](https://flasher.meshcore.io) in Chrome or Edge
3. Select your board: **Heltec V3** (sometimes listed as *Heltec WiFi LoRa 32 V3*)
4. Select the firmware variant: **Repeater**
5. Select the region preset: **EU/UK (Narrow)** - more on this in the next lesson
6. Click **Flash** and pick your board's serial port from the browser dialog. On Windows it appears as a COM port; on macOS it looks like `/dev/cu.usbserial-xxxx` or `/dev/cu.SLAB_USBtoUART`
7. Wait. The board reboots when it is done

If nothing appears in the port list, jump to the Common Issues at the bottom of this lesson.

---

## Flashing node 2 - the client

Exactly the same process, with one change:

1. Unplug node 1, plug in node 2
2. Same board type: **Heltec V3**
3. Firmware variant: **Companion (BLE)**
4. Same region preset: **EU/UK (Narrow)** - both nodes *must* match
5. Flash

---

## Did it work?

Look at the OLED screen.

- The **repeater** will show its name (something generic like `MeshCore-a1b2` to begin with), plus uptime and packet counters
- The **companion** will show a Bluetooth pairing prompt or its node name, waiting for a phone to connect

If the screen is blank but the board is warm, the flash probably failed part way - just flash it again.

---

## About the serial console

The flasher page has a built in **console** feature. This is how we will configure the repeater in lesson 7, so it is worth finding it now: flash the repeater, then look for the console or terminal option on the same page and connect to the board.

Type `ver` and press enter. If you get a firmware version back, you have a working console and the hard part is over.

There are two other ways to reach the same commands, and you may prefer them:

- [config.meshcore.io](https://config.meshcore.io) - a friendlier web UI for repeater settings
- Any serial terminal at 115200 baud - `screen`, `minicom`, PuTTY, or the Arduino IDE serial monitor

---

## Try it Yourself

1. Flash both boards, then swap the labels around and see if you can tell which is which from the OLED alone.
2. Connect the console to the repeater and run `ver`, `board` and `stats-core`. What is the board reporting for uptime?
3. Challenge: reflash node 2 as **Companion (USB Serial)**, connect it to [app.meshcore.nz](https://app.meshcore.nz), then put it back to BLE. Reflashing is cheap - get comfortable with it.

---

## Common Issues

**Problem**: The browser shows no serial ports at all.

**Solution**: Try a different USB-C cable first, then a different USB port.

**Why**: Charge-only cables have no data lines. This is by far the most common cause.

**Problem**: "WebUSB is not supported" or the Flash button does nothing.

**Solution**: Use Chrome or Edge on a desktop.

**Why**: Firefox, Safari, and mobile browsers do not implement WebUSB.

**Problem**: The flash fails partway through or the board keeps rebooting.

**Solution**: Hold the **BOOT** button (marked PRG on some V3 boards) while plugging the USB cable in, then release it and flash again.

**Why**: This forces the ESP32-S3 into its bootloader instead of running whatever half-written firmware is on it.

**Problem**: I flashed the wrong variant.

**Solution**: Just flash it again with the right one.

**Why**: Nothing is permanent here. The only cost is a minute of your time.

---
