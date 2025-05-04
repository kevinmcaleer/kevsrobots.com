---
title: Setting Up Your Board
description: Connect and configure your Arduino or ESP32 board so it can talk to Bottango.
layout: lesson
type: page
cover: assets/bottango_setup_board.png
date_updated: 2025-05-03
---

![Cover](assets/intro.png){:class="cover"}

---

To make Bottango control servos or other actuators, you need to connect a compatible microcontroller board and install the Bottango firmware.

---

## ✅ Supported Boards

Bottango currently supports:

- **Arduino Uno**
- **Arduino Nano (ATmega328 version)**
- **ESP32 (Dev Module or equivalent)**

These boards connect to your computer via USB and communicate directly with the Bottango app.

---

## 🔌 Step 1: Connect Your Board

Use a USB cable to connect your microcontroller to your computer.

Make sure:

- The board is powered via USB
- You’re using a **data-capable USB cable** (some charging cables won’t work)

---

## ⚙️ Step 2: Upload the Bottango Firmware

Bottango needs to upload a small firmware sketch to your board. This allows the app to control it in real-time.

### Automatically via Bottango:

1. Open Bottango.
2. Go to **Devices > Add Device**.
3. Select your board type from the list.
4. Click **Install Firmware**.
5. Bottango will detect the correct COM port and upload the firmware automatically.

> 💡 If the board isn’t found, check your cable and USB port, then restart Bottango.

### Manually (Advanced Option):

If automatic setup fails, you can upload the firmware using the Arduino IDE:

1. Download the Bottango firmware from the [official GitHub repo](https://github.com/bottango/bottango-firmware).
2. Open it in the Arduino IDE.
3. Select the correct board and port under **Tools**.
4. Upload the sketch.

---

## 🟢 Step 3: Verify Connection

Once the firmware is installed:

- The board should appear in Bottango’s device list
- A green indicator shows it’s connected and ready
- You can now assign servos and create motions!

---

## 🧪 Troubleshooting

- **Board not detected?** Try a different USB cable or port.
- **Upload fails?** Make sure no other software is using the COM port (e.g. Arduino IDE).
- **ESP32 issues?** Hold the **BOOT** button when clicking “Upload” if needed.

---

Next up: [Wiring and Connecting Servos](04_wiring.md)

---
