---
title: Advanced Arduino Mode
description: Learn how to export Bottango animations to your Arduino so they can run without a computer.
layout: lesson
type: page
cover: assets/bottango_arduino_mode.png
date_updated: 2025-05-03
---

By default, Bottango requires a live USB connection to your microcontroller. But what if you want your project to run **without a computer**? That’s where **Arduino Mode** comes in.

Arduino Mode allows you to export motion sequences as code and upload them to your board, enabling **completely standalone operation**.

---

## 🧠 How It Works

In Arduino Mode:

- Bottango generates Arduino code that plays your animation(s)
- The code is uploaded like a regular Arduino sketch
- Your board will loop or play the animation based on how you program it

Perfect for costumes, props, or permanent installations!

---

## 🛠️ Step 1: Design Your Animation

1. Create your motion in Bottango as usual
2. Keep it simple — Arduino Mode supports basic motion and triggers
3. Save the project before exporting

---

## 🔄 Step 2: Export for Arduino Mode

1. Go to **File > Export for Arduino Mode**
2. Choose your board type (e.g., Arduino Uno)
3. Bottango will generate a `.ino` file and a `lib/` folder with dependencies

---

## 📦 Step 3: Open in Arduino IDE

1. Open the exported `.ino` file in the [Arduino IDE](https://arduino.cc)
2. Install any required libraries if prompted
3. Select the correct board and port under **Tools**
4. Click **Upload**

Once uploaded, your board will run the motion sequence immediately on power-up.

---

## 🧪 Optional: Add Custom Logic

Since the exported sketch is real Arduino code, you can modify it! For example:

- Add a push button to start the motion
- Trigger animations with sensors (e.g., PIR, ultrasonic)
- Chain multiple sequences using timers or flags

---

## 🧰 Tips for Arduino Mode

- Export each sequence as a separate project if you want to switch between them
- Keep your animation short and loopable for best performance
- Test on USB before switching to standalone mode

---

## 🎯 Example: Halloween Prop

1. Animate a jaw moving up and down in Bottango
2. Export using Arduino Mode
3. Add a motion sensor in the sketch to trigger the animation
4. Power your board with a battery or wall adapter

Now you’ve got a spooky, motion-activated animatronic!

---

Next up: [Using Bottango with a Gamepad](10_use_with_gamepad.md)

---
