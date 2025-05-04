---
layout: lesson
title: Configuring Bottango
author: Kevin McAleer
type: page
cover: assets/bottango_configure.png
date: 2025-05-10
previous: 04_wiring.html
next: 06_create_motion.html
description: Learn how to configure your project, assign servos, and get ready to
  animate.
percent: 45
duration: 2
date_updated: 2025-05-03
navigation:
- name: Bottango Basics
- content:
  - section: Introduction
    content:
    - name: Introduction to Bottango
      link: 01_intro.html
  - section: Getting Started
    content:
    - name: Installing Bottango
      link: 02_install.html
    - name: Setting Up Your Board
      link: 03_setup_board.html
  - section: Wiring and Configuration
    content:
    - name: Wiring and Connecting Servos
      link: 04_wiring.html
    - name: Configuring Bottango
      link: 05_configure_bottango.html
  - section: Creating Animations
    content:
    - name: Creating Motion Sequences
      link: 06_create_motion.html
    - name: Using Triggers and Interactivity
      link: 07_triggers.html
    - name: Syncing Animations to Music
      link: 08_sync_music.html
  - section: Advanced Projects
    content:
    - name: Advanced Arduino Mode
      link: 09_arduino_mode.html
    - name: Using Bottango with a Gamepad
      link: 10_use_with_gamepad.html
  - section: Summary
    content:
    - name: Course Summary and Next Steps
      link: 11_summary.html
---


Now that your servos are wired up and your board is connected, it’s time to configure Bottango so you can start animating.

---

## 🗂️ Step 1: Create a New Project

1. Open Bottango.
2. Click **New Project**.
3. Give your project a name (e.g., "Servo Test Rig").
4. Choose a save location on your computer.
5. Click **Create**.

> 💾 Bottango projects are stored as `.btproj` files and contain all your servo assignments, motion data, and triggers.

---

## 🎚️ Step 2: Add a Servo to the Scene

1. In the main timeline view, click **Add Servo**.
2. A new servo object will appear in the left-hand panel.
3. Double-click the servo name to rename it (e.g., "Head", "Arm", "Wing").

---

## 📌 Step 3: Assign a Pin to the Servo

1. Select the servo you just added.
2. In the **Servo Settings** panel (bottom left), choose the correct **Board**.
3. Select the **Pin** that the servo is connected to (e.g., D3 or GPIO14).
4. Set the **Min** and **Max** angles (e.g., 0–180).
5. Click **Save**.

> 🔄 You can test the servo by dragging the position slider — it should move in real time!

---

## 🔄 Step 4: Add More Servos (Optional)

Repeat the steps above for each additional servo in your setup. You can:

- Name them descriptively (e.g., "Left Arm", "Tail", "Jaw")
- Assign each one to a unique pin
- Set motion ranges specific to your project

---

## 🧪 Test All Servos

Use the sliders or create a simple timeline keyframe to test each servo's range of motion. This helps:

- Identify reversed servos
- Prevent mechanical overextension
- Verify proper wiring and power

---

## 🗒️ Servo Configuration Tips

- If the servo moves the *wrong direction*, you can invert it in the settings.
- Avoid using pins reserved for USB/Serial on some boards (e.g., D0/D1 on Arduino).
- Bottango supports up to 6 servos per board, depending on performance.

---

You’re now fully set up to animate!

Next up: [Creating Motion Sequences](06_create_motion.md)

---
