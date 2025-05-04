---
layout: lesson
title: Using Triggers and Interactivity
author: Kevin McAleer
type: page
cover: assets/bottango_triggers.png
date: 2025-05-10
previous: 06_create_motion.html
next: 08_sync_music.html
description: Learn how to add interactivity to your animations using triggers in Bottango.
percent: 63
duration: 3
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


So far, you've created animations that play back on their own. But what if you want your servo to react to something — like a button press, sensor input, or keyboard event? That’s where **triggers** come in.

---

## 🔔 What Are Triggers?

Triggers let you **start animations based on events**.

You can trigger an animation using:

- A keyboard key
- A gamepad button
- A GPIO pin (e.g., button press on Arduino)
- Another animation finishing

Triggers are powerful for creating **interactive props, puppets, and characters**.

---

## 🛠️ Step 1: Add a Trigger

1. Go to the **Triggers** panel (usually on the right side).
2. Click **Add Trigger**.
3. Choose a trigger type (e.g., Keyboard, Gamepad, Digital Input).
4. Set the condition — e.g., key `A` is pressed.

---

## 🎬 Step 2: Assign an Action

1. Under **Trigger Action**, choose **Play Animation**.
2. Select the animation or servo motion you want to play.
3. Optional: Set the animation to play once or loop.

Now, when the trigger is activated, your animation will play instantly.

---

## 💡 Example: Press a Key to Wave

Let’s say you’ve created a servo animation called `Wave`.

- Add a **Keyboard trigger**
- Set it to key `W`
- Choose action: Play animation `Wave`

Now every time you press the `W` key, your servo will wave!

---

## 🎮 Gamepad Triggers

Bottango supports USB gamepads. You can:

- Trigger actions using buttons or joystick axes
- Combine triggers with servo control
- Use for remote-controlled puppetry or performances

> 🧪 Connect your gamepad before launching Bottango to ensure it’s recognized.

---

## 🔌 Digital Triggers (GPIO)

If your board supports it, you can trigger actions from a physical input like:

- A pushbutton
- A touch sensor
- A motion detector (PIR sensor)

In the **Trigger Source**, choose a pin (e.g., D7), and define it as HIGH or LOW.

---

## 🧠 Pro Tips

- You can chain triggers together for complex behaviors
- Triggers can also **stop animations**, **switch scenes**, or **send signals** to other boards
- Test your triggers in the editor before final performance

---

Now your project isn’t just moving — it’s responding!

Next up: [Syncing Animations to Music](08_sync_music.md)

---
