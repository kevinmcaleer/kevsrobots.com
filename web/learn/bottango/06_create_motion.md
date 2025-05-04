---
layout: lesson
title: Creating Motion Sequences
author: Kevin McAleer
type: page
cover: assets/bottango_motion.png
date: 2025-05-10
previous: 05_configure_bottango.html
next: 07_triggers.html
description: "Learn how to animate your servos using Bottango\u2019s timeline and\
  \ keyframes."
percent: 54
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


Now that your servos are configured, let’s bring your project to life by creating motion sequences using Bottango’s timeline editor.

---

## 🎬 Step 1: Understanding the Timeline

The Bottango timeline is similar to animation software:

- **Time is shown horizontally** from left to right.
- **Each servo gets its own track** in the timeline.
- **Keyframes** define servo positions at specific moments.

You can create smooth, lifelike motion by placing multiple keyframes along the timeline.

---

## ➕ Step 2: Add Your First Keyframe

1. Select a servo from the left panel.
2. Move the timeline cursor to the 1-second mark.
3. Set the desired servo position using the **slider**.
4. Click **Add Keyframe** (or press `K`).

A keyframe will appear on the timeline — this marks the servo’s position at that point in time.

---

## 🌀 Step 3: Add More Keyframes

Move the timeline cursor forward, adjust the servo’s position, and add more keyframes. For example:

- 0s → 90°  
- 1s → 0°  
- 2s → 90°  

This will create a simple back-and-forth movement.

> ⏯️ Click the **Play** button to preview the animation.

---

## 🧰 Step 4: Fine-Tune the Motion

- **Drag keyframes** left or right to change timing.
- **Right-click a keyframe** to delete or change easing.
- Choose between **linear**, **ease-in**, **ease-out**, or **custom easing curves** for smoother transitions.

---

## 🧠 Best Practices for Animations

- Start with slow, wide movements — fine-tune later.
- Use easing to create more natural, lifelike motion.
- Keep an eye on your servo’s physical limits to avoid stalling or jitter.

---

## 🔄 Looping & Playback

You can loop the animation by enabling the **Loop Playback** option.

To play the full sequence:

- Click **Play** or press `Space`
- Watch your servo(s) follow the animation in real time!

---

## 🎉 You're Animating!

At this point, your project is moving based on the keyframes you created — no code required. Bottango takes care of the hard work.

---

Next up: [Using Triggers and Interactivity](07_triggers.md)

---
