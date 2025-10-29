---
title: Configuring Bottango
description: Learn how to configure your project, assign servos, and get ready to animate.
layout: lesson
type: page
cover: assets/bottango_configure.png
date_updated: 2025-05-03
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

You're now fully set up to animate!

---

## Common Issues

**Problem**: Servo added but doesn't appear in timeline

**Solution**: Check that you've saved the servo settings and that the project window is maximized. Try refreshing the timeline view

**Why**: Sometimes the UI needs to refresh to show newly added servos

**Problem**: Can't select the correct pin in dropdown

**Solution**: Ensure your board is connected and recognized by Bottango. Go to Devices > Board Selection to verify connection

**Why**: Bottango only shows available pins for connected boards

**Problem**: Servo moves when slider changes but not during timeline playback

**Solution**: Make sure you've added keyframes to the timeline. Manual control works differently from animated playback

**Why**: Timeline playback requires keyframes - the servo follows programmed positions, not the slider

**Problem**: Servo calibration is off (90° position isn't centered)

**Solution**: Adjust the PWM values in advanced servo settings:
- Typical range: Min PWM = 544µs, Max PWM = 2400µs
- Fine-tune these values until 90° is physically centered

**Why**: Different servo brands use slightly different PWM timing

**Problem**: Multiple servos assigned to same pin by mistake

**Solution**: Edit each servo's settings and ensure each has a unique pin assignment. Bottango will warn you but won't prevent it

**Why**: Two servos on one pin will both receive the same signal and move together

**Problem**: Project won't save or keeps crashing

**Solution**:
1. Save to a location without special characters in path
2. Ensure you have write permissions to the folder
3. Check disk space

**Why**: File system issues can prevent Bottango from writing project files

**Problem**: Servo names disappeared or reset

**Solution**: Always save your project after making changes (Ctrl+S / Cmd+S). Bottango auto-saves but it's good practice

**Why**: Unsaved changes are lost if Bottango closes unexpectedly

---

Next up: [Creating Motion Sequences](05_create_motion.md)

---
