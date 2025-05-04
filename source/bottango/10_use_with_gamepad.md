---
title: Using Bottango with a Gamepad
description: Learn how to control your animations and servos in real time using a USB gamepad or joystick.
layout: lesson
type: page
cover: assets/bottango_gamepad.png
date_updated: 2025-05-03
---

Want to control your servos live — like a puppet? Bottango supports USB **gamepads and joysticks**, letting you trigger animations or control servo positions in real time. Great for live performances, interactive demos, or fun robot battles!

---

## 🎮 Compatible Devices

Bottango supports most USB gamepads and joysticks, including:

- Xbox controllers
- PlayStation controllers
- Generic USB gamepads
- Flight sticks and arcade sticks

> 🔌 Plug in your controller **before launching Bottango** to ensure it’s recognized.

---

## 🛠️ Step 1: Enable Gamepad Input

1. Open Bottango
2. Go to **Devices > Gamepad Settings**
3. You should see your controller listed
4. Test buttons and joysticks to confirm they register

---

## 🎚️ Step 2: Assign Gamepad Controls

You can assign gamepad inputs to either:

- **Trigger animations** (e.g., press button A to wave)
- **Directly control servo positions** (e.g., move a joystick to control a jaw)

### To trigger an animation:

1. Go to the **Triggers** panel
2. Click **Add Trigger**
3. Choose **Gamepad Button**
4. Select the button (e.g., Button A)
5. Set the action to **Play Animation** and choose your animation

### To control a servo directly:

1. Select your servo in the timeline
2. In the **Servo Settings**, choose **Gamepad Axis** under input method
3. Select the joystick axis (e.g., Left Stick Y)
4. Set the range (min/max) to match your servo's limits

---

## 🧪 Test It Out

- Press your assigned gamepad button — the servo should move or play an animation
- Move a joystick — the servo should follow your movement in real time

> 🎯 You can control multiple servos simultaneously with different axes!

---

## 🧠 Creative Uses

- Real-time puppet shows (control eyes, mouth, head)
- Remote-controlled robots with expressive motion
- Assistive tools (e.g., gamepad-controlled grippers)
- Multiplayer animatronic props or characters

---

## ⚠️ Tips for Smooth Gamepad Control

- Use **easing curves** or filtering for smoother servo movement
- Avoid using the same axis for multiple servos unless needed
- Keep servo limits in mind to avoid jitter or strain

---

With a gamepad in hand, your project becomes a **live, expressive performer**.

Next up: [Course Summary and Next Steps](11_summary.md)

---
