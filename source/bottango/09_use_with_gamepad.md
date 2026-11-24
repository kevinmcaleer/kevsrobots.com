---
title: Using Bottango with a Gamepad
description: Learn how to control your animations and servos in real time using a USB gamepad or joystick.
layout: lesson
type: page
cover: /learn/bottango/assets/banners/09_use_with_gamepad.jpg
date_updated: 2025-05-03
---

![Course Cover Image]({{page.cover}}){:class="cover"}

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

---

## Try it Yourself

Time to take control! Try these gamepad exercises:

1. **Button Triggers**: Set up four buttons to control a servo:
   - Button A → Move servo to 0°
   - Button B → Move servo to 90°
   - Button X → Move servo to 180°
   - Button Y → Play a custom animation you created

   Practice switching between positions quickly!

2. **Joystick Control**: Connect a servo to the left joystick Y-axis:
   - Moving stick up → Servo moves to maximum position
   - Moving stick down → Servo moves to minimum position
   - Center stick → Servo returns to 90° (neutral)

   How smooth can you make the movement?

3. **Multi-Servo Puppet**: If you have 2+ servos, create a simple "puppet":
   - Left stick Y-axis → Controls vertical head movement
   - Left stick X-axis → Controls horizontal head movement
   - Right stick Y-axis → Controls jaw open/close
   - Button A → Trigger a "blink" animation (if you have eye servos)

   Practice "performing" with your puppet!

4. **Animation Playlist**: Create 4 short animations (2-3 seconds each):
   - D-Pad Up → Play "nod yes"
   - D-Pad Down → Play "shake no"
   - D-Pad Left → Play "look left"
   - D-Pad Right → Play "look right"

   Create a sequence by pressing buttons in order!

**Advanced Challenge**: Build a "live performance system":
- Joysticks control real-time servo positions
- Buttons trigger pre-made animations
- One button acts as "home" position (resets all servos)
- Practice a 30-second performance combining both control methods

**Pro Tip**: Invert an axis if your servo moves the "wrong way" when you push the stick!

---

## Common Issues

- **Problem**: Gamepad not detected in Bottango
- **Solution**: Plug in controller before launching Bottango, then restart the application
- **Why**: Bottango scans for controllers on startup

- **Problem**: Servo jitters when controlled by joystick
- **Solution**: Add a small deadzone (5-10%) and enable smoothing/filtering in servo settings
- **Why**: Joysticks have sensitive analog inputs that can cause micro-movements

- **Problem**: Joystick axis controls servo backwards
- **Solution**: Invert the axis in Bottango's gamepad settings or servo configuration
- **Why**: Different controllers map axes differently (positive/negative)

- **Problem**: Multiple button presses don't work smoothly
- **Solution**: Ensure animations are set to "interrupt" previous animations, not "queue"
- **Why**: Queued animations will play in sequence, causing lag

---

Next up: [Course Summary and Next Steps](10_summary.md)

---
