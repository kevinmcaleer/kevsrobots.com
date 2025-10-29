---
title: Creating Motion Sequences
description: Learn how to animate your servos using Bottango’s timeline and keyframes.
layout: lesson
type: page
cover: assets/bottango_motion.png
date_updated: 2025-05-03
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

## Common Issues

- **Problem**: Keyframe added but servo doesn't move during playback
- **Solution**: Check that you've set different positions for each keyframe. If all keyframes are at the same position, there's no motion to display
- **Why**: Keyframes define positions - the servo interpolates between them

- **Problem**: Animation is too fast or too slow
- **Solution**: Drag keyframes further apart (slower) or closer together (faster) on the timeline. You can also adjust playback speed in settings
**Why**: Timeline spacing controls the duration of movement between keyframes

- **Problem**: Servo movement is jerky or robotic
- **Solution**: Add easing curves to your keyframes. Right-click keyframe → choose "ease-in-out" for smoother, more natural motion
- **Why**: Linear interpolation creates constant-speed movement which looks mechanical

- **Problem**: Can't see keyframes on timeline
- **Solution**: Zoom in using the timeline zoom controls (usually mouse wheel or zoom slider). Keyframes might be too close together at current zoom level
- **Why**: Timeline view compresses when showing long animations

- **Problem**: Animation loops but has a "jump" at the end
- **Solution**: Make sure your last keyframe position matches your first keyframe position (e.g., both at 90°)
- **Why**: If start/end positions differ, the servo will snap back to the beginning when looping

- **Problem**: Deleted keyframe by accident
- **Solution**: Use Ctrl+Z (Cmd+Z on Mac) to undo. Bottango supports multiple undo levels
- **Why**: Undo feature helps recover from accidental deletions

- **Problem**: Timeline playback stops in the middle
- **Solution**: Check your timeline length setting. Extend the timeline duration if your keyframes go beyond current timeline end
- **Why**: Playback stops when it reaches the timeline endpoint

---

## Try it Yourself

Ready to practice your new animation skills? Try these exercises:

1. **Wave Motion**: Create a 3-second animation that makes your servo wave back and forth (0° → 180° → 0°)

2. **Easing Experiment**: Create the same motion with three different easing types:
   - Linear easing (constant speed)
   - Ease-in-out (starts slow, speeds up, then slows down)
   - Custom curve (experiment with the curve editor)

   Compare how each one looks and feels!

3. **Multi-Servo Challenge**: If you have 2+ servos, create a coordinated animation where:
   - Servo 1 moves up while Servo 2 moves down
   - They swap positions at the 2-second mark
   - The animation loops smoothly

4. **Realistic Motion**: Create a "breathing" animation:
   - Start at 90° (neutral)
   - Slowly move to 95° over 2 seconds
   - Return to 90° over 2 seconds
   - Use ease-in-out for natural movement
   - Enable looping

**Bonus Challenge**: Can you create a 5-second animation that tells a story? Think of a character nodding "yes" or a robot arm reaching for something!

---

Next up: [Using Triggers and Interactivity](06_triggers.md)

---
