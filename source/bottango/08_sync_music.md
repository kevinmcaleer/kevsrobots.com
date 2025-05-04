---
title: Syncing Animations to Music
description: Learn how to sync your servo movements to music and audio tracks in Bottango.
layout: lesson
type: page
cover: assets/bottango_sync_music.png
date_updated: 2025-05-03
---

One of Bottango’s most powerful features is the ability to **synchronize your animations with music or sound effects**. This is perfect for building animatronics that lip-sync, dance, or react to music in time.

---

## 🎵 Step 1: Add an Audio Track

1. Open your Bottango project.
2. In the timeline view, click **Add Audio**.
3. Choose a `.mp3` or `.wav` file from your computer.
4. The audio waveform will appear in the timeline.

> 📝 Tip: Use short, high-quality audio clips for best results.

---

## 🕒 Step 2: Align Keyframes with the Beat

1. Play the audio using the timeline preview.
2. Use the waveform as a visual guide to **place keyframes** at musical beats or lyric changes.
3. You can **zoom in** on the timeline for precise placement.

This makes it easy to time head nods, arm waves, or jaw movements with music.

---

## 🗝️ Step 3: Lip-Syncing Basics

To create a simple lip-sync effect:

- Assign a servo to control the jaw or mouth
- Create quick up/down movements at syllables or beats
- Use easing curves to make the motion look more natural

For more realism, try animating multiple mouth positions (visemes) using multiple servos.

---

## 🎛️ Optional: Use Audio Triggers

If you want animations to start **when audio begins**, create a trigger:

1. Go to the **Triggers** panel.
2. Choose **Audio Starts** as the source.
3. Set the action to **Play Animation** or **Start Timeline**.

You can also sync lighting, gamepad actions, or other animations using audio events.

---

## 🔊 Output Options

- Bottango plays audio **on your computer**, not on the microcontroller.
- If you’re using a Bluetooth speaker or need tight timing, test playback latency.
- For installations or stage performances, consider routing audio and animation output through a synced show controller.

---

## 🎭 Example: Dancing Robot

1. Load a short music clip (e.g. 10–30 seconds).
2. Add servo keyframes to match the beat.
3. Add dramatic motions for chorus or breaks.
4. Use **loop playback** to rehearse and refine.

The result? A robot or puppet that dances in time with the music — great for stage props, Halloween decorations, or interactive art.

---

You’ve now learned how to animate with rhythm and sound — awesome work!

Next up: [Advanced Arduino Mode](09_arduino_mode.md)

---
