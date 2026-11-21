---
title: Introduction
description: Meet the Arduino Uno Q, a board with a Linux computer and a microcontroller on it, and see how this course teaches you to make them work together
layout: lesson
type: page
cover: /learn/arduino_uno_q/assets/banners/00_intro.jpg
date_updated: 2026-11-21
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Ahoy there makers! What if one board could run Python, a web server and an AI model, and also keep a servo pulse perfectly steady? The Arduino Uno Q does both, because it has two processors on it. This course shows you how to put each one to work.

Watch this first, then come back and let's build.

{% include youtubeplayer.html id="4HFb-FGQ8_w" %}

---

## Overview

The Uno Q looks like an Arduino Uno, but it is really two computers sharing one board. One is a small Linux computer that runs Debian. The other is a microcontroller that runs Arduino sketches. A built-in link called the **Bridge** lets them call each other's functions.

That is the valuable skill here: deciding which job belongs on which side, then making the two sides talk. You'll learn it by building one project, the **Q Board**, which grows a little in every lesson.

Lessons 10 to 16 are being published now, so the list on the left will grow.

```mermaid
flowchart LR
    A["Meet the board"] --> B["The microcontroller side"]
    B --> C["The Bridge"]
    C --> D["The Linux side"]
    D --> E["Showcase builds"]
```

## Course Content

- **Meet the board**: what the two processors are good at, first boot, Arduino App Lab, Wi-Fi, and your first App
- **The microcontroller side**: drawing on the LED matrix, reading pins and sensors, and moving a servo
- **The Bridge**: how the two sides talk, calling the sketch from Python, calling Python from the sketch, and fixing it when it goes wrong
- **The Linux side**: a Linux survival kit, a web page for your project, ready-made camera and AI Bricks, and running your App at startup
- **Showcase builds**: an always-on AI agent, and a chess computer that plays on a real board

## Key Results

After completing this course, you will:

- Explain what the Linux side and the microcontroller side are each good at
- Set the board up in Arduino App Lab and work on it over Wi-Fi
- Draw pictures and animations on the LED matrix
- Read a potentiometer and a sensor, and move a servo, from a sketch
- Call sketch functions from Python, and Python functions from a sketch, over the Bridge
- Find and fix the common Bridge bugs, including the `provide()` deadlock

## What you'll need

- An Arduino Uno Q, the 2 GB or the 4 GB version
- A USB-C **data** cable
- A computer (Windows, macOS or Linux) with Arduino App Lab installed. Lesson 2 shows you how
- A 10k potentiometer and 3 jumper wires, for lessons 5 and 8
- Optional, for lesson 5: a Modulino Thermo with a Qwiic cable, and a small hobby servo. There is a no-part route for each
- From lesson 12: a USB webcam and a powered USB-C hub

You don't need to know both Python and Arduino C++. A little of either is enough, and we explain the other as we go. If you'd like some background first, these are good companions:

- [Python for beginners](/learn/python/)
- [Arduino to Python](/learn/arduino_to_python/)

## How the course works

**The Q Board grows.** Every lesson adds a stage to the same App, and each one tells you which stage you have reached. By the end you'll have a number display that you drive from Python and read from your phone.

**Two kinds of code.** Every code block is labelled with the file it belongs in. `sketch/sketch.ino` runs on the microcontroller and is C++. `python/main.py` runs on the Linux side and is Python. When you see the label, you know which brain is running it.

**Quizzes.** Each lesson ends with a short quiz. Your score is kept in your browser and shown in the sidebar.

**Interactive boxes.** Some lessons have little tools you can play with in the browser, such as an LED matrix painter and a Bridge step-through. They work even if you don't own the board yet. They are not the real hardware, so each one ends with a step to try on the board.

**Videos.** Where a lesson has a video, watch it first. It makes the written steps easier to follow.

**Mistakes are normal.** Every lesson has a Common Issues list for the things that trip people up. Check it before you assume something is broken.

Ready? Let's meet the two brains.
