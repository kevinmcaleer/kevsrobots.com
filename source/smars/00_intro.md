---
layout: lesson
title: Introduction
type: page
description: Learn how to download, print, assemble, and program the original SMARS robot
cover: /learn/smars/assets/banners/00_intro.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

## Watch the video

New to SMARS? This video shows what makes it such a great first robot. Each lesson in this course also has its own video from the build series.

{% include youtubeplayer.html id="XhArIe9UDoc" %}

---

## Overview

Ahoy there makers! Welcome to the **Build a SMARS Robot** course. SMARS (Screwless Modular Assemblable Robotic System) is a small 3D-printed tracked robot that snaps together without screws. This course takes you from printing the parts to driving it around from your phone.

Unlike most of the other robots on this site, SMARS uses an `Arduino Uno`, and the code is written in `C++`. You will print the parts, solder two motors, wire up a motor shield, and upload a sketch. You will finish with a robot that rolls when you tell it to.

```mermaid
flowchart LR
    A["Print the parts"] --> B["Solder the motors"]
    B --> C["Assemble the chassis"]
    C --> D["Wire the shield"]
    D --> E["Upload the code"]
    E --> F["Drive it by Bluetooth"]
```

## Course Content

In this course you will learn:

* Where to download the files and what to print
* How to solder the motors and assemble the robot
* How to wire up the motor shield and power the robot
* How to load code onto the Arduino and control your SMARS

---

## Key Results

After you have completed this course you will:

* Have built your own SMARS robot, wired it up and powered it
* Know how to solder wires to a small motor and test it
* Understand how the Arduino, motor shield, battery and motors connect
* Be able to upload a sketch and drive the robot with Bluetooth commands

---

## What you'll need

To take this course you will need:

* Access to a 3D printer (or a friend or library with one)
* An `Arduino Uno` (or a clone) and a `Fundumoto` motor shield
* Two N20 geared motors and a 9V battery with a battery clip
* A soldering iron, solder and some red and black wire
* A computer with the [Arduino IDE](https://www.arduino.cc/en/software) and a USB cable for the Arduino
* A Bluetooth serial module such as an HC-05, for the last lesson

The full parts list is in the next lesson.

---

## How the course works

New keywords are highlighted inline like `this`, and code examples look like this:

```cpp
digitalWrite(12, HIGH);
```

Important notes are highlighted like this:

> ## This is a note
>
> Important notes are highlighted like this

Most lessons are one build step, with a photo to compare against. Every lesson (after this one) ends with **Try it Yourself** exercises, a **Common Issues** list for when things go wrong, and a short quiz so you can check your understanding.

> **Teachers**: This course suits ages 11 and up, with adult supervision for soldering. Allow about 20 to 30 minutes per lesson, plus print time (the parts take several hours on a typical printer, so print them in advance). Each learner or pair needs one robot kit: an Arduino Uno, a Fundumoto shield, two motors and a battery. Soldering needs an iron per bench and good ventilation, so plan it as a supervised session. Lessons 1 and 13 can be done with no hardware: reading the parts list and tracing through the sketch.

## Prerequisites

None! This is a great first robot. If you're new to robots, [Robotics 101](/learn/robotics_101/) explains the parts of a robot in more detail.
