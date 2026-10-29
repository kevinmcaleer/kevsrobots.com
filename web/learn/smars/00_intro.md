---
layout: lesson
title: Introduction
author: Kevin McAleer
type: page
cover: /learn/smars/assets/how_to_build_a_smars.jpg
date: 2023-02-19
next: smars01.html
description: Learn how to download, print, assemble, and program the original SMARS
  robot
percent: 6
duration: 2
quiz_total: 8
navigation:
- name: Build a SMARS Robot
- content:
  - section: Overview
    content:
    - name: Introduction
      link: 00_intro.html
  - section: Print Parts
    content:
    - name: Print the Parts
      link: smars01.html
  - section: Assemble Robot
    content:
    - name: Solder wires to motors
      link: smars02.html
    - name: Fit the Motors
      link: smars03.html
    - name: Fit the Battery
      link: smars04.html
    - name: Fit Motors Holders
      link: smars05.html
    - name: Attach Wheels
      link: smars06.html
    - name: Test Motors
      link: smars07.html
  - section: Wiring up the Robot
    content:
    - name: Add the Arduino
      link: smars08.html
    - name: Add Motor Shield
      link: smars09.html
    - name: Insert Wires
      link: smars10.html
    - name: Attach Tracks
      link: smars11.html
  - section: Code
    content:
    - name: Load the code
      link: smars12.html
    - name: Arduino Code
      link: smars13.html
  - section: Summary
    content:
    - name: Summary and Review
      link: summary.html
---


![SMARS Course image]({{page.cover}}){:class="cover"}

## Watch the video

New to SMARS? This video shows what makes it such a great first robot. Each lesson in this course also has its own video from the build series.

{% include youtubeplayer.html id="XhArIe9UDoc" %}

---

## Overview

Welcome to the **Build a SMARS Robot** course. SMARS (Screwless Modular Assemblable Robotic System) is a small 3D-printed tracked robot that snaps together without screws. This course takes you from printing the parts to driving it around. Unlike most of the other robots on this site, SMARS uses an `Arduino Uno`, and the code is written in `C++`.

## Course Content

In this course you will learn:

* Where to download the files and what to print
* How to solder the motors and assemble the robot
* How to wire up the motor shield and power the robot
* How to load code onto the Arduino and control your SMARS

---

## Key Results

After you have completed this course you will have built your own SMARS robot, wired it up and powered it, and loaded code that lets you drive it by remote control.

---

## What you'll need

To take this course you will need:

* Access to a 3D printer (or a friend or library with one)
* An `Arduino Uno` (or a clone) and a `Fundumoto` motor shield
* Two N20 geared motors and a 9V battery with a battery clip
* A soldering iron, solder and some red and black wire
* A computer with the [Arduino IDE](https://www.arduino.cc/en/software) and a USB cable for the Arduino

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

There's a short quiz at the end of each section so you can check your understanding.

## Prerequisites

None! This is a great first robot. If you're new to robots, [Robotics 101](/learn/robotics_101/) explains the parts of a robot in more detail.
