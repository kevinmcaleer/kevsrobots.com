---
layout: lesson
title: Overview
author: Kevin McAleer
type: page
cover: /learn/eye_mechanism/assets/banners/00_intro.jpg
date: 2023-01-29
next: 01_create_component.html
description: Design a robot eye mechanism with moving eyelids in Fusion 360
percent: 10
duration: 3
quiz_total: 27
date_updated: 2026-11-24
navigation:
- name: Robot Eye Mechanism
- content:
  - section: Overview
    content:
    - name: Overview
      link: 00_intro.html
  - section: Initial Design
    content:
    - name: Create a Component
      link: 01_create_component.html
    - name: Create a Sketch
      link: 02_create_sketch.html
    - name: Revolve
      link: 03_revolve.html
  - section: Refine the Top Eyelid detail
    content:
    - name: Create Hinge Sketch
      link: 04_create_hinges.html
    - name: Extrude The Hinge
      link: 05_hinge.html
  - section: Create the Lower Eyelid
    content:
    - name: Create the Lower Eyelid Component
      link: 06_lower_eyelid_sketch.html
    - name: Creating the Lower Eyelid
      link: 07_create_lower_sketch.html
    - name: Create Hinge from Profile
      link: 08_hinge.html
  - section: Joints and Export
    content:
    - name: Joints and Export
      link: 09_joint_and_export.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

## Watch the video

This course goes with the video below, which shows every step in action.

{% include youtubeplayer.html id="JbWKsdy5iOk" %}

There is also a newer build of the mechanism in this video:

{% include youtubeplayer.html id="XpKnyTx_1mQ" %}

---

## Overview

Eyes bring a robot to life! This course will teach you how to model an eye mechanism using Autodesk Fusion 360. The mechanism has two eyelids that open and close around a central eyeball. You can 3D print the parts and build them into your own robot.

## Course Content

* Creating components and sketches
* Using the **Revolve** tool to make curved parts like the eyelids
* Offset planes, the **Slice** view and sketch constraints
* Building hinges with **Extrude** join and cut operations
* Saving time with **Mirror**
* Joining the eyelids with a **revolute joint** and exporting the parts to print

## Key Results

By the end of this course you will be able to:

* Model curved parts in Fusion 360 by revolving a profile
* Create hinges that let two 3D-printed parts pivot around a screw
* Organise a design into components and connect them with joints
* Export each part so it can be 3D printed

## What you'll need

* Autodesk Fusion 360 [^1] installed on your computer
* A mouse with a scroll wheel makes moving around the model much easier

[^1]: In this course you will be using the features of Autodesk Fusion 360; there is a free educational and hobbyist licence available enabling you to use the software free of charge.

## How the course works

Each lesson is a list of steps with a screenshot of what you should see. Keyboard shortcuts are shown like `this`. Use the video if you get stuck. Every lesson starts with what you will be able to do by the end, and finishes with exercises, common problems and a short quiz so you can check you have it before moving on. The course has no code to run: it is all modelling in Fusion 360, finishing with STL files you can print.

> **Teachers**: This course suits learners aged about 12 and up who are comfortable with a mouse and keyboard, and works well in pairs. Each lesson takes about 20 to 40 minutes, so the whole course fits in four to six sessions. Each learner or pair needs a computer with Fusion 360 and an Autodesk account with the free educational licence, so set these up beforehand and check that your school network allows Fusion 360 to sign in. The 3D printing at the end needs a printer, filament and a small screw for the hinge pin, but it is optional: a class with no printer can finish at the joint and test the movement on screen. Printers are hot, so supervise them, and keep small screws counted and out of reach of younger learners.
{:class="bg-blue"}

## Prerequisites

None, but if you've never used Fusion 360 before, [Design a SMARS Robot in Fusion 360](/learn/smars_fusion360/) is a gentle place to start.
