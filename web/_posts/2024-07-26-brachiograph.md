---
title: BrachioGraph
description: >-
    Learn how to build and use a BrachioGraph, a simple and fun drawing robot that can be controlled using Python.
excerpt: >-
    A brachiograph is a type of robotic arm designed to mimic the movement of a human arm. It typically consists of a series of joints and links, which can be controlled to perform tasks such as drawing, picking, and placing objects. The term "brachiograph" is derived from the Greek words "brachion" (arm) and "graph" (to write), highlighting its initial use for drawing and writing applications.
layout: showcase
date: 2024-07-26
author: Kevin McAleer
difficulty: beginner
cover: /learn/brachiograph/assets/cover.png
hero: /learn/brachiograph/assets/hero.png
mode: light
tags: 
 - BrachioGraph
 - Robot arm
groups:
 - robots
 - robotarms
videos:
 - tkAv3HIw3CY
 - Tf2oxjDkv9c
 - 7hI-9dHqTeg
---

## Introduction

Are you fascinated by the world of robotics and eager to build your own robotic arm? Look no further! In this blog, we'll explore the brachiograph, a simple yet intriguing robotic arm project that you can build at home. 

---

## What is a Brachiograph?

A `brachiograph` is a type of robotic arm designed to mimic the movement of a human arm. It typically consists of a series of joints and links, which can be controlled to perform tasks such as drawing, picking, and placing objects. The term "brachiograph" is derived from the Greek words "brachion" (arm) and "graph" (to write), highlighting its initial use for drawing and writing applications.

---

## Components of a Brachiograph

![Brachiograph Components](/learn/brachiograph/assets/base.jpg){:class="img-fluid w-100 rounded-3 card-shadow card-hover"}

To build a brachiograph, you'll need the following components:

1. **Servomotors:** These are the muscles of your brachiograph, responsible for moving the joints. SG90 servos are cheap and easy to get hold of (though you do get what you pay for)
2. **Microcontroller:** This acts as the brain, controlling the servomotors based on your inputs. Popular choices include Arduino and Raspberry Pi; for this project we'll use a Raspberry Pi 3+.
3. **Power Supply:** Ensures your servomotors and microcontroller have the necessary power to operate; usually with servos you need extra power, but because this is so small, we can get away with powering the servos from the Pi.
4. **Mechanical Structure:** The frame of the brachiograph, which includes arms, joints, and a base. This can be made from materials like plastic, wood, or metal. 2 Popiscle sticks and some hot glue will do the trick.
5. **Software:** Programs that control the movements of the brachiograph. All the code you need can be downloaded from <https://www.github.com/evildmp/brachiograph>

---

## Building a Brachiograph

Building a brachiograph is a fun and educational project that can be completed in a few hours. Here's a step-by-step guide to help you get started:

<div class="row row-cols-md-3 row-cols-1">
<div class="col">
{% include card.html cardtitle="BrachioGraph Tutorial" link="/learn/brachiograph/" description="Learn how to build your own pen plotter" img="/learn/brachiograph/assets/cover.png" %}
</div>
</div>

---
