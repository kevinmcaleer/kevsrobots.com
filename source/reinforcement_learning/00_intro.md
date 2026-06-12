---
title: Introduction to Reinforcement Learning for Beginners
description: Discover how to teach a Pico-powered robot to learn obstacle avoidance through trial and error — no machine-learning PhD required.
layout: lesson
type: page
cover: assets/cover.jpg
date_updated: 2026-06-13
---

![Cover](assets/cover.jpg){:class="cover"}

---

Ahoy there makers! What if your robot could figure things out on its own?

Most robots we build follow instructions we wrote: "go forward, if distance < 20 cm then turn right". That works brilliantly when we know exactly what the environment looks like. But real floors are messy — furniture shifts, a dog wanders through, someone leaves a bag in the hallway. Hardcoded routines break down fast.

Reinforcement Learning (RL) is a completely different approach. Instead of telling the robot what to do, we set up a **reward signal** and let the robot work out the best strategy through **trial and error**. The same idea that trains game-playing AIs to beat world champions also works beautifully at the maker scale — using nothing more exotic than a Python dictionary.

In this course you'll build a differential-drive robot (BurgerBot-style: two DC motors and an HC-SR04 ultrasonic sensor on a Raspberry Pi Pico) and teach it to navigate an obstacle field without a single hardcoded rule. You'll train the policy in a pure-Python simulation on your laptop, then drop the learned "brain" onto the Pico as a JSON file.

---

## Overview

This course teaches **tabular Q-learning** — the clearest and most approachable form of reinforcement learning. You'll understand exactly what happens at every step because there are no black-box layers: just a table of numbers, a handful of Python functions, and a robot that genuinely gets better with practice.

The mathematics is deliberately kept accessible. Any formula that appears is first written as a commented Python expression so you can read it line by line.

---

## Course Content

- Why hardcoded robot behaviours fail in dynamic environments
- The core RL vocabulary: agent, environment, state, action, reward
- How to design a reward function that actually teaches what you want
- Turning continuous sensor readings into discrete states the algorithm can use
- The Markov Decision Process — just enough theory to be useful
- Discount factors and why future rewards matter less than immediate ones
- Building a Q-table from scratch using a Python dictionary
- Exploration vs exploitation and the ε-greedy strategy
- Coding the complete Q-learning update rule
- Training in a grid-world simulation (runs on any laptop, no GPU needed)
- The sim-to-real gap and practical strategies to bridge it
- Reading and filtering HC-SR04 data in MicroPython
- Exporting a trained Q-table and running it on the Pico
- Where tabular Q-learning runs out of road and what comes next (Deep RL)

---

## Key Results

After completing this course, you will:

- Understand the difference between programmed behaviour and learned behaviour
- Be able to design and implement tabular Q-learning for a simple robot task
- Know how to represent a robot's world as states and actions
- Have trained a Q-table in simulation and watched it improve over hundreds of episodes
- Have deployed a frozen policy to a Raspberry Pi Pico running MicroPython
- Understand where to go next when the problem outgrows a lookup table

---

## What You'll Need

**For lessons 1–10 (simulation, runs on your laptop):**

- A computer running Python 3.8 or newer
- No external libraries required — everything uses the Python standard library
- A text editor or IDE (VS Code, Thonny, or similar)

**For lessons 11–13 (Pico deployment):**

- Raspberry Pi Pico (standard or Pico W)
- L298N or similar H-bridge motor driver
- 2x DC gear motors with wheels
- HC-SR04 ultrasonic distance sensor (3.3 V version, or voltage-divided 5 V version)
- Suitable battery pack (4×AA or a LiPo with a regulator)
- Jumper wires and a breadboard or chassis with connectors
- Thonny IDE with MicroPython firmware on the Pico

> If you've already built a robot for the [MicroPython Robotics Projects](/learn/micropython_robotics/) course, that hardware is perfect for this course too.

---

## Prerequisites

This course assumes you're comfortable writing Python functions and using loops and dictionaries. If you'd like a refresher:

- [Introduction to Python](/learn/python/) — covers the Python fundamentals we rely on throughout
- [MicroPython Robotics Projects](/learn/micropython_robotics/) — covers the exact hardware wiring and MicroPython motor/sensor code we build on in the final module

You do **not** need any prior machine-learning experience. We build everything from first principles.

---

## How the Course Works

Each lesson focuses on **one new concept** and connects it to the running project. Code blocks are complete and runnable. Where a mathematical idea appears (like the Q-learning update rule), it is shown first as a commented Python expression — read the comments and the code together and the maths will make sense.

Lessons in the final module include full MicroPython files you can copy directly to your Pico. The simulation lessons (1–10) run with a plain `python3 filename.py` command — no installation steps needed.

Look for these callout styles throughout:

> **Note:** background context or a reminder about something covered earlier.

> **Tip:** a practical suggestion for making things work better.

> **Warning:** something that commonly goes wrong and how to avoid it.

Let's build a robot that can learn!

---
