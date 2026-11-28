---
title: Design Thinking for Vibe Coders
description: Learn the five design thinking modes and use them with AI coding assistants, so you build the right thing for a real person, then check that it works
layout: lesson
type: page
cover: /learn/design_thinking/assets/banners/00_intro.jpg
date_updated: 2026-11-28
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

{% include youtubeplayer.html id="90pPahgU9JM" %}

In this video I let an AI assistant write the control software for Buddy, my robot arm. The repo has a `CLAUDE.md` file telling the AI what I wanted, and a `tests/` folder so the work could be checked. Knowing what you want, and how you'll check it, is what this course is about.

By the end of this lesson you will be able to say what this course covers, what you need for it, and how the Classroom Plant Helper project runs through every lesson.

---

## Overview

Ahoy there makers! AI assistants such as Claude Code, Copilot, Cursor, ChatGPT and the chat pane in Snakie have made *building* cheap. You can describe a robot, and a few minutes later you have code that runs. The hard part has moved. Now it's deciding *what* to build, and knowing whether it actually works for a real person.

Design thinking is a set of habits for exactly that. In this course we'll learn its five modes, **Empathise, Define, Ideate, Prototype and Test**, and at each step we'll look at how they fit with AI-assisted, "vibe coded" projects: what to hand the AI, what never to hand it, and how to check what comes back.

We'll practise on one project all the way through: the **Classroom Plant Helper**. It starts as a vague brief, "Build a robot that waters the classroom plants", and ends as a tested Raspberry Pi Pico gadget that real pupils can use.

---

## Course Content

- Why vibe coding needs design thinking
- The design thinking process: five modes, three lenses and the Double Diamond
- Empathise: observing and interviewing real people
- Define: point-of-view statements and How Might We questions
- Writing a design brief your AI assistant can use
- Ideate: going wide with Crazy 8s, then choosing
- Prototyping to learn, from cardboard to code
- Vibe coding a prototype safely, step by step
- Prototyping robots with AI on a Pico
- Testing with real people, and testing the code the AI wrote
- Iterating on what you learn
- Project: the Classroom Plant Helper

---

## Key Results

After completing this course, you will:

- Find the real problem behind a brief by watching and talking to the people it's for
- Turn what you learn into a point-of-view statement and a written brief an AI can work from
- Generate lots of ideas, then pick one using clear reasons
- Build cheap prototypes that answer one question each
- Use an AI assistant to write code in small, checkable steps, and spot its common mistakes
- Run a simple usability test and write tests for AI-written code
- Build a working Plant Helper with a soil sensor, traffic-light LEDs, a buzzer and a button

---

## What you'll need

- A Raspberry Pi Pico or Pico W running MicroPython, and a USB cable
- A capacitive soil moisture sensor (v1.2 style, analogue output)
- Three LEDs (green, amber, red) and three 220 ohm resistors
- A piezo buzzer and a push button
- A breadboard and jumper wires
- A potted plant and a glass of water (for calibrating the sensor)
- Thonny or Snakie to run MicroPython, and an AI assistant you're allowed to use
- Paper, pens, sticky notes and some cardboard

---

## How the course works

Each lesson opens with what you'll be able to do by the end, explains one idea, and then applies it to the Plant Helper in a "Plant Helper progress" section. You'll finish each lesson with a **Try it Yourself** section, some **Common Issues**, and a short quiz.

Code blocks marked as Python are complete programs you can run. Most run on the Pico in MicroPython; lesson 8 has a short script for your computer. The people in the Plant Helper story, Ms Okafor, Leo, Priya and Mr Hughes, are a made-up example, so you can see the method used on someone. In your own project you'll use real people.

### Before you start

You don't need to be an expert, but these help:

- [Learn MicroPython](/learn/micropython/) for the Pico code
- [Getting started with Snakie](/learn/snakie_getting_started/) if you want an editor with an AI chat built in
- [Learn Git](/learn/git/), because we use commits as save points when the AI is writing code

> **Teachers**: this course suits ages 11 and up. It works as 10 to 14 sessions of 50 minutes. Lessons 1 to 7 and lesson 10 need no hardware at all, just paper, pens and cardboard. For the build, one kit per group of three or four is plenty. Most AI tools have minimum age rules and need an account, so check them and your school's policy first. A simple option is for the teacher to drive a single AI account on the projector while the class decides what to ask.
