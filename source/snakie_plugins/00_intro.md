---
title: Extend Snakie with Python Plugins
description: Learn to write Snakie plugins in Python, with commands, a linter with quick fixes and custom blocks for a robot club
layout: lesson
type: page
cover: /learn/snakie_plugins/assets/banners/00_intro.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

## Overview

Snakie is built for people who write Python, so you extend it in **Python** too. A plugin is a small Python file on your computer. It can add commands that read or rewrite your code, underline mistakes as you type, offer one-click fixes, and add new blocks to the Blocks workspace.

In this course you'll build one plugin from an empty file: **ClubKit**, a toolkit for a robot club. By the end it will stamp a header on every program, list the pins a program uses, catch two classic MicroPython mistakes while you type, fix one of them with a click, and give the club's robot its own drawer of blocks.

{% include youtubeplayer.html id="QVe9meIHZtk" %}

New to Snakie? Watch Kev's introduction above first, then come back.

---

## Course Content

- How Snakie runs plugins, and why that matters for safety
- The Plugin Rack and the plugins that come with Snakie
- Writing a plugin with one command
- What happens when a plugin has a bug, and how to find it
- Commands that read your code and report back
- Commands that change your code
- A linter that underlines problems as you type
- Quick fixes on the lightbulb
- Rules that know about your hardware
- Custom blocks for the Blocks workspace
- Sharing your plugin with a club or a class

---

## Key Results

After this course you will be able to:

- Create a Snakie plugin and load it into the Plugin Rack
- Write commands that show messages, update the status bar and edit the open file
- Write a linter that puts warnings and errors in the editor and the Problems panel
- Attach a quick fix to a warning
- Add your own blocks that generate real MicroPython
- Package a plugin so other people can use it, and judge whether a plugin is safe to install

---

## What you'll need

- The **Snakie desktop app** for Windows, macOS or Linux. Plugins don't run in the browser version at [app.snakie.org](https://app.snakie.org)
- **Python 3** installed on your computer. Snakie uses it to run plugins
- Some Python: functions, loops, strings and `import`. If decorators (the `@something` lines) are new, don't worry, we'll explain the two you need
- No robot and no microcontroller. Plugins work on the code in the editor

It helps to have done [Snakie - Getting Started](/learn/snakie_getting_started/) and [Coding Faster in Snakie](/learn/snakie_code_tools/), which covers the built-in linter and the Problems panel that your plugin will plug into.

This course was written with Snakie **0.92.0**. Snakie changes quickly, so a button may have moved by the time you read this.

---

## How the course works

Each lesson adds one feature to the same plugin, so do them in order. The plugin code is shown in full each time it grows, and the finished file is in the last lesson if you get lost.

Plugin code is ordinary Python that runs on your computer, not MicroPython on a board. Every lesson has a **Try it Yourself** task and a short quiz.

> **A note on trust.** A plugin can do anything a Python program on your computer can do. That's what makes plugins powerful, and it's why you should only install ones you trust. We'll come back to this in the last lessons.

> **Teachers**: This course suits learners who already write some Python and have used Snakie, such as an older class or a coding club. Each lesson is roughly 30 to 45 minutes. Unlike most Snakie courses it does not run in the browser: plugins need the Snakie desktop app and Python 3 on the computer, and they load from a `.snakie/plugins` folder in each user's home folder, so check that school computers allow both before the first lesson. No robot or microcontroller is needed. Because a plugin can do anything a Python program can do, make sure learners only install plugins that they or their teacher wrote, or that someone they trust has read.
