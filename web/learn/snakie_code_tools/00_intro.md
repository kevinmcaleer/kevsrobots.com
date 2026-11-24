---
layout: lesson
title: Coding Faster in Snakie
author: Kevin McAleer
type: page
cover: /learn/snakie_code_tools/assets/banners/00_intro.jpg
date: 2026-11-21
next: 01_smart_autocomplete.html
description: Learn the editor tools in Snakie that make MicroPython faster to write
  and safer to change, from autocomplete and linting to refactors, packages and Git
percent: 6
duration: 4
quiz_total: 42
date_updated: 2026-11-24
navigation:
- name: Coding Faster in Snakie
- content:
  - section: Overview
    content:
    - name: Coding Faster in Snakie
      link: 00_intro.html
  - section: Writing
    content:
    - name: Smart Autocomplete
      link: 01_smart_autocomplete.html
    - name: Hover Help and Definitions
      link: 02_hover_help_and_definitions.html
    - name: Find and Replace
      link: 03_find_and_replace.html
  - section: Checking
    content:
    - name: Linting and the Problems Panel
      link: 04_linting_and_problems.html
    - name: MicroPython Refactors
      link: 05_micropython_refactors.html
    - name: YAML and JSON Validation
      link: 06_yaml_and_json.html
  - section: Libraries
    content:
    - name: Installing Packages
      link: 07_installing_packages.html
    - name: The Modules Shelf
      link: 08_the_modules_shelf.html
    - name: Compiling to mpy
      link: 09_compiling_to_mpy.html
  - section: Working Like a Pro
    content:
    - name: Version Control with Git
      link: 10_version_control_with_git.html
    - name: AI Chat and Suggestions
      link: 11_ai_chat_and_suggestions.html
    - name: Menus and Keyboard Shortcuts
      link: 12_menus_and_shortcuts.html
  - section: Project
    content:
    - name: Project - A Tidy Sensor Driver
      link: 13_project_sensor_driver.html
    - name: Summary and What's Next
      link: 14_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

{% include youtubeplayer.html id="qfT_Ndq-tJI" %}

## Overview

You can write MicroPython in any text editor. What makes Snakie different is that its editor *knows MicroPython*. It knows `PWM` has a `duty_u16()` method. It knows a float in `time.sleep()` costs memory on a Pico. It knows which modules are already inside your board's firmware.

This course is a tour of those tools. None of them write your robot for you. All of them remove a little friction: a typo caught before you press Run, an answer without leaving the editor, a change you can undo next week.

![A refactoring hint with a one-click fix](/learn/snakie_code_tools/assets/quickfix.jpg){:class="img-fluid w-100"}

By the end you'll have used every one of them on a real job: a driver for an ultrasonic distance sensor, checked, tidied, compiled and saved in Git.

## Course Content

- Autocomplete and signature help that know MicroPython
- Hover help, Help for symbol and jumping to a definition
- Find and Replace
- Linting with ruff and the Problems panel
- MicroPython refactoring hints, quick fixes and Tidy this file
- YAML and JSON validation
- Installing packages with mip
- The Modules shelf: what's on the board and what's built in
- Compiling `.py` files to `.mpy`
- Version control with the built-in Git panel
- The AI chat pane and ghost-text suggestions
- Menus and keyboard shortcuts
- Project: a tidy, compiled, version-controlled sensor driver

## Key Results

After this course you will be able to:

- Write MicroPython with fewer trips to the documentation
- Read the Problems panel and fix what it finds
- Apply a refactoring safely, with a preview of every change
- Decide whether a library needs installing or is already on the board
- Shrink a module by compiling it
- Commit your work and see exactly what changed since

## What you'll need

- [Snakie](https://app.snakie.org). Most lessons work in the browser
- The **Snakie desktop app** for four lessons: Python linting, Git, the AI chat and the full menus. Each lesson says so at the top
- No hardware. A Raspberry Pi Pico is useful for the packages lesson but not required

| Feature | Browser | Desktop app |
|---|---|---|
| Autocomplete, hover, Go to Definition | Yes | Yes |
| Find and Replace | Yes | Yes |
| MicroPython refactoring hints | Yes | Yes |
| YAML and JSON validation | Yes | Yes |
| Packages and Modules | Yes | Yes |
| Python linting with ruff | No | Yes |
| Source Control (Git) | No | Yes |
| AI chat | No | Yes |
{:class="table table-single"}

## How the course works

Each lesson covers one tool, with screenshots and a short task. We use the same small project throughout: a two-wheeled robot's `main.py` and its distance sensor driver, `hcsr04.py`.

Where a lesson has code to run, it's in a live Snakie box. Screenshots are from Snakie 0.92 on the desktop, using the Ctrl key. On a Mac, read <kbd>Ctrl</kbd> as <kbd>Cmd</kbd>.

Every lesson ends with things to try, fixes for common problems and a short quiz.

> **Teachers**: this is an intermediate course for learners who can already write a short MicroPython program. It needs no hardware, and most lessons work in the browser. Four lessons (Python linting, Git, the AI chat and the full menus) need the Snakie desktop app, so plan installs in advance, and Git needs Git installed on the computer. The AI chat needs a learner's own API key, which is tied to a payment account, so decide who holds keys before using it with a class.

## Prerequisites

This is an intermediate course. You should be able to write a short MicroPython program with functions and a loop.

- [Getting Started with Snakie](/learn/snakie_getting_started/) covers the basics of the app
- [Learn MicroPython - The basics](/learn/micropython/) covers the language
- [Wire It Up - Electronics in Snakie](/learn/snakie_electronics/) builds the robot this course's code belongs to

Let's start with the tool you'll use every few seconds.
