---
layout: lesson
title: Learn MicroPython Introduction Video
author: Kevin McAleer
type: page
cover: /learn/micropython/assets/banners/00_videos.jpg
date: 2022-12-04
previous: 00_intro.html
next: 01_why_is_it_called_micropython.html
description: Watch the MicroPython Introduction Video
percent: 10
duration: 3
quiz_total: 57
date_updated: 2026-11-24
navigation:
- name: Learn MicroPython - The basics
- content:
  - section: Overview
    content:
    - name: Introduction
      link: 00_intro.html
    - name: Learn MicroPython Introduction Video
      link: 00_videos.html
  - section: Introduction
    content:
    - name: Why is it called MicroPython?
      link: 01_why_is_it_called_micropython.html
    - name: Where to get MicroPython
      link: 02_where_to_get_micropython.html
    - name: How to Install MicroPython
      link: 03_installing_micropython.html
    - name: Why use MicroPython?
      link: 04_why_use_micropython.html
    - name: Python Development Environments
      link: 05_ides.html
    - name: Our first program
      link: 06_our_first_program.html
    - name: Example 01
      link: 07_hello_world.html
  - section: Variables and Reserved Words
    content:
    - name: Variables and Constants
      link: 08_variables.html
    - name: Example 02
      link: 09_example02.html
    - name: Values & Variables Types
      link: 10_values_and_variable_types.html
    - name: Reserved Words
      link: 11_reserved_words.html
  - section: Controlling the Flow
    content:
    - name: If, elif, else
      link: 12_if_elif_else.html
    - name: Loops
      link: 13_loops.html
    - name: Operators
      link: 14_operators.html
  - section: Functions and Modules
    content:
    - name: Functions
      link: 15_functions.html
    - name: The REPL
      link: 16_repl.html
    - name: Modules
      link: 17_modules.html
  - section: Summary and Review
    content:
    - name: Summary and Review
      link: 18_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will know which video goes with which part of the course, so you can watch and read side by side.

## YouTube Series

I've recorded the Learn MicroPython course in video form, so if you prefer to watch than read you can follow along here too. If you find this of value, please consider [subscribing](https://www.youtube.com/c/kevinmcaleer28).

The three videos were recorded in 2022, so a few screens, such as the Thonny menus and the firmware version number, look a little different today. The written lessons are the up-to-date version.

{% include youtubeplayer.html id="TkuDQ-SzXwU" %}

{% include youtubeplayer.html id="ghAnP18sbl8" %}

{% include youtubeplayer.html id="HGRvgrl8g1Q" %}

## Which video matches which lesson?

| Video | Matching lessons |
|---|---|
| Part 1: Controlling the flow | Lessons 12 and 13 (decisions and loops) |
| Part 2: If, Else and Data Types | Lessons 10 and 12 (data types, `if`, `elif`, `else`) |
| Part 3: Loops and Operators | Lessons 13 and 14 (loops and operators) |
{:class="table table-single"}

You can watch them in any order, but the written course builds up from installing MicroPython, so start at lesson 1 if you are brand new.

## Try it Yourself

1. Watch the first few minutes of any video, then find the matching written lesson and compare. What is the same, and what looks different?
2. Pause a video before the presenter runs a program, and guess what it will print. Then press play and check.
3. Challenge: after a video, close it and re-type the example from memory in your editor.

## Check your understanding

{% quiz %}
? Which lessons go with the Loops and Operators video?
- [ ] Lessons 1 and 2
- [x] Lessons 13 and 14
- [ ] Lessons 16 and 17
> Loops are lesson 13 and operators are lesson 14, as the table above shows.

? Why might a screen in a 2022 video look different from your screen today?
- [ ] The videos are for a different language
- [x] The videos were recorded in 2022 and software menus and version numbers have changed
- [ ] Only the written lessons use Thonny
> The written lessons are the up-to-date version of the course.

? You are brand new to MicroPython. Where should you start?
- [ ] Lesson 17, because it is the last
- [x] Lesson 1, because the course builds up from installing MicroPython
> The course is ordered so each lesson introduces one new idea.
{% endquiz %}
