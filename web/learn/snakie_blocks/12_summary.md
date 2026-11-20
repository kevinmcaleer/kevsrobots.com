---
layout: lesson
title: Summary and Next Steps
author: Kevin McAleer
type: page
cover: /learn/snakie_blocks/assets/banners/12_summary.jpg
date: 2026-11-20
previous: 11_project_reaction_timer.html
description: Recap of the Snakie Blocks course, with blocks in other languages, tips
  for teachers and where to go next
percent: 100
duration: 4
quiz_total: 36
date_updated: 2026-11-20
navigation:
- name: Snakie Blocks - From Blocks to Python
- content:
  - section: Overview
    content:
    - name: Snakie Blocks - From Blocks to Python
      link: 00_intro.html
    - name: The Blocks Workspace
      link: 01_the_blocks_workspace.html
  - section: Blocks Basics
    content:
    - name: Turtle Blocks - Draw a Square
      link: 02_turtle_draw_a_square.html
    - name: Variables, Maths and Loops
      link: 03_variables_maths_and_loops.html
    - name: Making Decisions
      link: 04_making_decisions.html
  - section: Blocks and Hardware
    content:
    - name: Hardware Blocks - LEDs and Buttons
      link: 05_leds_and_buttons.html
    - name: PWM, Servos and Buzzers
      link: 06_pwm_servos_and_buzzers.html
    - name: Functions
      link: 07_functions.html
    - name: My Parts
      link: 08_my_parts.html
  - section: From Blocks to Python
    content:
    - name: Reading the Python
      link: 09_reading_the_python.html
    - name: Simple and Advanced Blocks
      link: 10_simple_and_advanced_blocks.html
  - section: Project
    content:
    - name: Project - Reaction Timer
      link: 11_project_reaction_timer.html
    - name: Summary and Next Steps
      link: 12_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

You started by dragging a square into existence and finished by reading the Python for a game you built. Nice work.

---

## What you learned

| Blocks | Python |
|---|---|
| repeat 4 times | `for _ in range(4):` |
| forever | `while True:` |
| set size to 10 | `size = 10` |
| if roll = 6 | `if roll == 6:` |
| toggle LED on GP15 | `pin_15.toggle()` |
| wait 400 milliseconds | `time.sleep_ms(400)` |
| set power of GP15 to 50 % | `pwm_15.duty_u16(...)` |
| buzzer on GP16 play 440 Hz for 200 ms | `buzzer_16.tone(440, 200)` |
| to polygon with: sides, size | `def polygon(sides, size):` |
| create list with | `[262, 294, 330]` |
{:class="table table-single"}

You also know how to move between the two: hover a block, click a line, Show me the Python, and type in whichever pane suits you.

## Blocks in your language

The Blocks canvas can be shown in any of the 80 languages Scratch offers, from Cymraeg to 简体中文. Go to **Settings ▸ Appearance ▸ Language**.

A language changes the words on the blocks and in the toolbox. It doesn't change the program: the Python is identical in every language, so a program built in Deutsch opens unchanged in English.

Snakie's own blocks, such as the hardware and turtle ones, are translated into German, French and Spanish so far. In other languages those show in English, alongside the translated standard blocks. The rest of the app's menus are in English for now.

On the web you can also add `?lang=fr` to the address to try a language for one visit.

## Tips for teachers

- **Hardly any hardware needed.** Everything except pressing a real button runs on the simulator, on any laptop or Chromebook, at [app.snakie.org](https://app.snakie.org)
- **Start with the turtle.** A square on screen in the first ten minutes beats wiring an LED
- **Say it out loud once**: the thing on the right is what the thing on the left means
- **The best instruction in a blocks lesson**: "change that number and watch what happens on the right"
- **Hand out a link.** `https://app.snakie.org/?lang=cy` opens Snakie with the blocks in that language for that visit, and saves nothing
- **Work goes home.** A blocks program is one ordinary `.py` file, with no export step and no project folder
- **Let them drift.** There's no "switch to Python" moment to manage. Learners who start predicting the Python, or get annoyed that a block can't do something, are already crossing over

Snakie also has its own short course built in: open **Learn** in the app and choose **Blocks to Python**. Each of its lessons opens with a working program ready to take apart, which makes a good warm-up or homework alongside this course.

## What's Next

You're ready for:

- [Learn MicroPython - The basics](/learn/micropython/) covers the language properly, now that you can already read it
- [Raspberry Pi Pico with MicroPython - GPIO Mastery](/learn/micropython_gpio/) goes deeper into pins, PWM and sensors
- [MicroPython Robotics Projects with the Raspberry Pi Pico](/learn/micropython_robotics/) puts it all on wheels

This course is part of the [Start with Snakie](/learn/learning_pathways/snakie.html) learning pathway. Courses on wiring circuits in the Electronics workspace and on Snakie's instruments are on their way.

If you skipped it, [Getting Started with Snakie](/learn/snakie_getting_started/) covers the rest of the app: the REPL, finding your board, flashing MicroPython and managing files.

---

## Try it Yourself

One last challenge. Pick any project from this course and rebuild it **from the code side**: start a new file in the split view, type the Python, and watch the blocks build themselves.

## Check your understanding

{% quiz %}
? What changes when you switch the blocks to another language?
- [x] The words on the blocks and toolbox
- [ ] The Python that's generated
- [ ] The pins on the board
> Only the presentation changes. The program is the same in every language.

? Which block writes `while True:`?
- [ ] repeat 10 times
- [x] forever
- [ ] if
> Forever is a loop that never ends.

? What's the best first lesson for a class with no hardware?
- [ ] Wiring an LED
- [x] Drawing with the turtle on the simulator
- [ ] Flashing firmware
> The turtle needs nothing plugged in and gives a result in minutes.
{% endquiz %}
