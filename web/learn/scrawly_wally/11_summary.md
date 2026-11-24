---
layout: lesson
title: Summary and Next Steps
author: Kevin McAleer
type: page
cover: /learn/scrawly_wally/assets/banners/11_summary.jpg
date: 2024-08-02
previous: 10_vectorize.html
description: Recap what you built with Scrawly Wally, what limits a plotter like this,
  and where to go next.
percent: 100
duration: 5
quiz_total: 30
date_updated: 2026-11-24
navigation:
- name: Wall Drawing Robot Tutorial
- content:
  - section: Introduction
    content:
    - name: Introduction to the Wall Drawing Robot
      link: 00_intro.html
  - section: Build the Robot
    content:
    - name: Components Needed
      link: 02_components.html
    - name: Assembling the Robot
      link: 03_assembly.html
    - name: Wiring the Components
      link: 04_wiring.html
  - section: Programming
    content:
    - name: Setting Up the Raspberry Pi
      link: 05_setup_pi.html
    - name: Installing the Software
      link: 06_software.html
    - name: Writing Control Code
      link: 07_control.html
    - name: Understanding Positioning with Trigonometry
      link: 08_positioning.html
    - name: Creating Your First Drawing
      link: 09_draw.html
    - name: Vectorizing Images for Drawing
      link: 10_vectorize.html
  - section: Summary
    content:
    - name: Summary and Next Steps
      link: 11_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to describe how the whole robot fits together, name what limits its accuracy, and choose what to build or learn next.

Well done, makers! You've gone from a pile of parts to a robot that draws on a wall. Let's look back at how it all fits together, and then at where you could take it.

## What you built

```mermaid
flowchart LR
    A["SVG or shape"] --> B["Wall coordinates (x, y)"]
    B --> C["Belt lengths: Pythagoras"]
    C --> D["Motor steps: steps per mm"]
    D --> E["Half-step patterns on the pins"]
    E --> F["Pen moves, servo lifts and lowers"]
```

Each lesson added one link in that chain:

1. **Parts and build** (lessons 2 and 3): a Pi, two 28BYJ-48 steppers with ULN2003 drivers, an SG90 servo, 3GT cogs and belts, and three 3D printed parts.
2. **Wiring and setup** (lessons 4 to 6): eight signal wires for the steppers and one for the servo, and a Pi with `RPi.GPIO` and Kev's repository, with serial, SPI and I2C left off.
3. **Movement** (lesson 7): a half-step pattern that turns each motor, and PWM that moves the servo.
4. **Positioning** (lesson 8): the pen's position gives two belt lengths, and the belt lengths give steps.
5. **Drawing** (lessons 9 and 10): short straight pieces, motors stepped together, and SVG strokes scaled to your board.

## What limits a plotter like this

Knowing the limits makes you a better maker, because you can see why a drawing goes wrong.

- **No feedback.** The steppers have no sensor. The Pi trusts that every step happened. A stall or a slipping cog loses steps, and the error stays for the rest of the drawing.
- **Your measurements.** `BOARD_W`, `START` and `STEPS_PER_MM` all come from a ruler. A few millimetres of error in any of them shows in the drawing.
- **Belt and gondola.** A stretchy belt, a loose clamp or a swinging gondola moves the pen without the motors knowing.
- **Speed.** Stepping at 2 ms per step is gentle. Even a small drawing takes minutes.

> **Teachers**: for a finished project, ask groups to document their robot's measured values (`BOARD_W`, `STEPS_PER_MM`, pen angles and which way each motor turns) and a dry-run printout next to a photo of the real drawing. The differences make a good discussion about calibration and error. If your robot hangs on a wall, check the fixings again at the end of the project and before it is left up between lessons.

## Try it Yourself

1. Draw the same square three times. How close do the corners come to meeting each time?
2. Measure the drawn square with a ruler. How close is it to 200 mm, and what does that tell you about `STEPS_PER_MM`?
3. Pick a favourite simple logo, trace it in Inkscape and draw it on your robot.
4. Challenge: think of one way to give the robot feedback, so it can tell when it has lost steps. What would you add?

## Common Issues

- **Problem**: The drawing gets less accurate as it goes on.
- **Solution**: Slow the steps down a little with `STEP_DELAY`, and check the cog grub screws and belt clamps.
- **Why**: Lost steps add up, because the robot never checks where the pen really is.

- **Problem**: The shape is the right shape but the wrong size.
- **Solution**: Recalibrate `STEPS_PER_MM`.
- **Why**: Size comes straight from the steps-per-millimetre number.

## Where next?

- Try a different kind of drawing robot with the [BrachioGraph tutorial](/learn/brachiograph/), which uses servos and an arm.
- Learn more about steppers, servos and drivers in [Motors and motor drivers](/learn/motors_and_drivers/).
- Strengthen your Python with the [Python course](/learn/python/), and your terminal skills with the [Linux command line course](/learn/linux_intro/).
- Revisit the build files in this course's `assets` folder and improve the gondola or the mounts.

## Check your understanding

{% quiz %}
? What is the chain from a drawing to the pen moving?
- [ ] Pixels, colours, then pen
- [x] (x, y) position, belt lengths, motor steps, then pin patterns
- [ ] Belt lengths, then x and y, then steps
> Each step of the chain converts one thing into the next, as in the diagram.

? Why do errors build up during a long drawing?
- [ ] The marker runs dry
- [x] The steppers have no feedback, so any lost step stays lost
- [ ] Python slows down
> The Pi counts the steps it sent, not the steps that happened.

? Which of these should you measure rather than guess?
- [ ] The colour of the marker
- [x] `BOARD_W`, `START` and `STEPS_PER_MM`
- [ ] The name of the file
> The robot trusts these numbers entirely, so they must match your build.
{% endquiz %}

Thanks for building along. Share what your Scrawly Wally draws with the community, and happy plotting!
