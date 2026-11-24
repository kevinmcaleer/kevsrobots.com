---
title: The Instrument Dock
description: Find Snakie's instrument dock, open and close instruments, read their help pages and float one in its own window
layout: lesson
type: page
cover: /learn/snakie_instruments/assets/banners/01_the_instrument_dock.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to find the Instrument Dock, open and close instruments, read their help pages and float one in its own window.

Every instrument in Snakie lives in one place. Let's find it and learn the controls they all share.

---

## Where the dock is

Open Snakie and stay in the **Code** workspace. On the right, under the picture of your board, is a panel titled **INSTRUMENT DOCK**.

It holds two rows of small icons:

- **INPUTS** are instruments that show you something: Oscilloscope, Multimeter, Plotter, Data Logger, Range, Potentiometer, IMU, Barometer, Button, Encoder, Display, Wi-Fi scan, Bluetooth, I²C detect and Gamepad
- **OUTPUTS** are panels that drive something: LED, Bench PSU, Servo, Motor, Poses, Buzzer and more

Hover over an icon and Snakie names it, for example **Show Plotter**. Click it and the instrument opens underneath. Click again to hide it.

You can open several at once. They stack down the panel, and the panel scrolls.

## Finding one by name

Icons are quick once you know them. Until then, click **+ Add** under the icons. A searchable list opens with a line about what each instrument is for.

![The Add list with a description of each instrument](/learn/snakie_instruments/assets/add_menu.jpg){:class="img-fluid w-100"}

Type `plot` in the search box, click **Plotter**, and it opens.

## The dock knows what your code uses

Paste a program that creates a `PWM` or an `ADC` object and look at the icons again. The matching instruments gain a small dot, and their hover text adds **(used by this file)**. In the Add list they carry an **IN USE** badge.

It's Snakie's way of saying: *this file has something an oscilloscope could show you.*

## The buttons every instrument has

Each instrument has a title bar with the same small buttons:

| Button | What it does |
|---|---|
| **?** | Opens that instrument's help page in the Help panel |
| **LIVE** | Oscilloscope and Multimeter only. Lets Snakie read the board itself. Lesson 2 |
| **Float as overlay** | Lifts the instrument out into its own window |
| **Close** | Hides it again |
{:class="table table-single"}

The **?** button is worth a click on every instrument you meet. The help page says what the instrument shows, how to feed it, and gives a snippet to copy.

![The Plotter's help page beside the code](/learn/snakie_instruments/assets/instr_help.jpg){:class="img-fluid w-100"}

**Float as overlay** is the one to remember when you're short of room. A floating oscilloscope can sit over the console, or on a second screen, while the dock shows something else.

---

## Try it Yourself

1. Open the Plotter, the Multimeter and the Oscilloscope together, then close them one at a time.
2. Use **+ Add** to find the instrument that scans the I²C bus. What does its description say?
3. Open the Multimeter's **?** page and find the snippet. Which function does it call?

## Common Issues

**Problem**: I can't see the Instrument Dock.
**Solution**: Make sure you're in the **Code** workspace, and open the right-hand panel with the small arrow at its top edge.
**Why**: The dock is part of the Code workspace's side panel, which can be collapsed.

**Problem**: The plotter is already drawing lines and I haven't run anything.
**Solution**: Nothing is wrong. That's the simulated device's demo data.
**Why**: The simulator streams sample readings so the instruments aren't blank. A real board only shows what your program sends.

**Problem**: An instrument has gone missing after I floated it.
**Solution**: Look for its separate window, close that, and open the instrument from the dock again.
**Why**: A floated instrument lives in its own window, which can hide behind the main one.

## Check your understanding

{% quiz %}
? Where is the Instrument Dock?
- [ ] In the Electronics workspace
- [x] In the Code workspace, under the board view
- [ ] In Settings
> It sits on the right of the Code workspace, right beside the program it's measuring.

? What does a dot on an instrument's icon tell you?
- [ ] The instrument is broken
- [ ] It's recording
- [x] The open file uses something that instrument can show
> Create a PWM or ADC object and the matching instruments light up.

? Which button opens an instrument's help page?
- [x] ?
- [ ] LIVE
- [ ] Close
> Every instrument has its own help article with a snippet to copy.
{% endquiz %}
