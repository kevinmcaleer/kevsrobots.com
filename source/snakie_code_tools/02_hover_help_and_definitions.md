---
title: Hover Help and Definitions
description: Read documentation by hovering, open a full help article for any symbol and jump straight to where a function is defined in Snakie
layout: lesson
type: page
cover: /learn/snakie_code_tools/assets/banners/02_hover_help_and_definitions.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to use hover help, Help for symbol and Go to Definition to find out what a name is and where it came from.

Reading code is harder than writing it. These tools answer the two questions you ask most when reading: *what is this?* and *where did it come from?*

---

## Hover

Rest the mouse on any name. Here it's on `PWM` in line 10 of the robot's `main.py`.

![Hovering over PWM shows where it was imported and its documentation](/learn/snakie_code_tools/assets/hover.jpg){:class="img-fluid w-100"}

The card tells you three things:

1. **Where the name came from.** *import defined on line 1*, with the line itself
2. **How to get there.** A **Go to line 1** link
3. **What it is.** The signature of `machine.PWM` and its description from the MicroPython documentation

Hover over a wavy underline and you get the reason for it, which we'll use a lot in lesson 4.

## Help for symbol

A hover card is a summary. For the full story, right-click a name.

![The editor's right-click menu](/learn/snakie_code_tools/assets/context_menu.jpg){:class="img-fluid w-100"}

Choose **Help for symbol (Snakie)**. The Help panel opens on the left with an article about that name.

![The Help panel showing the PWM article](/learn/snakie_code_tools/assets/symbol_help.jpg){:class="img-fluid w-100"}

For `PWM` you get the basics, a fade example and how to drive a servo, with code you can copy. It's the same reference as the **Help** button in the activity bar, opened at the right page.

## Go to Definition

Somewhere near the bottom of `main.py` is a call to `drive(60)`. To see what `drive` does, put the cursor on the name and press <kbd>F12</kbd>. The editor jumps to the `def drive(speed):` line.

There are three ways to do it:

| How | Where |
|---|---|
| <kbd>F12</kbd> | With the cursor on the name |
| <kbd>Ctrl</kbd>+click | Hold Ctrl and the name becomes a link |
| Right-click, **Go to Definition** | From the menu |
{:class="table table-single"}

It works for functions, classes, variables and imports **in the file you're in**. For a class imported from another of your files, such as `HCSR04`, it takes you to the import line. Open that file from the Files panel to read the class itself.

To find your way back afterwards, use the outline below.

## The outline

Click **Inspect** in the activity bar. The top of the panel lists every constant, variable and function at the top level of the file, each with its line number.

For a long file this is a table of contents. `LEFT_MOTOR_PIN  5`, `sensor  9`, `drive (speed)  14`. With a board connected, the lower half of the same panel shows the live values of the board's global variables.

---

## Try it Yourself

1. Hover over `time` in `import time`, then over `sleep` in `time.sleep(0.1)`. How do the two cards differ?
2. Use **Help for symbol** on `Pin`. Find the section on pull-ups.
3. Put the cursor on `TOO_CLOSE_MM` where it's used in the `if`, and press <kbd>F12</kbd>. Where do you land?

## Common Issues

- **Problem**: Nothing appears when I hover.
- **Solution**: Hold the mouse still on the name for a moment, and check it's a name and not a number or a bracket.
- **Why**: The card waits briefly so it doesn't flash up as you move across the code.

- **Problem**: <kbd>F12</kbd> does nothing.
- **Solution**: Click on the name first so the cursor is in it. On some laptops you need the <kbd>Fn</kbd> key for function keys, or use <kbd>Ctrl</kbd>+click.
- **Why**: Go to Definition acts on the name under the cursor.

- **Problem**: Help for symbol opens a general page.
- **Solution**: Right-click directly on the name you want, such as `PWM`, not on the variable it's assigned to.
- **Why**: Articles exist for MicroPython's classes and functions, not for your own variable names.

## Check your understanding

{% quiz %}
? What does hovering over an imported name show?
- [ ] Only its spelling
- [x] Where it was imported and its documentation
- [ ] The pin it's connected to
> The card gives the origin, a link to that line, and the MicroPython description.

? Which key jumps to a function's definition?
- [ ] F5
- [ ] Ctrl+F
- [x] F12
> F12, Ctrl+click and the right-click menu all do the same thing.

? Where is the list of a file's top-level functions and variables?
- [x] The Inspect panel
- [ ] The Packages panel
- [ ] The console
> Inspect shows the file's outline, and the board's live variables when connected.
{% endquiz %}
