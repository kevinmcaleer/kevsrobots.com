---
layout: lesson
title: Find and Replace
author: Kevin McAleer
type: page
cover: /learn/snakie_code_tools/assets/banners/03_find_and_replace.jpg
date: 2026-11-21
previous: 02_hover_help_and_definitions.html
next: 04_linting_and_problems.html
description: Search a MicroPython file and change many places at once with Snakie's
  Find and Replace, and learn when whole-word matching saves you from a bug
percent: 24
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

By the end of this lesson you will be able to find and replace names across a file safely, one at a time or all at once.

Sooner or later you'll want to change the same thing in six places. Doing it by hand means missing one. This is what Find and Replace is for.

---

## Opening it

Click **Find** at the top right of the editor, or press <kbd>Ctrl</kbd>+<kbd>F</kbd>. To go straight to replacing, press <kbd>Ctrl</kbd>+<kbd>H</kbd>. In the desktop app it's also under **Tools ▸ Find & Replace…**, and it opens as a small window of its own that you can park beside your code.

![The Find and Replace window](/learn/snakie_code_tools/assets/find_window.jpg){:class="img-fluid w-100"}

Type `MOTOR_PIN` in the **Find** box. Every match in the file is highlighted, and the window counts them: **4 matches**.

## The controls

| Control | What it does |
|---|---|
| **Aa** | Match case. `pin` no longer finds `Pin` |
| **\|ab\|** | Whole word. `left` no longer finds `left_speed` |
| **↑** **↓** | Previous and next match |
| **Replace** | Change the current match |
| **Replace+Find** | Change it and move to the next |
| **Replace all** | Change every match at once |
{:class="table table-single"}

The keyboard does the same: <kbd>Enter</kbd> for the next match, <kbd>Shift</kbd>+<kbd>Enter</kbd> for the previous one, <kbd>Esc</kbd> to close.

## Why whole word matters

Say you want to rename the variable `left` to `left_motor`. Our file has these lines:

```python
left = PWM(Pin(LEFT_MOTOR_PIN))
left.duty_u16(duty)
```
{: .no-snakie }

Search for `left` with nothing switched on and you might expect two matches. But a search that ignores case also finds the `LEFT` in `LEFT_MOTOR_PIN`. **Replace all** would turn that into `left_motor_MOTOR_PIN`, and your program would stop with a `NameError`.

Switch on **Aa** and **\|ab\|** before any Replace all. Then check the match count is what you expect. If you thought there were two and it says five, stop and look.

## Replace one at a time

When you're not sure, don't use Replace all. Use **Replace+Find**. It changes the highlighted match and jumps to the next, so you see every change as it's made. Press **↓** to skip one you want to keep.

And remember <kbd>Ctrl</kbd>+<kbd>Z</kbd>. A Replace all that went wrong is one undo away.

## A quicker way for one name

If you only want to change a name everywhere in the file, there's a shortcut that needs no window. Put the cursor on the name and press <kbd>Ctrl</kbd>+<kbd>F2</kbd> (**Change All Occurrences** in the right-click menu). Every occurrence gets a cursor, and whatever you type is typed in all of them at once.

It matches the text, not the meaning, so the same care applies.

## Find is also for reading

Search isn't only for changing things. <kbd>Ctrl</kbd>+<kbd>F</kbd>, type `sleep`, and step through with <kbd>Enter</kbd>: you've just found every place your program pauses. That's often the first step in working out why a robot feels sluggish.

---

## Try it Yourself

1. In the robot's `main.py`, find `drive`. How many matches are there with whole word on, and what are they?
2. Replace `TOO_CLOSE_MM` with `STOP_DISTANCE_MM` using Replace+Find, one at a time.
3. Use <kbd>Ctrl</kbd>+<kbd>F2</kbd> on `duty` inside the `drive` function and rename it to `level`. Undo it afterwards.

## Common Issues

- **Problem**: Replace all changed things I didn't mean to change.
- **Solution**: Press <kbd>Ctrl</kbd>+<kbd>Z</kbd>, then switch on **Aa** and **\|ab\|** and try again.
- **Why**: Without them, a short search term matches inside longer names.

- **Problem**: It says no matches, but I can see the word.
- **Solution**: Check **Aa** isn't on when the capitals differ, and that there's no stray space in the Find box.
- **Why**: With match case on, `Pin` and `pin` are different words.

- **Problem**: I lost the Find window.
- **Solution**: Press <kbd>Ctrl</kbd>+<kbd>F</kbd> again.
- **Why**: On the desktop it's a separate window, and it can slip behind the main one.

## Check your understanding

{% quiz %}
? What does the |ab| button do?
- [ ] Makes the text bold
- [x] Matches whole words only
- [ ] Replaces everything
> With whole word on, "left" won't match inside "left_speed".

? What should you check before pressing Replace all?
- [ ] The board is connected
- [ ] The file is saved
- [x] The match count is what you expect
> An unexpected count means the search is finding things you didn't intend.

? Which shortcut edits every occurrence of a name at once, with no window?
- [x] Ctrl+F2
- [ ] Ctrl+P
- [ ] F12
> Change All Occurrences puts a cursor on each one.
{% endquiz %}
