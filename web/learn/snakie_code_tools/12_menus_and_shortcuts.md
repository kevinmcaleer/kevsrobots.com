---
layout: lesson
title: Menus and Keyboard Shortcuts
author: Kevin McAleer
type: page
cover: /learn/snakie_code_tools/assets/banners/12_menus_and_shortcuts.jpg
date: 2026-11-21
previous: 11_ai_chat_and_suggestions.html
next: 13_project_sensor_driver.html
description: Learn Snakie's menus and the keyboard shortcuts worth memorising, and
  where to find the built-in shortcut sheet
percent: 78
duration: 5
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

The mouse is fine for exploring. Once you know where things are, the keyboard is quicker. This lesson is a map of the menus and a short list of shortcuts worth learning.

---

## The shortcut sheet

Snakie will show you its own shortcuts. In the desktop app, choose **Help ▸ Keyboard Shortcuts**, or press <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>/</kbd>.

![Snakie's keyboard shortcut sheet](/learn/snakie_code_tools/assets/shortcuts.jpg){:class="img-fluid w-100"}

The note at the bottom is worth reading: the sheet is *generated from the application menu, so it can't drift from the real bindings*. If it's on the sheet, it works.

On a Mac, every <kbd>Ctrl</kbd> below is <kbd>Cmd</kbd>.

## Files

| Shortcut | Does |
|---|---|
| <kbd>Ctrl</kbd>+<kbd>N</kbd> | New file |
| <kbd>Ctrl</kbd>+<kbd>O</kbd> | Open a file |
| <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>O</kbd> | Open a folder |
| <kbd>Ctrl</kbd>+<kbd>S</kbd> | Save |
| <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>S</kbd> | Save as |
| <kbd>Ctrl</kbd>+<kbd>P</kbd> | Export to PDF |
| <kbd>Ctrl</kbd>+<kbd>W</kbd> | Close the tab |
{:class="table table-single"}

The **File** menu also has **Open Recent**, a list of the folders you've worked in lately.

## Getting around

| Shortcut | Does |
|---|---|
| <kbd>Ctrl</kbd>+<kbd>1</kbd> | Blocks workspace |
| <kbd>Ctrl</kbd>+<kbd>2</kbd> | Code workspace |
| <kbd>Ctrl</kbd>+<kbd>3</kbd> | Electronics workspace |
| <kbd>Ctrl</kbd>+<kbd>4</kbd> | Build workspace |
| <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>B</kbd> | Show or hide the Board View |
| <kbd>Ctrl</kbd>+<kbd>,</kbd> | Settings |
{:class="table table-single"}

Flipping between Code and Electronics with <kbd>Ctrl</kbd>+<kbd>2</kbd> and <kbd>Ctrl</kbd>+<kbd>3</kbd> is the one that changes how you work: check a pin number, flip back, type it.

## Running

| Shortcut | Does |
|---|---|
| <kbd>Ctrl</kbd>+<kbd>R</kbd> | Run |
| <kbd>Ctrl</kbd>+<kbd>.</kbd> | Stop |
{:class="table table-single"}

The **Device** menu holds the rest: **Connect**, **Disconnect**, **Soft Reset** and **Sync Now**.

Careful with <kbd>Ctrl</kbd>+<kbd>.</kbd> It stops a running program from the menu, and it opens the lightbulb when your cursor is in the editor on a line with a fix.

## In the editor

These belong to the editor itself, so they work in the browser version too.

| Shortcut | Does | Lesson |
|---|---|---|
| <kbd>Ctrl</kbd>+<kbd>Space</kbd> | Show suggestions | 1 |
| <kbd>F12</kbd> | Go to definition | 2 |
| <kbd>Ctrl</kbd>+<kbd>F</kbd> | Find and Replace | 3 |
| <kbd>Ctrl</kbd>+<kbd>F2</kbd> | Change all occurrences | 3 |
| <kbd>Ctrl</kbd>+<kbd>.</kbd> | Quick fix | 5 |
| <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>R</kbd> | Refactor | 5 |
| <kbd>Ctrl</kbd>+<kbd>/</kbd> | Comment or uncomment the line | |
| <kbd>Ctrl</kbd>+<kbd>Z</kbd> | Undo | |
{:class="table table-single"}

<kbd>Ctrl</kbd>+<kbd>/</kbd> deserves a mention. Select a block of lines and press it to comment them all out, then again to bring them back. It's the fastest way to switch a piece of code off while you hunt for a bug.

## The Tools menu

**Tools** is the way to parts of Snakie that don't have a button on the main screen:

| Item | What it opens |
|---|---|
| **Firmware Flasher…** | Install or update MicroPython on a board |
| **Board Finder…** | Identify a board and find its details |
| **Parts Catalog…** | The full-screen parts library from the Electronics course |
| **Sprite Editor…** | Draw images and animations for small displays |
| **Find & Replace…** | Lesson 3 |
| **Settings…** | Themes, editor options and chat setup |
{:class="table table-single"}

## The status bar

The strip along the bottom of the window is easy to ignore and worth a glance:

- On the left, the **connection status**, then a rotating **tip**. The tips are real features, one line each. Many people find their favourite tool there
- On the right, the number of **lines** in the file, whether it's **Saved** or **Unsaved**, the Snakie **version** (click it to check for updates) and **Flash firmware**

## Make it yours

Open **Settings** and look at the **Editor** tab. You can change the ruled-paper background to dots or nothing, set the line spacing, pick an editor theme (Paper, Bright or Midnight) and hide the mini-map. None of it changes your code. All of it changes how long you're comfortable looking at it.

![The Editor tab in Settings](/learn/snakie_code_tools/assets/settings_editor.jpg){:class="img-fluid w-100"}

## Five to learn first

If you only memorise five:

1. <kbd>Ctrl</kbd>+<kbd>S</kbd> Save
2. <kbd>Ctrl</kbd>+<kbd>R</kbd> Run
3. <kbd>Ctrl</kbd>+<kbd>Space</kbd> Suggestions
4. <kbd>Ctrl</kbd>+<kbd>/</kbd> Comment
5. <kbd>Ctrl</kbd>+<kbd>F</kbd> Find

---

## Try it Yourself

1. Open the shortcut sheet and find one shortcut that isn't in this lesson's "five to learn first".
2. Write three lines, select them, and toggle them off and on with <kbd>Ctrl</kbd>+<kbd>/</kbd>.
3. Run and stop a program without touching the mouse.

## Common Issues

- **Problem**: A shortcut does nothing.
- **Solution**: Click in the Snakie window first. In the browser, some shortcuts such as <kbd>Ctrl</kbd>+<kbd>N</kbd> and <kbd>Ctrl</kbd>+<kbd>W</kbd> belong to the browser itself, so use the buttons.
- **Why**: A browser keeps certain keys for its own tabs and windows.

- **Problem**: <kbd>Ctrl</kbd>+<kbd>.</kbd> opened a menu instead of stopping my program.
- **Solution**: Click the **Stop** button, or click outside the editor first.
- **Why**: With the cursor in the editor, that key belongs to the quick-fix lightbulb.

- **Problem**: I can't find the menus.
- **Solution**: In the desktop app they're at the top of the window, or the top of the screen on a Mac. In the browser, use the toolbar and the activity bar.
- **Why**: The full menu bar is part of the desktop app.

## Check your understanding

{% quiz %}
? How do you open Snakie's shortcut sheet?
- [x] Help ▸ Keyboard Shortcuts
- [ ] File ▸ Print
- [ ] Right-click the console
> Or press Ctrl+Shift+/.

? What does Ctrl+/ do in the editor?
- [ ] Divides two numbers
- [ ] Opens Find
- [x] Comments or uncomments the selected lines
> It's the quickest way to switch code off and on while debugging.

? Which shortcut switches to the Electronics workspace?
- [ ] Ctrl+E
- [x] Ctrl+3
- [ ] F3
> Ctrl+1 to Ctrl+4 are Blocks, Code, Electronics and Build.
{% endquiz %}
