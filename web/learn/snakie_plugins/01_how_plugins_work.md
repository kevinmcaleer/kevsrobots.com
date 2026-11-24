---
layout: lesson
title: How Plugins Work
author: Kevin McAleer
type: page
cover: /learn/snakie_plugins/assets/banners/01_how_plugins_work.jpg
date: 2026-11-21
previous: 00_intro.html
next: 02_the_plugin_rack.html
description: What a Snakie plugin is, how Snakie runs it in a separate Python process,
  and what plugins can and cannot do
percent: 14
duration: 5
quiz_total: 36
date_updated: 2026-11-24
navigation:
- name: Extend Snakie with Python Plugins
- content:
  - section: Overview
    content:
    - name: Extend Snakie with Python Plugins
      link: 00_intro.html
    - name: How Plugins Work
      link: 01_how_plugins_work.html
    - name: The Plugin Rack
      link: 02_the_plugin_rack.html
  - section: Commands
    content:
    - name: Your First Plugin
      link: 03_your_first_plugin.html
    - name: When Plugins Break
      link: 04_when_plugins_break.html
    - name: Commands That Read Code
      link: 05_commands_that_read_code.html
    - name: Commands That Change Code
      link: 06_commands_that_change_code.html
  - section: Linters
    content:
    - name: Your First Linter
      link: 07_your_first_linter.html
    - name: Quick Fixes
      link: 08_quick_fixes.html
    - name: Rules for Your Hardware
      link: 09_rules_for_your_hardware.html
  - section: Blocks and Sharing
    content:
    - name: Blocks from a Plugin
      link: 10_blocks_from_a_plugin.html
    - name: Sharing Your Plugin
      link: 11_sharing_your_plugin.html
    - name: Summary and Next Steps
      link: 12_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to explain what a Snakie plugin is, how Snakie runs it in a separate Python process, and what plugins can and cannot do.

## Two programs talking

When you open the Snakie desktop app, it quietly starts a second program: a **plugin host**. The host is plain Python, run by the Python 3 that's installed on your computer.

The host does three jobs:

1. It looks in a folder for plugins
2. It loads each one
3. It waits for Snakie to ask for something

```
 Snakie (the editor)                  Plugin host (Python)
+---------------------+   request   +----------------------+
| "run the command    | ----------> | finds your function  |
|  called clubkit.hi" |             | and calls it         |
|                     | <---------- |                      |
| shows the message   |   answer    | returns a message    |
+---------------------+             +----------------------+
```

Snakie sends a request such as "run this command on this file". The host calls your Python function and sends back the answer. Snakie then acts on it.

---

## Your plugin never touches the editor

This is the most important idea in the course. Your function doesn't reach into Snakie and change things. It **returns a description of what it wants**, and Snakie does the work.

| Your function returns | Snakie does |
|---|---|
| a message | shows it in the Plugins panel |
| an edit | replaces the text of the open file |
| a status | shows it in the status bar |
| a list of problems | draws squiggles in the editor |
| a list of blocks | adds them to the Blocks toolbox |
{:class="table table-single"}

That keeps plugins simple. A command is a function that takes the current file and returns one or more of those things. You can understand every plugin in this course by asking "what goes in, and what comes back?"

---

## What goes in

Every command and linter receives a **context**, usually called `ctx`. It describes the file that's open in the editor.

| Value | What it holds |
|---|---|
| `ctx.file.name` | the file name, such as `patrol.py` |
| `ctx.file.path` | where the file is saved |
| `ctx.file.source` | `"local"` for your computer, `"device"` for a file on a board |
| `ctx.file.content` | all the text in the editor, as one string |
{:class="table table-single"}

`ctx.file.content` is the one you'll use most. It's the text as it is in the editor right now, including changes you haven't saved.

---

## What plugins can't do

A plugin can't talk to the board. It doesn't run your MicroPython, flash firmware or read sensors. It works on **text**: the program in the editor.

That's less limiting than it sounds. Most mistakes can be spotted by reading the code, and that's exactly what a plugin is good at.

---

## Where plugins live

Snakie loads plugins from a folder in your home folder called `.snakie/plugins`.

| System | Folder |
|---|---|
| Windows | `C:\Users\<you>\.snakie\plugins` |
| macOS | `/Users/<you>/.snakie/plugins` |
| Linux | `/home/<you>/.snakie/plugins` |
{:class="table table-single"}

A plugin is either a single file in there, such as `clubkit.py`, or a folder with an `__init__.py` inside, such as `clubkit/__init__.py`. We'll use a folder so there's room to grow.

The folder name starts with a dot, so your file manager may hide it. On macOS press **Cmd + Shift + .** in Finder. On Windows turn on **Hidden items** in the View menu. On Linux press **Ctrl + H**.

---

## Why a separate program?

Three good reasons:

- **It's real Python.** Your plugin can use anything in Python's standard library, such as `re` for patterns
- **A crash stays put.** If your plugin has a bug, the host reports it. The editor carries on
- **It's clear who runs what.** Plugin code runs on your computer as you. Snakie tells you so at the top of the Plugins panel

> **Teachers**: Plugins run as the logged-in user, using the Python 3 installed on the computer, and load from `.snakie/plugins` in the home folder. On shared or locked-down machines check that learners can create and keep files there, and that their plugins will still be there next session.

---

## Try it Yourself

1. Open a terminal and type `python3 --version` (or `python --version` on Windows). You should see a version number starting with 3
2. Find your home folder and look for `.snakie`. If Snakie has run before, it may already be there
3. Create the `plugins` folder inside it if it's missing

---

## Common Issues

**Problem**: `python3` isn't recognised.
**Solution**: Install Python 3 from [python.org](https://www.python.org/downloads/). On Windows, tick **Add Python to PATH** in the installer.
**Why**: Snakie looks for `python3`, then `python`, on your system's PATH. No Python means no plugin host.

**Problem**: I can't see the `.snakie` folder.
**Solution**: Show hidden files, or create it yourself. In a terminal, `mkdir -p ~/.snakie/plugins` works on macOS and Linux.
**Why**: Names starting with a dot are hidden by default.

---

## Check your understanding

{% quiz %}
? What runs your plugin code?
- [ ] The MicroPython board
- [x] A Python host program on your computer
- [ ] The web browser
> Snakie starts a Python host on your computer and sends it requests.

? How does a command change the text in the editor?
- [ ] It writes to the file on disk
- [x] It returns an edit and Snakie applies it
- [ ] It presses keys in the editor
> Plugins return descriptions of what they want. Snakie does the work.

? Which value holds the text of the open file?
- [ ] ctx.file.name
- [ ] ctx.file.path
- [x] ctx.file.content
> ctx.file.content is the whole file as one string, including unsaved changes.
{% endquiz %}
