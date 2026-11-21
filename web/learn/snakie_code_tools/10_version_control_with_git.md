---
layout: lesson
title: Version Control with Git
author: Kevin McAleer
type: page
cover: /learn/snakie_code_tools/assets/banners/10_version_control_with_git.jpg
date: 2026-11-21
previous: 09_compiling_to_mpy.html
next: 11_ai_chat_and_suggestions.html
description: Keep a history of your MicroPython project with Snakie's built-in Source
  Control panel, from the first commit to reading a diff and discarding a bad change
percent: 66
duration: 6
quiz_total: 42
date_updated: 2026-11-21
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

> **Desktop app**: Source Control is in the Snakie desktop app. You also need [Git](https://git-scm.com) installed on your computer.

Every maker has done it. The robot worked on Tuesday. You "improved" it on Wednesday. Now it doesn't work and you can't remember what you changed.

**Git** is the cure. It keeps a history of your project as a series of snapshots called **commits**, and it can show you exactly what changed between any two of them.

---

## Start a repository

Open your project folder in the Files panel, then click **Source** in the activity bar.

If the folder isn't under Git yet, the panel says so and offers **Initialise Repository**. Click it. Snakie asks you to confirm, then does two things:

- Creates the repository (a hidden `.git` folder that holds the history)
- Adds a starter `.gitignore`, if you don't already have one

![A new repository with every file untracked](/learn/snakie_code_tools/assets/git_untracked.jpg){:class="img-fluid w-100"}

Nothing has been committed. Snakie says so: *Created an empty repository on main, added a .gitignore — 5 files ready for a first commit.* The chip at the top reads **main · no commits yet**, and every file is listed under **UNTRACKED**, marked **U**.

## The .gitignore

Open the `.gitignore` Snakie wrote. It lists files Git should leave out of the history: desktop clutter like `.DS_Store`, Python's `__pycache__`, editor scratch files and `.bak` rescue copies.

Everything that *is* your project stays in: your `.py` files, `robot.yml`, URDF models and their meshes. The file is yours to edit.

## Your first commit

1. Type a message in the box at the top: `First version of the obstacle robot`
2. Click **Commit**, or press <kbd>Ctrl</kbd>+<kbd>Enter</kbd>

![A clean working tree after the first commit](/learn/snakie_code_tools/assets/git_clean.jpg){:class="img-fluid w-100"}

The panel now says **Committed.** and *No changes. Working tree is clean.* That snapshot is safe. Whatever you do next, you can get back to it.

When nothing is staged, **Commit** includes every change. That's the simple way to work, and it's fine for a one-person project.

## See what changed

Edit `main.py`. Change `TOO_CLOSE_MM = 200` to `150` and save. Back in Source Control, the file appears under **CHANGES**, marked **M** for modified. Click it.

![The diff for a one-line change](/learn/snakie_code_tools/assets/git_diff.jpg){:class="img-fluid w-100"}

This is a **diff**. The red line, starting with `-`, is what was there. The green line, starting with `+`, is what's there now:

```
-TOO_CLOSE_MM = 200
+TOO_CLOSE_MM = 150
```

This is the answer to "what did I change?". When a robot stops working, the bug is somewhere in the diff. That narrows a 300-line program down to the three lines you touched.

## Choosing what goes in

Each changed file has two small buttons:

| Button | Name | What it does |
|---|---|---|
| **＋** | Stage | Mark this file to go into the next commit |
| **⨯** | Discard changes | Throw away your edits and restore the last committed version |
{:class="table table-single"}

**Staging** lets you commit some changes and not others. Fixed a bug and also started a new feature? Stage the fix, commit it with a clear message, and leave the rest for later. Staged files move to **STAGED CHANGES**, where a **−** button unstages them.

**Discard** is the undo for a whole afternoon. It can't itself be undone, so read the diff first.

## Messages worth writing

A commit message is a note to yourself in three months. Say what changed and why.

| Weak | Better |
|---|---|
| `update` | `Stop sooner: 150 mm` |
| `fix` | `Fix motors running after stop` |
| `stuff` | `Add reverse when too close` |
{:class="table table-single"}

Commit small and often. *Got the servo sweeping* is a perfectly good commit.

## Branches and GitHub

The **BRANCH** drop-down shows which line of history you're on, and switches between them. A branch is a safe place to try an idea without disturbing the version that works.

**Publish to GitHub** creates a repository on GitHub from this folder and pushes your history to it. That gives you a backup away from your computer, and a link you can share. Once published, the panel shows how many commits you are ahead of or behind the copy on GitHub.

The status bar at the bottom of the window shows a **⎇** count of changed files, wherever you are in the app.

## What Git doesn't see

Git tracks the folder on your **computer**. Files that only exist on the board aren't in it. Keep the real copy of your code in the project folder and upload from there, and every change will show up in Source Control.

---

## Try it Yourself

1. Initialise a repository in a project folder and make a first commit.
2. Change two files. Stage only one, commit it, then commit the other with its own message.
3. Break something on purpose and save. Find it in the diff, then use **Discard changes** to put it right.

## Common Issues

**Problem**: Source isn't in the activity bar.
**Solution**: Use the Snakie desktop app.
**Why**: Git needs access to your computer's files, which a browser tab doesn't have.

**Problem**: The panel doesn't show a change I just made.
**Solution**: Save the file, then click the **⟳** refresh button at the top of the panel.
**Why**: Git compares files on disk. An unsaved edit hasn't reached the disk yet.

**Problem**: Commit fails asking who I am.
**Solution**: Tell Git once, in a terminal: `git config --global user.name "Your Name"` and `git config --global user.email "you@example.com"`.
**Why**: Every commit records its author.

## Check your understanding

{% quiz %}
? What is a commit?
- [ ] A copy of the file on the board
- [x] A saved snapshot of your project with a message
- [ ] An error message
> Each commit is a point in history you can compare with or return to.

? In a diff, what does a line starting with - mean?
- [ ] The line was added
- [ ] The line has an error
- [x] The line was removed or replaced
> Minus and red is the old version. Plus and green is the new one.

? What does Discard changes do to a file?
- [x] Restores it to the last committed version
- [ ] Deletes it from the project
- [ ] Uploads it to the board
> It throws away your uncommitted edits, so check the diff first.
{% endquiz %}
