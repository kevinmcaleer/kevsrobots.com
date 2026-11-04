---
title: Course Overview
description: What you'll learn in Git for Beginners
layout: lesson
type: page
cover: /learn/git/assets/cover.jpg
navigation:
- name: Git for Beginners
- content:
  - section: Overview
    content:
    - name: Course Overview
      link: 00_intro.html
  - section: Getting Started
    content:
    - name: What is Git?
      link: 01_what-is-git.html
    - name: Installing Git
      link: 02_installing-git.html
    - name: Your First Repository
      link: 03_your-first-repository.html
  - section: Saving Your Work
    content:
    - name: Staging and Committing
      link: 04_staging-and-committing.html
    - name: Viewing History
      link: 05_viewing-history.html
    - name: Undoing Changes
      link: 06_undoing-changes.html
    - name: Ignoring Files
      link: 07_ignoring-files.html
  - section: Branches
    content:
    - name: Branches
      link: 08_branches.html
    - name: Merging and Conflicts
      link: 09_merging-and-conflicts.html
  - section: Working with GitHub
    content:
    - name: GitHub and Remotes
      link: 10_github-and-remotes.html
    - name: Pulling and Collaborating
      link: 11_pulling-and-collaborating.html
  - section: Putting It All Together
    content:
    - name: 'Project: Version Control a Robot'
      link: 12_project-version-control-a-robot.html
    - name: Cheat Sheet and Next Steps
      link: 13_cheat-sheet-and-next-steps.html
---


![banner]({{page.cover}}){:class="cover"}

## Overview

Welcome to `Git for Beginners`! Have you ever broken a robot's code that was working perfectly five minutes ago, and couldn't remember what you changed? Or ended up with a folder full of files called `robot.py`, `robot_v2.py`, `robot_final.py` and `robot_final_REALLY.py`?

Git fixes all of that. It's a tool that takes snapshots of your project every time you ask it to, so you can see exactly what changed, go back to any earlier version, try out ideas safely, and share your work with other people. It's used by almost every software team on the planet, and it's just as useful for a one-person MicroPython robot project.

This course starts from zero. You don't need to know anything about Git, GitHub or the command line beyond typing a command and pressing Enter.

---

## Course Content

Module | Title                                  | What you'll learn
-------|----------------------------------------|------------------------------------------------------------------
1      | What is Git?                           | Version control, snapshots, repositories, and how Git differs from GitHub
2      | Installing Git                         | Installing Git on Windows, macOS, Linux and Raspberry Pi, and telling it who you are
3      | Your First Repository                  | Creating a repository with `git init` and checking it with `git status`
4      | Staging and Committing                 | The three areas of Git, `git add`, `git commit`, and writing good commit messages
5      | Viewing History                        | `git log`, `git show` and `git diff`
6      | Undoing Changes                        | Throwing away edits, unstaging, fixing the last commit and reverting
7      | Ignoring Files                         | Keeping junk and secrets out of your repository with `.gitignore`
8      | Branches                               | Trying out ideas safely on a branch
9      | Merging and Conflicts                  | Bringing branches together, and fixing merge conflicts
10     | GitHub and Remotes                     | Putting your project on GitHub with `git push`, and `git clone`
11     | Pulling and Collaborating              | `git fetch`, `git pull`, and how pull requests work
12     | Project: Version Control a Robot       | Using everything you've learned on a real MicroPython robot project
13     | Cheat Sheet and Next Steps             | Every command from the course on one page
{:class="table table-striped"}

---

## Key Results

After completing this course, you will be able to:

* Install and set up Git on your computer or Raspberry Pi
* Track a project with Git and save meaningful snapshots (commits)
* Look back through your project's history and see what changed
* Undo mistakes without panicking
* Use branches to try out new features without breaking working code
* Merge branches and fix merge conflicts
* Publish your projects on GitHub and keep them in sync across computers

---

## What you'll need

* A computer running Windows, macOS or Linux, or a Raspberry Pi
* A text editor such as [VS Code](https://code.visualstudio.com) or [Thonny](https://thonny.org)
* A free [GitHub](https://github.com) account (for modules 10 to 12)

---

## Prerequisites

None! If you've never used a terminal before, don't worry. Every command is shown in full, along with what you should see when you run it. If you'd like a gentle introduction to the command line first, take [Introduction to Linux](/learn/linux_intro/).

---

## How the course works

Each module introduces one new idea, shows you the commands, and explains what they do. Commands you type into a terminal look like this:

```bash
git status
```

What Git prints back looks like this:

```text
On branch main
nothing to commit, working tree clean
```

Most modules end with a **Try it yourself** exercise, a **Common issues** section for when things go wrong, and a short quiz. Your quiz score and progress are saved in your browser and shown in the sidebar.

We'll build up a small robot project as we go, so by the end you'll have a real repository on GitHub with a proper history.

Let's get started!

---
