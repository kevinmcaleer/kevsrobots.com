---
layout: lesson
title: Introduction to Git
author: Kevin McAleer
type: page
cover: /learn/linux_intro/assets/banners/14_intro_git.jpg
date: 2024-08-23
previous: 13_basic_scripting_constructs.html
next: 15_basic_git_commands.html
description: Learn what Git does for your scripts, install it on Raspberry Pi OS,
  and set your name, email and default branch.
percent: 68
duration: 5
quiz_total: 85
date_updated: 2026-11-24
navigation:
- name: Introduction to the Linux Command Line on Raspberry Pi OS
- content:
  - section: Getting Started with the Command Line
    content:
    - name: Course Overview
      link: 00_intro.html
    - name: Getting Help with `man`
      link: 01a_man.html
    - name: Basic Command Line Navigation
      link: 02_basic_navigation.html
    - name: Managing Users and Using `sudo`
      link: 02a_user.html
  - section: Managing Files and Directories
    content:
    - name: Working with Files
      link: 03_working_with_files.html
    - name: Working with Directories
      link: 04_working_with_directories.html
    - name: File Permissions and Ownership
      link: 05_file_permissions.html
  - section: Managing Software
    content:
    - name: Installing Software
      link: 06_installing_software.html
    - name: Managing Software
      link: 07_managing_software.html
  - section: System Monitoring and Management
    content:
    - name: Monitoring System Performance
      link: 08_monitoring_performance.html
    - name: Managing Processes
      link: 09_managing_processes.html
    - name: Services and systemctl
      link: 09a_services.html
  - section: Networking Basics
    content:
    - name: Networking Commands
      link: 10_networking_commands.html
    - name: Transferring Files Over a Network
      link: 11_transferring_files.html
  - section: Scripting Basics
    content:
    - name: Introduction to Shell Scripting
      link: 12_intro_scripting.html
    - name: Basic Scripting Constructs
      link: 13_basic_scripting_constructs.html
  - section: Working with Git (Optional Advanced Topic)
    content:
    - name: Introduction to Git
      link: 14_intro_git.html
    - name: Basic Git Commands
      link: 15_basic_git_commands.html
  - section: 'Final Project: Building a Simple Command Line Application'
    content:
    - name: Final Project Overview
      link: 16_final_project_overview.html
    - name: Final Project Steps
      link: 17_final_project_steps.html
  - section: Summary and Next Steps
    content:
    - name: Summary and Next Steps
      link: 18_resources_next_steps.html
    - name: Linux Command Line Cheatsheet
      link: 19_cheatsheet.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to explain what Git is for, install it on your Pi, and set up your name and email so it can label your work.

## An undo button for your scripts

Have you ever edited a script, broken it, and wished you could go back to yesterday's version? **Git** is a tool that remembers every saved version of your files, who changed them and why. It is the same tool used by millions of programmers, and it is how most Docker projects, robot code and websites (including this one) are shared.

You do not need an account anywhere to use Git. It works entirely on your Pi, and later you can put a copy on a site such as GitHub if you want to share it. For our Docker-ready Pi, it means that the scripts and settings in `~/homelab` can be tracked and rolled back, so you can experiment without fear.

```mermaid
flowchart LR
    W["Working folder: the files you edit"] -->|"git add"| S["Staging area: what goes in the next snapshot"]
    S -->|"git commit"| R["Repository: the saved history"]
    R -->|"git checkout / restore"| W
```

Git has three places, shown above. You edit files in the working folder, choose which changes to include with `git add`, and save a labelled snapshot (a **commit**) with `git commit`. The next lesson practises these.

## Installing Git

Check whether you already have it:

```bash
$ git --version
git version 2.x.x
```

If it says `command not found`, install it with the package manager from lesson 6:

```bash
$ sudo apt update
$ sudo apt install git
```

Run `git --version` again to confirm. The exact number does not matter.

## Telling Git who you are

Every commit is labelled with a name and an email. Set them once for your user:

```bash
$ git config --global user.name "Your Name"
$ git config --global user.email "you@example.com"
$ git config --global init.defaultBranch main
```

The third line makes new repositories start on a branch called `main`, which avoids a hint message you would otherwise see. Check what you saved:

```bash
$ git config --global --list
user.name=Your Name
user.email=you@example.com
init.defaultBranch=main
```

This is stored in a file called `~/.gitconfig`. You can read it with `cat ~/.gitconfig`.

> **Careful**: your commit email can become public if you push your work to a website. If that worries you, use an address your Git hosting site gives you for this, such as GitHub's `noreply` address, or any made-up address, since Git does not check it.

> **Teachers**: do not ask pupils to use personal email addresses for Git. Git does not verify the address, so a class code or a made-up address like `student1@example.com` works fine for local use. Only create accounts on sites such as GitHub with your school's agreement, and check their minimum age rules before anyone signs up. Everything in this course works with no account at all.

Pi progress: Git is installed and knows who you are. Next we put `~/homelab/scripts` under version control.

## Try it Yourself

1. Run `git --version`, and if needed install Git with `apt`.
2. Set your name and email, then use `git config --global user.name` to read the name back.
3. Open `~/.gitconfig` with `cat` and find the three lines you set.
4. Challenge: run `git help commit` (it uses the manual page viewer) and find what `-m` does.

## Common Issues

- **Problem**: `git: command not found`.
- **Solution**: Run `sudo apt install git`.
- **Why**: Git is not part of every Raspberry Pi OS image.

- **Problem**: Later, Git says `Please tell me who you are`.
- **Solution**: Run the two `git config --global` commands for name and email.
- **Why**: Git refuses to make a commit with no author information.

- **Problem**: I put the wrong name or email in.
- **Solution**: Run the same `git config --global` command again with the correct value.
- **Why**: Setting a value again simply overwrites the old one.

## Check your understanding

{% quiz %}
? What does Git do?
- [ ] Speeds up the Pi
- [x] Remembers every saved version of your files
- [ ] Backs up the SD card automatically
> Git records snapshots of your files, so you can see and restore older versions.

? What is a commit?
- [x] A saved, labelled snapshot of your changes
- [ ] A file permission
- [ ] A kind of branch
> Each commit is a point in history you can return to.

? Which command installs Git on Raspberry Pi OS?
- [ ] git install
- [x] sudo apt install git
- [ ] sudo systemctl enable git
> Git is a normal package, so `apt` installs it.

? Do you need an online account to use Git on your Pi?
- [ ] Yes, you must sign up first
- [x] No, it works locally
- [ ] Only for commits
> An account is only needed if you want to share your work on a hosting site.
{% endquiz %}
