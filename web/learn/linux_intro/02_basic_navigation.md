---
layout: lesson
title: Basic Command Line Navigation
author: Kevin McAleer
type: page
cover: /learn/linux_intro/assets/ls-l.png
date: 2024-08-23
previous: 01a_man.html
next: 03_working_with_files.html
description: Learn how to navigate the filesystem using basic commands like pwd, ls,
  and cd.
percent: 15
duration: 3
navigation:
- name: Introduction to the Linux Command Line on Raspberry Pi OS
- content:
  - section: Getting Started with the Command Line
    content:
    - name: Introduction to the Terminal
      link: 01_intro_terminal.html
    - name: Getting Help with `man`
      link: 01a_man.html
    - name: Basic Command Line Navigation
      link: 02_basic_navigation.html
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
  - section: Resources and Next Steps
    content:
    - name: Resources and Next Steps
      link: 18_resources_next_steps.html
    - name: Linux Command Line Cheatsheet
      link: 19_cheatsheet.html
---


![cover image]({{page.cover}}){:class="cover"}

## Introduction

Navigating the filesystem is a fundamental skill when working with the command line. In this lesson, you'll learn how to use basic commands to find your way around the Raspberry Pi OS filesystem.

---

## Learning Objectives

- Understand the structure of the Raspberry Pi OS filesystem.
- Learn how to use the `pwd`, `ls`, and `cd` commands.
- Differentiate between absolute and relative paths.

---

### Understanding the Filesystem Structure

The Linux filesystem is organized in a hierarchical structure, starting from the root directory (`/`). Every file and directory is nested within this structure. Here's a basic overview:

- `/` : Root directory, the top level of the filesystem.
- `/home` : Contains user directories, like `/home/pi` for the Raspberry Pi user.
- `/etc` : Configuration files for the system.
- `/var` : Variable data, such as logs and temporary files.
- `/usr` : User programs and data.
- `/bin` : Essential system binaries.
- `/lib` : Shared libraries.
- `/opt` : Optional software packages.
- `/tmp` : Temporary files.

---

### The `pwd` Command

The `pwd` (print working directory) command displays the current directory you are in. This is useful to know where you are in the filesystem.

```bash
pwd
```

![pwd command](/learn/linux_intro/assets/pwd.png){:class="w-100 img-fluid card-hover card-shadow rounded-3"}

---

## The `ls` Command

The `ls` (list) command lists the contents of a directory. You can use it to see files and subdirectories within your current directory.

``` bash
ls
```

![ls command](/learn/linux_intro/assets/ls.png){:class="w-100 img-fluid card-hover card-shadow rounded-3"}

You can also use options with ls to get more detailed information:

```bash
ls -l
```

![ls -l command](/learn/linux_intro/assets/ls-l.png){:class="w-100 img-fluid card-hover card-shadow rounded-3"}

---

## The `cd` Command

The `cd` (change directory) command allows you to navigate between directories. For example, to move into the `Documents` directory, you would type:

```bash
cd Documents
```

![cd command](/learn/linux_intro/assets/cd.png){:class="w-100 img-fluid card-hover card-shadow rounded-3"}

To return to the home directory, simply type:

```bash
cd $HOME
```

![cdhome command](/learn/linux_intro/assets/cdhome.png){:class="w-100 img-fluid card-hover card-shadow rounded-3"}

To back up one directory, use:

```bash
cd ..
```

![cd  .. command](/learn/linux_intro/assets/cddotdot.png){:class="w-100 img-fluid card-hover card-shadow rounded-3"}

---

## Absolute vs. Relative Paths

- **Absolute Path:** A path that starts from the root directory `/`, e.g., `/home/pi/Documents`.
- **Relative Path:** A path relative to the current directory, e.g., `Documents` if you are in `/home/pi`.

---

## Clearing the console

To clear the terminal screen, you can use the `clear` command:

        clear

This will clear the terminal screen, making it easier to read the output of new commands.

---

## Summary

In this lesson, you learned how to navigate the filesystem using the `pwd`, `ls`, and `cd` commands, and you now understand the difference between absolute and relative paths. These are essential skills for working efficiently in the command line.

---
